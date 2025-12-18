"""
Database module for conversation persistence.

Provides async connection pool management and functions for storing/retrieving
conversation history using Neon Postgres via asyncpg.

T025 Implementation - Spec-3 RAG Agent API
"""

import os
import json
import uuid
from datetime import datetime
from typing import Any
from contextlib import asynccontextmanager

import structlog

# Optional asyncpg import - gracefully handle if not available
try:
    import asyncpg
    ASYNCPG_AVAILABLE = True
except ImportError:
    asyncpg = None  # type: ignore
    ASYNCPG_AVAILABLE = False

logger = structlog.get_logger(__name__)

# Configuration
DATABASE_URL = os.getenv("DATABASE_URL")
MAX_CONVERSATION_HISTORY = 20  # 10 conversation turns (user + assistant pairs)
MAX_CONTEXT_TOKENS = 8000

# Connection pool singleton
_pool: "asyncpg.Pool | None" = None


async def init_pool() -> "asyncpg.Pool | None":
    """
    Initialize the asyncpg connection pool.

    Returns:
        Connection pool if DATABASE_URL is set, None otherwise.
    """
    global _pool

    if not ASYNCPG_AVAILABLE:
        logger.warning("asyncpg not installed - conversation persistence disabled")
        return None

    if not DATABASE_URL:
        logger.info("DATABASE_URL not set - conversation persistence disabled")
        return None

    if _pool is not None:
        return _pool

    try:
        _pool = await asyncpg.create_pool(
            DATABASE_URL,
            min_size=1,
            max_size=10,
            command_timeout=30,
        )
        logger.info("Database connection pool initialized")

        # Create tables if they don't exist
        await _ensure_schema()

        return _pool
    except Exception as e:
        logger.error("Failed to initialize database pool", error=str(e))
        return None


async def close_pool() -> None:
    """Close the connection pool gracefully."""
    global _pool

    if _pool is not None:
        await _pool.close()
        _pool = None
        logger.info("Database connection pool closed")


async def _ensure_schema() -> None:
    """Create the conversations table and indexes if they don't exist."""
    if _pool is None:
        return

    create_table_sql = """
    CREATE TABLE IF NOT EXISTS conversations (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        session_id VARCHAR(255) NOT NULL,
        query TEXT NOT NULL,
        response TEXT NOT NULL,
        sources JSONB DEFAULT '[]'::jsonb,
        mode VARCHAR(50) NOT NULL DEFAULT 'general',
        selected_text TEXT,
        chunks_retrieved INTEGER DEFAULT 0,
        latency_ms FLOAT DEFAULT 0.0,
        created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
    );

    CREATE INDEX IF NOT EXISTS idx_conversations_session_id
        ON conversations(session_id);

    CREATE INDEX IF NOT EXISTS idx_conversations_session_created
        ON conversations(session_id, created_at DESC);
    """

    try:
        async with _pool.acquire() as conn:
            await conn.execute(create_table_sql)
        logger.info("Database schema verified/created")
    except Exception as e:
        logger.error("Failed to ensure schema", error=str(e))


@asynccontextmanager
async def get_connection():
    """Get a connection from the pool with automatic release."""
    if _pool is None:
        yield None
        return

    conn = await _pool.acquire()
    try:
        yield conn
    finally:
        await _pool.release(conn)


async def save_conversation(
    session_id: str,
    query: str,
    response: str,
    sources: list[dict[str, Any]],
    mode: str,
    selected_text: str | None = None,
    chunks_retrieved: int = 0,
    latency_ms: float = 0.0,
) -> str | None:
    """
    Save a conversation exchange to the database.

    Args:
        session_id: Client-provided session identifier
        query: User's question
        response: Agent's answer
        sources: Array of SourceCitation objects
        mode: "general" or "selected_text"
        selected_text: User-provided text (for selected_text mode)
        chunks_retrieved: Number of chunks considered
        latency_ms: Response time in milliseconds

    Returns:
        The conversation ID if saved, None if database unavailable.
    """
    if _pool is None:
        logger.debug("Database not available - skipping conversation save")
        return None

    insert_sql = """
    INSERT INTO conversations
        (session_id, query, response, sources, mode, selected_text, chunks_retrieved, latency_ms)
    VALUES
        ($1, $2, $3, $4::jsonb, $5, $6, $7, $8)
    RETURNING id;
    """

    try:
        async with get_connection() as conn:
            if conn is None:
                return None

            result = await conn.fetchval(
                insert_sql,
                session_id,
                query,
                response,
                json.dumps(sources),
                mode,
                selected_text,
                chunks_retrieved,
                latency_ms,
            )

            logger.debug(
                "Conversation saved",
                conversation_id=str(result),
                session_id=session_id,
            )
            return str(result)

    except Exception as e:
        logger.error(
            "Failed to save conversation",
            error=str(e),
            session_id=session_id,
        )
        return None


async def get_conversation_history(
    session_id: str,
    limit: int = MAX_CONVERSATION_HISTORY,
) -> list[dict[str, Any]]:
    """
    Retrieve conversation history for a session.

    Args:
        session_id: Client-provided session identifier
        limit: Maximum number of conversations to retrieve (default: 10)

    Returns:
        List of conversation records, ordered oldest to newest.
        Each record contains: query, response, mode, created_at
    """
    if _pool is None:
        logger.debug("Database not available - returning empty history")
        return []

    # Enforce maximum limit
    limit = min(limit, MAX_CONVERSATION_HISTORY)

    select_sql = """
    SELECT query, response, mode, sources, created_at
    FROM conversations
    WHERE session_id = $1
    ORDER BY created_at DESC
    LIMIT $2;
    """

    try:
        async with get_connection() as conn:
            if conn is None:
                return []

            rows = await conn.fetch(select_sql, session_id, limit)

            # Reverse to get oldest first (for context building)
            history = [
                {
                    "query": row["query"],
                    "response": row["response"],
                    "mode": row["mode"],
                    "sources": row["sources"] if row["sources"] else [],
                    "created_at": row["created_at"].isoformat() if row["created_at"] else None,
                }
                for row in reversed(rows)
            ]

            logger.debug(
                "Retrieved conversation history",
                session_id=session_id,
                count=len(history),
            )
            return history

    except Exception as e:
        logger.error(
            "Failed to retrieve conversation history",
            error=str(e),
            session_id=session_id,
        )
        return []


def truncate_context_to_token_limit(
    messages: list[dict[str, str]],
    max_tokens: int = MAX_CONTEXT_TOKENS,
) -> list[dict[str, str]]:
    """
    Truncate conversation context to fit within token limit.

    Uses a simple character-based estimation (4 chars ~= 1 token).
    Keeps the most recent messages when truncation is needed.

    Args:
        messages: List of message dicts with 'role' and 'content' keys
        max_tokens: Maximum allowed tokens (default: 8000)

    Returns:
        Truncated list of messages fitting within the token limit.
    """
    if not messages:
        return []

    # Simple estimation: 4 characters ~= 1 token
    chars_per_token = 4
    max_chars = max_tokens * chars_per_token

    # Calculate total characters
    total_chars = sum(
        len(msg.get("content", "")) + len(msg.get("role", ""))
        for msg in messages
    )

    if total_chars <= max_chars:
        return messages

    # Truncate from the beginning (keep recent context)
    truncated = []
    current_chars = 0

    for msg in reversed(messages):
        msg_chars = len(msg.get("content", "")) + len(msg.get("role", ""))
        if current_chars + msg_chars > max_chars:
            break
        truncated.insert(0, msg)
        current_chars += msg_chars

    logger.debug(
        "Context truncated",
        original_messages=len(messages),
        truncated_messages=len(truncated),
        estimated_tokens=current_chars // chars_per_token,
    )

    return truncated


async def is_database_available() -> bool:
    """
    Check if database connection is available.

    Returns:
        True if database is connected and responsive, False otherwise.
    """
    if _pool is None:
        return False

    try:
        async with get_connection() as conn:
            if conn is None:
                return False
            await conn.fetchval("SELECT 1")
            return True
    except Exception:
        return False


async def get_database_latency() -> float | None:
    """
    Measure database response latency.

    Returns:
        Latency in milliseconds, or None if database unavailable.
    """
    if _pool is None:
        return None

    try:
        start = datetime.now()
        async with get_connection() as conn:
            if conn is None:
                return None
            await conn.fetchval("SELECT 1")
        elapsed = (datetime.now() - start).total_seconds() * 1000
        return round(elapsed, 2)
    except Exception:
        return None
