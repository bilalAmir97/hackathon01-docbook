"""RAG Agent using OpenAI Agents SDK with Gemini backend.

This module provides the agent configuration and execution logic for
the RAG Agent API. It integrates with the Spec-2 retrieval pipeline
and uses Gemini for LLM generation via LiteLLM.
"""

from __future__ import annotations

# CRITICAL: Apply Pydantic patch BEFORE any OpenAI SDK imports
import pydantic_patch  # noqa: F401

import asyncio
import os
from typing import Annotated, Any

from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# OpenAI Agents SDK imports
from agents import Agent, Runner, function_tool, set_tracing_disabled
from agents.extensions.models.litellm_model import LitellmModel

# Spec-2 imports
from models import RetrievalRequest
from retrieval import retrieve

# Disable OpenAI tracing (not supported with Gemini backend)
set_tracing_disabled(disabled=True)


# =============================================================================
# System Prompts
# =============================================================================

GENERAL_SYSTEM_PROMPT = """You are a friendly teaching assistant for the Physical AI & Humanoid Robotics textbook.

MANDATORY WORKFLOW:
1. FIRST: Call search_documentation tool with the user's question
2. WAIT for the search results to be returned
3. THEN: Answer ONLY based on the returned chunks
4. Include [Source: page_title] citations for all facts

CRITICAL RULES:
- You MUST call search_documentation before answering ANY question
- NEVER answer from your own knowledge - ONLY use retrieved chunks
- If search returns no results, say "I couldn't find information about that in the documentation"
- ALWAYS cite sources using [Source: page_title] format

TEACHING STYLE - BE CONCISE, FRIENDLY, AND PEDAGOGICAL:
- Keep answers brief (2-4 sentences) to maintain conversational flow
- Explain concepts in simple, easy-to-understand wording
- Use analogies when helpful to clarify complex ideas
- Focus on the core concept, not technical jargon or lengthy details
- Include inline citations [Source: page_title] naturally
- END with a follow-up question to reinforce learning
  Examples: "Does this help clarify [concept]?" or "Would you like to explore [related topic] next?"

RESPONSE STRUCTURE:
1. Direct answer in simple terms
2. Brief analogy or clarification if needed
3. Citation
4. Follow-up question to deepen understanding
"""

SELECTED_TEXT_SYSTEM_PROMPT = """You are analyzing a specific text selection provided by the user.

CRITICAL RULES:
1. ONLY answer based on the provided selected_text below
2. Do NOT use any external knowledge
3. If the answer is NOT in the selection, respond:
   "The provided selection does not contain information about [topic]."

Selected text:
---
{selected_text}
---

Answer the user's question based ONLY on the text above.
"""


# =============================================================================
# Model Provider
# =============================================================================


def get_model() -> LitellmModel:
    """Get configured Gemini model via LiteLLM.

    Returns:
        LitellmModel configured for Gemini with rate limit handling

    Raises:
        ValueError: If GEMINI_API_KEY is not set
    """
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY environment variable not set")

    # Use gemini-2.5-flash as default (stable, good rate limits)
    # Options: gemini/gemini-3.0-flash (latest), gemini/gemini-2.5-flash (stable)
    model_name = os.environ.get("GEMINI_MODEL", "gemini/gemini-2.5-flash")

    return LitellmModel(
        model=model_name,
        api_key=api_key,
    )


# =============================================================================
# Rate Limit Retry Logic
# =============================================================================

MAX_RETRIES = 3
RETRY_DELAYS = [5, 15, 30]  # Exponential backoff delays in seconds


async def run_with_retry(coro_func, *args, **kwargs):
    """Run a coroutine with retry logic for rate limit errors.

    Args:
        coro_func: Async function to call
        *args, **kwargs: Arguments to pass to the function

    Returns:
        Result from the coroutine

    Raises:
        Last exception if all retries fail
    """
    last_exception = None

    for attempt in range(MAX_RETRIES):
        try:
            return await coro_func(*args, **kwargs)
        except Exception as e:
            error_str = str(e).lower()
            # Check if it's a rate limit error
            if "429" in error_str or "rate" in error_str or "quota" in error_str:
                last_exception = e
                if attempt < MAX_RETRIES - 1:
                    delay = RETRY_DELAYS[attempt]
                    import structlog
                    logger = structlog.get_logger()
                    logger.warning(
                        "rate_limit_retry",
                        attempt=attempt + 1,
                        max_retries=MAX_RETRIES,
                        delay_seconds=delay,
                        error=str(e)[:200],
                    )
                    await asyncio.sleep(delay)
                    continue
            # Not a rate limit error, raise immediately
            raise e

    # All retries exhausted
    raise last_exception


# =============================================================================
# Retrieval Tool
# =============================================================================


@function_tool
def search_documentation(
    query: Annotated[str, "Search query for relevant documentation"],
    top_k: Annotated[int, "Number of results to retrieve (1-20)"] = 5,
) -> list[dict[str, Any]]:
    """Search the Physical AI & Humanoid Robotics documentation.

    Returns relevant text chunks with source metadata for citation.
    Use this tool to find information before answering questions.
    """
    # Clamp top_k to valid range
    top_k = max(1, min(top_k, 20))

    # Create retrieval request and execute search
    request = RetrievalRequest(query=query, top_k=top_k)
    result = retrieve(request)

    # Format results for agent consumption
    return [
        {
            "source_url": chunk.source_url,
            "page_title": chunk.page_title,
            "section_heading": chunk.section_heading or "Introduction",
            "chunk_text": chunk.chunk_text[:2000],  # Limit chunk size for context window
            "relevance_score": round(chunk.score, 4),
        }
        for chunk in result.results
    ]


# =============================================================================
# Source Extraction
# =============================================================================


def extract_sources_from_result(result: Any) -> list[dict[str, Any]]:
    """Extract sources from agent run result.

    Iterates through run_items to find tool call outputs
    that contain source information.

    Args:
        result: RunResult from agent execution

    Returns:
        List of source dictionaries from tool calls
    """
    sources: list[dict[str, Any]] = []

    if not hasattr(result, "run_items"):
        return sources

    for item in result.run_items:
        # Check if item has tool output
        if hasattr(item, "output") and isinstance(item.output, list):
            sources.extend(item.output)

    return sources


# =============================================================================
# Agent Execution
# =============================================================================


async def run_agent(
    query: str,
    mode: str,
    selected_text: str | None = None,
    top_k: int = 5,
    score_threshold: float | None = None,
    filters: dict[str, str | list[str]] | None = None,
) -> tuple[str, list[dict[str, Any]]]:
    """Run the agent and return (answer, sources).

    Args:
        query: User's question
        mode: "general" or "selected_text"
        selected_text: Text to constrain answers (required for selected_text mode)
        top_k: Number of chunks to retrieve (general mode only)
        score_threshold: Minimum similarity score (general mode only)
        filters: Metadata filters (general mode only)

    Returns:
        Tuple of (answer_text, list_of_source_dicts)

    Raises:
        ValueError: If mode is selected_text but no selected_text provided
    """
    if mode == "selected_text":
        if not selected_text:
            raise ValueError("selected_text is required for selected_text mode")

        # Create agent without tools for selected-text mode
        agent = Agent(
            name="Selection Assistant",
            instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(selected_text=selected_text),
            tools=[],  # No tools - constrain to provided text only
            model=get_model(),
        )

        # Create synthetic source for the selected text
        sources = [
            {
                "source_url": "selected_text",
                "page_title": "User Selection",
                "section_heading": "Selected Text",
                "chunk_text": selected_text[:500] if selected_text else "",
                "relevance_score": 1.0,
            }
        ]
    else:
        # General mode: manually retrieve chunks BEFORE agent execution
        # This ensures chunks are always retrieved (Gemini doesn't reliably call tools)
        request = RetrievalRequest(
            query=query,
            top_k=top_k,
            score_threshold=score_threshold,
            filters=filters,
        )
        retrieval_result = retrieve(request)

        # Format sources for response
        sources = [
            {
                "source_url": chunk.source_url,
                "page_title": chunk.page_title,
                "section_heading": chunk.section_heading or "Introduction",
                "chunk_text": chunk.chunk_text,
                "relevance_score": round(chunk.score, 4),
            }
            for chunk in retrieval_result.results
        ]

        # Build context from retrieved chunks
        if sources and sources[0]['relevance_score'] > 0.3:  # Only use if reasonably relevant
            context = "\n\n".join([
                f"[Source: {s['page_title']}]\n{s['chunk_text']}"
                for s in sources
            ])
            enhanced_prompt = f"""{GENERAL_SYSTEM_PROMPT}

RETRIEVED CONTEXT:
{context}

USER QUESTION: {query}

Answer the question using ONLY the context above. Include [Source: page_title] citations."""
        else:
            # No relevant chunks or low relevance - handle conversational queries
            enhanced_prompt = f"""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

The user asked: "{query}"

This question is not related to the Physical AI & Humanoid Robotics documentation, or no relevant information was found.

RESPONSE GUIDELINES:
- If this is a greeting (hi, hello, how are you): Respond warmly and mention you're here to help with questions about Physical AI & Humanoid Robotics.
- If this is a general question unrelated to the textbook: Politely explain you specialize in Physical AI & Humanoid Robotics and suggest asking related questions.
- If this seems like a textbook question but nothing was found: Say "I couldn't find information about that in the Physical AI & Humanoid Robotics documentation. Could you rephrase your question or ask about a different topic?"

Keep responses brief and friendly."""

        # Create agent WITHOUT tools (context is already provided)
        agent = Agent(
            name="RAG Assistant",
            instructions=enhanced_prompt,
            tools=[],  # No tools needed - context is pre-retrieved
            model=get_model(),
        )

    # Run the agent with retry logic for rate limits
    async def _run():
        return await Runner.run(agent, input=query)

    result = await run_with_retry(_run)

    return result.final_output, sources


async def run_agent_streamed(
    query: str,
    mode: str,
    selected_text: str | None = None,
    top_k: int = 5,
    score_threshold: float | None = None,
    filters: dict[str, str | list[str]] | None = None,
):
    """Run the agent with streaming output.

    Yields events for SSE streaming:
    - {"type": "chunk", "content": "..."}  - Text chunks
    - {"type": "sources", "sources": [...]} - Source citations
    - {"type": "done", "metadata": {...}}   - Completion

    Args:
        query: User's question
        mode: "general" or "selected_text"
        selected_text: Text to constrain answers (required for selected_text mode)
        top_k: Number of chunks to retrieve (general mode only)
        score_threshold: Minimum similarity score (general mode only)
        filters: Metadata filters (general mode only)

    Yields:
        Event dictionaries for SSE streaming
    """
    from openai.types.responses import ResponseTextDeltaEvent

    if mode == "selected_text":
        if not selected_text:
            raise ValueError("selected_text is required for selected_text mode")

        agent = Agent(
            name="Selection Assistant",
            instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(selected_text=selected_text),
            tools=[],
            model=get_model(),
        )

        sources = [
            {
                "source_url": "selected_text",
                "page_title": "User Selection",
                "section_heading": "Selected Text",
                "chunk_text": selected_text[:500] if selected_text else "",
                "relevance_score": 1.0,
            }
        ]
    else:
        # General mode: manually retrieve chunks BEFORE agent execution
        # This ensures chunks are always retrieved (Gemini doesn't reliably call tools)
        request = RetrievalRequest(
            query=query,
            top_k=top_k,
            score_threshold=score_threshold,
            filters=filters,
        )
        retrieval_result = retrieve(request)

        # Format sources for response
        sources = [
            {
                "source_url": chunk.source_url,
                "page_title": chunk.page_title,
                "section_heading": chunk.section_heading or "Introduction",
                "chunk_text": chunk.chunk_text,
                "relevance_score": round(chunk.score, 4),
            }
            for chunk in retrieval_result.results
        ]

        # Build context from retrieved chunks
        if sources and sources[0]['relevance_score'] > 0.3:  # Only use if reasonably relevant
            context = "\n\n".join([
                f"[Source: {s['page_title']}]\n{s['chunk_text']}"
                for s in sources
            ])
            enhanced_prompt = f"""{GENERAL_SYSTEM_PROMPT}

RETRIEVED CONTEXT:
{context}

USER QUESTION: {query}

Answer the question using ONLY the context above. Include [Source: page_title] citations."""
        else:
            # No relevant chunks or low relevance - handle conversational queries
            enhanced_prompt = f"""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

The user asked: "{query}"

This question is not related to the Physical AI & Humanoid Robotics documentation, or no relevant information was found.

RESPONSE GUIDELINES:
- If this is a greeting (hi, hello, how are you): Respond warmly and mention you're here to help with questions about Physical AI & Humanoid Robotics.
- If this is a general question unrelated to the textbook: Politely explain you specialize in Physical AI & Humanoid Robotics and suggest asking related questions.
- If this seems like a textbook question but nothing was found: Say "I couldn't find information about that in the Physical AI & Humanoid Robotics documentation. Could you rephrase your question or ask about a different topic?"

Keep responses brief and friendly."""

        # Create agent WITHOUT tools (context is already provided)
        agent = Agent(
            name="RAG Assistant",
            instructions=enhanced_prompt,
            tools=[],  # No tools needed - context is pre-retrieved
            model=get_model(),
        )

    # Run with streaming and retry logic for rate limits
    last_exception = None
    for attempt in range(MAX_RETRIES):
        try:
            result = Runner.run_streamed(agent, input=query)

            # Stream events
            async for event in result.stream_events():
                if event.type == "raw_response_event":
                    if isinstance(event.data, ResponseTextDeltaEvent):
                        yield {"type": "chunk", "content": event.data.delta}

            # If we got here without error, break the retry loop
            break

        except Exception as e:
            error_str = str(e).lower()
            if "429" in error_str or "rate" in error_str or "quota" in error_str:
                last_exception = e
                if attempt < MAX_RETRIES - 1:
                    delay = RETRY_DELAYS[attempt]
                    import structlog
                    logger = structlog.get_logger()
                    logger.warning(
                        "rate_limit_retry_stream",
                        attempt=attempt + 1,
                        max_retries=MAX_RETRIES,
                        delay_seconds=delay,
                    )
                    await asyncio.sleep(delay)
                    continue
            raise e
    else:
        if last_exception:
            raise last_exception

    # Yield sources (already retrieved above)
    yield {"type": "sources", "sources": sources}


# =============================================================================
# Citation Validation
# =============================================================================


def validate_citations(
    response_sources: list[dict[str, Any]],
    retrieved_chunks: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Validate that cited sources match retrieved chunks.

    Removes any citations that don't correspond to actual retrieved content.
    This prevents hallucinated citations.

    Args:
        response_sources: Sources cited in the response
        retrieved_chunks: Chunks actually retrieved from Qdrant

    Returns:
        Validated list of sources (only those that were actually retrieved)
    """
    import structlog

    logger = structlog.get_logger()

    # Build set of valid source URLs
    valid_urls = {chunk["source_url"] for chunk in retrieved_chunks}
    valid_urls.add("selected_text")  # Always allow selected_text source

    validated = []
    for source in response_sources:
        source_url = source.get("source_url", "")
        if source_url in valid_urls:
            validated.append(source)
        else:
            logger.warning(
                "citation_stripped",
                url=source_url,
                reason="Source URL not in retrieved chunks",
            )

    return validated
