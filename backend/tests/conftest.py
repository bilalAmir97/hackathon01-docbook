"""Pytest fixtures for retrieval pipeline and API tests."""

from __future__ import annotations

import os
import sys
from typing import Any, AsyncGenerator
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

# Add backend to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


# ============================================================================
# FastAPI Test Client Fixtures (Spec-3)
# ============================================================================


@pytest.fixture
def test_client():
    """Create a FastAPI test client for synchronous tests."""
    from fastapi.testclient import TestClient
    from api import app

    return TestClient(app)


@pytest.fixture
async def async_test_client():
    """Create an async FastAPI test client for async tests."""
    from httpx import AsyncClient, ASGITransport
    from api import app

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        yield client


@pytest.fixture
def mock_agent_run():
    """Mock the agent.run_agent function for API tests."""
    with patch("api.run_agent") as mock:
        # Default return value
        mock.return_value = (
            "ROS 2 is a robotics middleware framework.",
            [
                {
                    "source_url": "https://example.com/ros2",
                    "page_title": "Introduction to ROS 2",
                    "section_heading": "Overview",
                    "chunk_text": "ROS 2 is the next generation...",
                    "relevance_score": 0.95,
                }
            ],
        )
        yield mock


@pytest.fixture
def mock_agent_run_streaming():
    """Mock the agent.run_agent_streamed function for streaming API tests."""
    async def mock_stream(*args, **kwargs):
        yield {"type": "chunk", "content": "ROS 2 "}
        yield {"type": "chunk", "content": "is a robotics middleware."}
        yield {"type": "sources", "sources": []}

    with patch("api.run_agent_streamed", side_effect=mock_stream):
        yield


# ============================================================================
# Mock Cohere Client Fixtures
# ============================================================================


class MockCohereEmbedResponse:
    """Mock Cohere embed response."""

    def __init__(self, embeddings: list[list[float]]):
        self.embeddings = embeddings


@pytest.fixture
def mock_cohere_client():
    """Create a mock Cohere client for embedding tests."""
    client = MagicMock()

    def mock_embed(texts: list[str], model: str, input_type: str) -> MockCohereEmbedResponse:
        embeddings = [[0.1] * 1024 for _ in texts]
        return MockCohereEmbedResponse(embeddings)

    client.embed = mock_embed
    return client


@pytest.fixture
def mock_cohere_embed():
    """Patch cohere.Client for embedding operations."""
    with patch("cohere.Client") as mock_class:
        mock_client = MagicMock()

        def mock_embed_fn(texts: list[str], model: str, input_type: str):
            embeddings = [[0.1] * 1024 for _ in texts]
            return MockCohereEmbedResponse(embeddings)

        mock_client.embed = mock_embed_fn
        mock_class.return_value = mock_client
        yield mock_client


# ============================================================================
# Mock Qdrant Client Fixtures
# ============================================================================


class MockQdrantPoint:
    """Mock Qdrant ScoredPoint."""

    def __init__(
        self,
        id: str,
        score: float,
        payload: dict[str, Any],
    ):
        self.id = id
        self.score = score
        self.payload = payload


class MockQdrantSearchResult:
    """Mock Qdrant search result."""

    def __init__(self, points: list[MockQdrantPoint]):
        self.points = points

    def __iter__(self):
        return iter(self.points)

    def __len__(self):
        return len(self.points)


@pytest.fixture
def sample_qdrant_points() -> list[MockQdrantPoint]:
    """Return sample Qdrant points for testing."""
    return [
        MockQdrantPoint(
            id="abc123def456abc123def456abc12345",
            score=0.95,
            payload={
                "source_url": "https://example.com/intro",
                "page_title": "Introduction to ROS 2",
                "section_heading": "Getting Started",
                "chunk_index": 0,
                "chunk_text": "ROS 2 is the next generation Robot Operating System.",
                "content_hash": "hash1",
                "indexed_at": "2025-01-01T00:00:00Z",
            },
        ),
        MockQdrantPoint(
            id="def456abc123def456abc123def45678",
            score=0.88,
            payload={
                "source_url": "https://example.com/basics",
                "page_title": "ROS 2 Basics",
                "section_heading": "Nodes and Topics",
                "chunk_index": 1,
                "chunk_text": "Nodes are the fundamental units of computation in ROS 2.",
                "content_hash": "hash2",
                "indexed_at": "2025-01-01T00:00:00Z",
            },
        ),
        MockQdrantPoint(
            id="ghi789abc123def456abc123def45678",
            score=0.75,
            payload={
                "source_url": "https://example.com/advanced",
                "page_title": "Advanced Topics",
                "section_heading": None,
                "chunk_index": 2,
                "chunk_text": "Advanced ROS 2 features include lifecycle nodes.",
                "content_hash": "hash3",
                "indexed_at": "2025-01-01T00:00:00Z",
            },
        ),
    ]


class MockQueryResponse:
    """Mock Qdrant QueryResponse."""

    def __init__(self, points: list[MockQdrantPoint]):
        self.points = points


@pytest.fixture
def mock_qdrant_client(sample_qdrant_points):
    """Create a mock Qdrant client for search tests."""
    client = MagicMock()

    def mock_query_points(
        collection_name: str,
        query: list[float],
        limit: int = 5,
        query_filter: Any = None,
        with_payload: bool = True,
        score_threshold: float | None = None,
        **kwargs,
    ) -> MockQueryResponse:
        return MockQueryResponse(sample_qdrant_points[:limit])

    client.query_points = mock_query_points

    class MockCollectionInfo:
        points_count = 150
        indexed_vectors_count = 150

        class config:
            class params:
                vectors = MagicMock()
                vectors.size = 1024
                vectors.distance = "Cosine"

    client.get_collection = MagicMock(return_value=MockCollectionInfo())
    client.scroll = MagicMock(return_value=(sample_qdrant_points, None))

    return client


@pytest.fixture
def mock_qdrant_search(sample_qdrant_points):
    """Patch qdrant_client.QdrantClient for search operations."""
    with patch("qdrant_client.QdrantClient") as mock_class:
        mock_client = MagicMock()

        def mock_query_points_fn(
            collection_name: str,
            query: list[float],
            limit: int = 5,
            query_filter: Any = None,
            with_payload: bool = True,
            score_threshold: float | None = None,
            **kwargs,
        ):
            return MockQueryResponse(sample_qdrant_points[:limit])

        mock_client.query_points = mock_query_points_fn
        mock_class.return_value = mock_client
        yield mock_client


# ============================================================================
# Environment Fixtures
# ============================================================================


@pytest.fixture
def mock_env_vars():
    """Set up mock environment variables for testing."""
    env_vars = {
        "COHERE_API_KEY": "test-cohere-key",
        "QDRANT_URL": "https://test-qdrant-url.com",
        "QDRANT_API_KEY": "test-qdrant-key",
    }
    with patch.dict(os.environ, env_vars):
        yield env_vars


@pytest.fixture
def clean_env():
    """Remove retrieval-related env vars for negative testing."""
    vars_to_remove = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    original = {k: os.environ.get(k) for k in vars_to_remove}

    for var in vars_to_remove:
        os.environ.pop(var, None)

    yield

    for var, value in original.items():
        if value is not None:
            os.environ[var] = value


# ============================================================================
# Sample Data Fixtures
# ============================================================================


@pytest.fixture
def sample_retrieval_request():
    """Return a sample RetrievalRequest for testing."""
    from models import RetrievalRequest

    return RetrievalRequest(
        query="What is ROS 2?",
        top_k=5,
        score_threshold=None,
        filters=None,
    )


@pytest.fixture
def sample_retrieval_request_with_filters():
    """Return a RetrievalRequest with metadata filters."""
    from models import RetrievalRequest

    return RetrievalRequest(
        query="How to launch Gazebo?",
        top_k=3,
        score_threshold=0.5,
        filters={"source_url": "https://example.com/intro"},
    )


@pytest.fixture
def golden_queries():
    """Return sample golden queries for evaluation testing."""
    return [
        {
            "query": "What is ROS 2?",
            "expected_urls": ["https://example.com/intro"],
            "query_type": "factual",
        },
        {
            "query": "How does navigation work?",
            "expected_urls": ["https://example.com/basics"],
            "query_type": "conceptual",
        },
        {
            "query": "How to launch Gazebo simulation?",
            "expected_urls": ["https://example.com/advanced"],
            "query_type": "procedural",
        },
    ]
