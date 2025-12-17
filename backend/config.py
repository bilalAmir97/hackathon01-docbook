"""Configuration constants for retrieval pipeline.

This module contains all configuration values used across the retrieval,
validation, and evaluation modules. Values are derived from:
- Spec-1 implementation (Qdrant collection, Cohere model)
- data-model.md specifications
- research.md design decisions
"""

from __future__ import annotations

import os
from pathlib import Path

from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv(Path(__file__).parent / ".env")


# ============================================================================
# Environment Variables
# ============================================================================

def get_env_var(name: str, required: bool = True) -> str | None:
    """Get environment variable with optional requirement check."""
    value = os.environ.get(name)
    if required and not value:
        raise ValueError(f"Required environment variable {name} is not set")
    return value


# ============================================================================
# Cohere Settings
# ============================================================================

COHERE_MODEL = "embed-english-v3.0"
COHERE_DIMENSIONS = 1024
COHERE_INPUT_TYPE_QUERY = "search_query"      # For retrieval queries
COHERE_INPUT_TYPE_DOCUMENT = "search_document"  # For ingestion (Spec-1)


# ============================================================================
# Qdrant Settings
# ============================================================================

QDRANT_COLLECTION = "rag_embedding"


# ============================================================================
# Timeouts (milliseconds)
# ============================================================================

QUERY_TIMEOUT_MS = 5000    # Total query timeout
QDRANT_TIMEOUT_MS = 3000   # Qdrant search timeout
COHERE_TIMEOUT_MS = 2000   # Cohere API timeout


# ============================================================================
# Retry Settings
# ============================================================================

MAX_RETRIES_CONNECTION = 3
MAX_RETRIES_EMBEDDING = 2
RETRY_BACKOFF_MS = [100, 200, 400]  # Exponential backoff


# ============================================================================
# Input Constraints
# ============================================================================

QUERY_MIN_LENGTH = 1
QUERY_MAX_LENGTH = 2000
K_MIN = 1
K_MAX = 100
K_DEFAULT = 10  # Balanced default for good coverage


# ============================================================================
# Validation Settings
# ============================================================================

SCHEMA_SAMPLE_SIZE = 10  # Number of vectors to sample for schema validation


# ============================================================================
# Evaluation Settings
# ============================================================================

MRR_PASS_THRESHOLD = 0.5  # Minimum MRR for evaluation to pass


# ============================================================================
# Payload Fields (from Spec-1)
# ============================================================================

REQUIRED_PAYLOAD_FIELDS = {
    "source_url",
    "page_title",
    "section_heading",
    "chunk_index",
    "chunk_text",
}

FILTERABLE_FIELDS = {
    "source_url",
    "page_title",
    "section_heading",
}


# ============================================================================
# Paths
# ============================================================================

BACKEND_DIR = Path(__file__).parent
TESTS_DIR = BACKEND_DIR / "tests"
GOLDEN_QUERIES_PATH = TESTS_DIR / "golden_queries.json"
