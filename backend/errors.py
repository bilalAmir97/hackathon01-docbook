"""Error taxonomy and exceptions for retrieval pipeline.

This module defines the exception hierarchy for all retrieval operations.
Each error has a unique code (E001-E010) and indicates whether it is retryable.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, TypeVar

from config import RETRY_BACKOFF_MS


# ============================================================================
# Base Exception
# ============================================================================


@dataclass
class RetrievalError(Exception):
    """Base exception for all retrieval errors.

    Attributes:
        code: Error code (E001-E010)
        message: Human-readable error message
        details: Additional context (optional)
        retry_after: Seconds to wait before retry (if retryable)
    """

    code: str
    message: str
    details: dict[str, Any] | None = None
    retry_after: int | None = None

    def __str__(self) -> str:
        return f"[{self.code}] {self.message}"

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(code={self.code!r}, message={self.message!r})"

    @property
    def is_retryable(self) -> bool:
        """Return True if this error type supports retry."""
        return self.retry_after is not None


# ============================================================================
# Specific Error Classes (E001-E010)
# ============================================================================


@dataclass
class QdrantConnectionError(RetrievalError):
    """E001: Cannot connect to Qdrant cluster.

    Retryable: Yes (3x with exponential backoff)
    """

    code: str = field(default="E001", init=False)
    retry_after: int | None = 1

    def __init__(self, message: str = "Cannot connect to Qdrant cluster", details: dict[str, Any] | None = None):
        super().__init__(code="E001", message=message, details=details, retry_after=1)


@dataclass
class CollectionNotFoundError(RetrievalError):
    """E002: Target collection does not exist.

    Retryable: No
    """

    code: str = field(default="E002", init=False)

    def __init__(self, collection_name: str, details: dict[str, Any] | None = None):
        message = f"Collection not found: {collection_name}"
        super().__init__(code="E002", message=message, details=details, retry_after=None)


@dataclass
class InvalidQueryError(RetrievalError):
    """E003: Query is empty, too long, or malformed.

    Retryable: No
    """

    code: str = field(default="E003", init=False)

    def __init__(self, message: str = "Invalid query", details: dict[str, Any] | None = None):
        super().__init__(code="E003", message=message, details=details, retry_after=None)


@dataclass
class EmbeddingError(RetrievalError):
    """E004: Cohere API failure or timeout.

    Retryable: Yes (2x with backoff)
    """

    code: str = field(default="E004", init=False)
    retry_after: int | None = 1

    def __init__(self, message: str = "Embedding API error", details: dict[str, Any] | None = None):
        super().__init__(code="E004", message=message, details=details, retry_after=1)


@dataclass
class InvalidFilterError(RetrievalError):
    """E005: Unknown filter field or invalid syntax.

    Retryable: No
    """

    code: str = field(default="E005", init=False)

    def __init__(self, field_name: str, valid_fields: set[str], details: dict[str, Any] | None = None):
        message = f"Invalid filter field: {field_name}. Valid fields: {sorted(valid_fields)}"
        super().__init__(code="E005", message=message, details=details, retry_after=None)


@dataclass
class QdrantTimeoutError(RetrievalError):
    """E006: Search exceeded timeout threshold.

    Retryable: Yes (1x)
    """

    code: str = field(default="E006", init=False)
    retry_after: int | None = 1

    def __init__(self, timeout_ms: int, details: dict[str, Any] | None = None):
        message = f"Qdrant search timed out after {timeout_ms}ms"
        super().__init__(code="E006", message=message, details=details, retry_after=1)


@dataclass
class SchemaValidationError(RetrievalError):
    """E007: Payload missing required fields.

    Retryable: No
    """

    code: str = field(default="E007", init=False)

    def __init__(self, missing_fields: set[str], details: dict[str, Any] | None = None):
        message = f"Payload missing required fields: {sorted(missing_fields)}"
        super().__init__(code="E007", message=message, details=details, retry_after=None)


@dataclass
class RateLimitError(RetrievalError):
    """E008: Cohere or Qdrant rate limit hit.

    Retryable: Yes (with backoff)
    """

    code: str = field(default="E008", init=False)

    def __init__(self, service: str, retry_after: int = 5, details: dict[str, Any] | None = None):
        message = f"Rate limit exceeded for {service}. Retry after {retry_after}s"
        super().__init__(code="E008", message=message, details=details, retry_after=retry_after)


@dataclass
class ConfigurationError(RetrievalError):
    """E009: Missing environment variables or invalid configuration.

    Retryable: No
    """

    code: str = field(default="E009", init=False)

    def __init__(self, message: str = "Configuration error", details: dict[str, Any] | None = None):
        super().__init__(code="E009", message=message, details=details, retry_after=None)


@dataclass
class InternalError(RetrievalError):
    """E010: Unexpected system error.

    Retryable: No
    """

    code: str = field(default="E010", init=False)

    def __init__(self, message: str = "Internal error", details: dict[str, Any] | None = None):
        super().__init__(code="E010", message=message, details=details, retry_after=None)


# ============================================================================
# Retry Helper
# ============================================================================


T = TypeVar("T")


def retry_with_backoff(
    func: Callable[[], T],
    max_retries: int,
    backoff_ms: list[int] | None = None,
    retryable_errors: tuple[type[Exception], ...] = (QdrantConnectionError, EmbeddingError, QdrantTimeoutError, RateLimitError),
) -> T:
    """Execute function with exponential backoff retry on retryable errors.

    Args:
        func: Function to execute (should take no arguments)
        max_retries: Maximum number of retry attempts
        backoff_ms: List of backoff delays in milliseconds
        retryable_errors: Tuple of exception types that trigger retry

    Returns:
        Result of successful function execution

    Raises:
        The last exception if all retries fail
    """
    if backoff_ms is None:
        backoff_ms = RETRY_BACKOFF_MS

    last_exception: Exception | None = None

    for attempt in range(max_retries + 1):
        try:
            return func()
        except retryable_errors as e:
            last_exception = e

            # Check if we have retries left
            if attempt >= max_retries:
                raise

            # Calculate backoff delay
            delay_ms = backoff_ms[min(attempt, len(backoff_ms) - 1)]

            # If error specifies retry_after, use that instead
            if isinstance(e, RetrievalError) and e.retry_after:
                delay_ms = max(delay_ms, e.retry_after * 1000)

            # Sleep before retry
            time.sleep(delay_ms / 1000.0)

    # Should never reach here, but satisfy type checker
    if last_exception:
        raise last_exception
    raise RuntimeError("Unexpected state in retry_with_backoff")


# ============================================================================
# Error Code Mapping
# ============================================================================


ERROR_CODES = {
    "E001": QdrantConnectionError,
    "E002": CollectionNotFoundError,
    "E003": InvalidQueryError,
    "E004": EmbeddingError,
    "E005": InvalidFilterError,
    "E006": QdrantTimeoutError,
    "E007": SchemaValidationError,
    "E008": RateLimitError,
    "E009": ConfigurationError,
    "E010": InternalError,
}
