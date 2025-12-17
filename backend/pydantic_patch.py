"""Pydantic patch for OpenAI SDK compatibility with LiteLLM/Gemini.

This module MUST be imported BEFORE any OpenAI Agents SDK imports.
LiteLLM returns None for cached_tokens and reasoning_tokens but
the OpenAI SDK's Pydantic models expect integers.

This patch modifies the model fields to accept None with default 0.
"""

from openai.types.responses.response_usage import InputTokensDetails, OutputTokensDetails
from pydantic.fields import FieldInfo
from typing import Optional

# Make cached_tokens optional with default 0
InputTokensDetails.model_fields["cached_tokens"] = FieldInfo(default=0, annotation=Optional[int])
InputTokensDetails.model_rebuild(force=True)

# Make reasoning_tokens optional with default 0
OutputTokensDetails.model_fields["reasoning_tokens"] = FieldInfo(default=0, annotation=Optional[int])
OutputTokensDetails.model_rebuild(force=True)

# Flag to indicate patch was applied
PYDANTIC_PATCH_APPLIED = True
