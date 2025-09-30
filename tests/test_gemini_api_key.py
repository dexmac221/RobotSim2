"""Integration check to verify the configured Gemini API key is usable."""

from __future__ import annotations

import os

import pytest

try:  # pragma: no cover - optional import guard for test discovery
    from google import genai
except ImportError:  # pragma: no cover
    genai = None


def _get_api_key() -> str | None:
    for env_name in ("GEMINI_API_KEY", "GENAI_API_KEY", "GOOGLE_API_KEY"):
        value = os.getenv(env_name)
        if value:
            return value
    return None


def test_gemini_api_key_can_list_models() -> None:
    """Fail fast if the configured Gemini API key is rejected by the service."""
    if genai is None:
        pytest.skip("google-genai library is not installed")

    api_key = _get_api_key()
    if not api_key:
        pytest.skip("Gemini API key not provided in environment")

    client = genai.Client(api_key=api_key)

    try:
        iterator = client.models.list()
        first_model = next(iter(iterator))
    except StopIteration:  # pragma: no cover - empty response
        pytest.fail("Gemini API returned an empty model list")
    except Exception as exc:  # pragma: no cover - network/credential failure
        pytest.fail(f"Gemini API key validation failed: {exc}")

    assert getattr(first_model, "name", ""), "First model entry is missing a name"