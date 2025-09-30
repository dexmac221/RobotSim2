"""Integration test modeled after Gemini Robotics documentation prompts."""

from __future__ import annotations

import io
import json
import os

import numpy as np
import pytest

try:  # pragma: no cover
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None

try:  # pragma: no cover
    from google import genai
    from google.genai import types as genai_types
except ImportError:  # pragma: no cover
    genai = None
    genai_types = None  # type: ignore


def _get_api_key() -> str | None:
    for env_name in ("GEMINI_API_KEY", "GENAI_API_KEY", "GOOGLE_API_KEY"):
        value = os.getenv(env_name)
        if value:
            return value
    return None
def test_gemini_robotics_point_prompt_returns_json() -> None:
    """Send a documented Gemini Robotics prompt and verify JSON-like output."""
    if genai is None or genai_types is None:
        pytest.skip("google-genai library is not installed")
    if Image is None:
        pytest.skip("Pillow is required for Gemini robotics integration test")

    api_key = _get_api_key()
    if not api_key:
        pytest.skip("Gemini API key not provided in environment")

    # Create a simple synthetic overhead image with a red circle target.
    height, width = 256, 256
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    yy, xx = np.ogrid[:height, :width]
    mask = (yy - height // 2) ** 2 + (xx - width // 2) ** 2 <= 35 ** 2
    frame[mask] = (220, 40, 40)
    image = Image.fromarray(frame)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    image_bytes = buffer.getvalue()

    prompt = (
        "Point to the red target on the table. "
        "Respond as a JSON list with entries of the form {\"point\": [y, x], \"label\": \"target\"}. "
        "Coordinates must be integers between 0 and 1000."
    )

    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model="gemini-robotics-er-1.5-preview",
        contents=[
            genai_types.Part.from_bytes(data=image_bytes, mime_type="image/png"),
            prompt,
        ],
        config=genai_types.GenerateContentConfig(
            temperature=0.2,
            thinking_config=genai_types.ThinkingConfig(thinking_budget=16),
        ),
    )

    text = (response.text or "").strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if len(lines) >= 3:
            closing_idx = next((i for i, line in enumerate(lines[1:], start=1) if line.startswith("```")), None)
            if closing_idx is not None:
                text = "\n".join(lines[1:closing_idx]).strip()
                if not text:
                    pytest.fail("Gemini Robotics response code fence was empty")
    assert text, "Gemini Robotics response was empty"

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:  # pragma: no cover - debugging aid
        pytest.fail(f"Gemini Robotics response was not valid JSON: {exc}: {text}")

    assert isinstance(parsed, list), "Gemini Robotics response was not a list"
    assert parsed, "Gemini Robotics response list was empty"
    first = parsed[0]
    assert "point" in first and "label" in first, "Response missing expected keys"