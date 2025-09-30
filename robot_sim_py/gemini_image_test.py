"""CLI helper to query the Gemini Robotics model with a saved top camera image.

The flow mirrors the Gemini Robotics quick-start documentation: load an RGB frame,
convert it to JPEG bytes, call `google.genai.Client.models.generate_content`, and
parse the structured JSON result. The response is then mapped into the RobotSim
workspace using the same calibration constants as the live OpenGL viewer.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image

from .gemini_agent import GeminiRobotAgent
from .robot import RobotModel
from .scene import TABLE_HEIGHT

_DEFAULT_MODEL = "gemini-robotics-er-1.5-preview"
_API_ENV_VARS = ("GEMINI_API_KEY", "GENAI_API_KEY", "GOOGLE_API_KEY")


def _resolve_api_key(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    for name in _API_ENV_VARS:
        value = os.getenv(name)
        if value:
            return value
    return None


def _load_image(path: Path) -> np.ndarray:
    with Image.open(path) as img:
        rgb = img.convert("RGB")
        return np.array(rgb)


def _format_detection(agent: GeminiRobotAgent, point: Sequence[float], label: str, index: int) -> dict:
    world = agent._normalized_to_world(point)
    return {
        "index": index,
        "label": label,
        "pixel": [float(point[0]), float(point[1])],
        "world": {
            "x": float(world[0]),
            "y": float(world[1]),
            "z": float(world[2]),
        },
        "hover_height": float(agent.hover_height),
        "table_height": float(TABLE_HEIGHT),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Send a saved image to Gemini Robotics and print detections.")
    parser.add_argument("image", type=Path, help="Path to an RGB image (PNG/JPEG) captured from the top camera")
    parser.add_argument(
        "--gemini-api-key",
        dest="api_key",
        default=None,
        help="Gemini API key. Falls back to GEMINI_API_KEY/GENAI_API_KEY/GOOGLE_API_KEY env vars.",
    )
    parser.add_argument("--model", default=_DEFAULT_MODEL, help="Gemini Robotics model to use")
    parser.add_argument("--target-label", default="target", help="Label identifier to match in the response")
    parser.add_argument("--hover-height", type=float, default=0.12, help="Hover height used to map world coordinates")
    parser.add_argument(
        "--thinking-budget",
        type=int,
        default=0,
        help="Optional thinking budget tokens, see Gemini Robotics docs",
    )
    parser.add_argument(
        "--joint-count",
        type=int,
        default=None,
        help="Number of robot joints to simulate (1-6). Defaults to the base arm if omitted.",
    )
    parser.add_argument(
        "--gemini-joint-count",
        type=int,
        default=None,
        help="Override the joint count mentioned in the Gemini prompt",
    )
    parser.add_argument(
        "--dump-json",
        type=Path,
        default=None,
        help="If provided, write the parsed detections to this JSON file",
    )

    args = parser.parse_args(argv)

    image_path = args.image.expanduser()
    if not image_path.exists():
        parser.error(f"Image path not found: {image_path}")

    api_key = _resolve_api_key(args.api_key)
    if not api_key:
        parser.error(
            "No Gemini API key provided. Pass --gemini-api-key or set GEMINI_API_KEY, GENAI_API_KEY, or GOOGLE_API_KEY."
        )

    frame = _load_image(image_path)

    robot_model = RobotModel.default(args.joint_count)
    resolved_joint_count = (
        args.gemini_joint_count
        if args.gemini_joint_count is not None
        else (args.joint_count if args.joint_count is not None else robot_model.dof)
    )

    agent = GeminiRobotAgent(
        robot=robot_model,
        hover_height=args.hover_height,
        model_id=args.model,
        target_label=args.target_label,
        thinking_budget=args.thinking_budget,
        api_key=api_key,
        min_request_interval=0.0,
        joint_count=resolved_joint_count,
    )

    print(f"Sending {image_path} to {args.model}...", file=sys.stderr)
    prompt = agent.prompt_text()
    print("Prompt:", prompt, file=sys.stderr)

    try:
        detections, raw_text = agent.infer_from_frame(frame)
    except Exception as exc:  # pragma: no cover - network errors are environment-dependent
        print(f"Gemini request failed: {exc}", file=sys.stderr)
        return 1

    print("\n=== Gemini raw response ===\n")
    print(raw_text or "<no text returned>")

    parsed = []
    for idx, item in enumerate(detections):
        label = str(item.get("label", ""))
        point = item.get("point")
        if not isinstance(point, Sequence) or len(point) != 2:
            continue
        parsed.append(_format_detection(agent, point, label, idx))

    summary = {
        "image_path": str(image_path),
        "model": args.model,
        "prompt": prompt,
        "robot_joint_count": robot_model.dof,
        "gemini_joint_count": resolved_joint_count,
        "raw_response": raw_text,
        "detections": parsed,
    }

    print("\n=== Parsed detections ===\n")
    if parsed:
        for item in parsed:
            pixel = item["pixel"]
            world = item["world"]
            print(
                f"#{item['index']}: label='{item['label']}' pixel={pixel} -> world=({world['x']:.3f}, {world['y']:.3f}, {world['z']:.3f})"
            )
    else:
        print("No valid target points found in the Gemini response.")

    if args.dump_json:
        args.dump_json.parent.mkdir(parents=True, exist_ok=True)
        args.dump_json.write_text(json.dumps(summary, indent=2))
        print(f"\nSaved detailed results to {args.dump_json}")

    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
