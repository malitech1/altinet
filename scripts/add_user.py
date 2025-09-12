#!/usr/bin/env python3
"""Interactively add a user and capture training photos."""

import argparse
import json
from pathlib import Path
from typing import Any, Dict

from altinet.services.face_recognition import FaceRecognitionService

try:  # pragma: no cover - optional dependencies
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - dependency might be missing
    cv2 = None  # type: ignore

try:  # pragma: no cover - optional dependency
    import face_recognition  # type: ignore
except ImportError:  # pragma: no cover - dependency might be missing
    face_recognition = None  # type: ignore


def collect_user_data() -> Dict[str, Any]:
    """Prompt for basic user information and return it as a dict."""
    name = input("Name: ").strip()
    birthday = input("Birthday (YYYY-MM-DD): ").strip()
    phone = input("Phone number: ").strip()
    email = input("Email: ").strip()
    level = int(
        input("User level (1=Owner, 2=Friend/Family, 3=Guest): ").strip()
    )
    return {
        "name": name,
        "birthday": birthday,
        "phone": phone,
        "email": email,
        "level": level,
    }


def capture_photos(out_dir: Path, count: int) -> None:
    """Capture ``count`` photos from the default camera into ``out_dir``.

    If OpenCV is not available or the camera cannot be opened, a message is
    printed and the function returns without raising an exception.
    """
    if cv2 is None:  # pragma: no cover - OpenCV not installed
        print("OpenCV is not installed; skipping photo capture.")
        return

    cap = cv2.VideoCapture(0)  # pragma: no cover - requires hardware
    if not cap.isOpened():  # pragma: no cover - requires hardware
        print("Could not access camera; skipping photo capture.")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    for i in range(count):  # pragma: no cover - requires hardware
        ret, frame = cap.read()
        if not ret:
            break
        filename = out_dir / f"photo_{i+1}.jpg"
        cv2.imwrite(str(filename), frame)
        print(f"Captured {filename}")
    cap.release()


def train_user_photos(user_dir: Path, name: str) -> None:
    """Train the face recognition system on images in ``user_dir``.

    Each photo in ``user_dir / 'photos'`` is encoded and added to the
    :class:`~altinet.services.face_recognition.FaceRecognitionService` cache as
    ``name``.  If the ``face_recognition`` dependency is unavailable, a message
    is printed and the function returns silently.
    """
    if face_recognition is None:  # pragma: no cover - dependency missing
        print("face_recognition is not installed; skipping training.")
        return
    service = FaceRecognitionService()
    photos_dir = user_dir / "photos"
    if not photos_dir.exists():
        return
    for photo in photos_dir.glob("*.jpg"):
        image = face_recognition.load_image_file(photo)
        service.train(image, name)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Add a user and capture training photos."
    )
    parser.add_argument(
        "--photo-count",
        type=int,
        default=5,
        help="Number of photos to capture",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("assets/users"),
        help="Directory to store user data and photos",
    )
    args = parser.parse_args()

    data = collect_user_data()
    user_dir = args.output / data["name"].replace(" ", "_")
    photos_dir = user_dir / "photos"

    capture_photos(photos_dir, args.photo_count)

    user_dir.mkdir(parents=True, exist_ok=True)
    with (user_dir / "metadata.json").open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"User data saved to {user_dir}")

    train_user_photos(user_dir, data["name"])


if __name__ == "__main__":
    main()
