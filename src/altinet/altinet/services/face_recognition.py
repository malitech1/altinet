"""Facial recognition service with caching of known identities."""

from __future__ import annotations

import logging
from typing import Any, Callable, List, Optional, Tuple

try:  # pragma: no cover - optional dependency
    import face_recognition as _fr  # type: ignore
except ImportError:  # pragma: no cover - dependency might be missing
    _fr = None  # type: ignore


class FaceRecognitionService:
    """Identify faces only when necessary.

    The service maintains a cache of known face encodings and their
    corresponding identities. When ``recognize`` is called with a new face,
    the service computes its encoding using ``encoder`` and checks against the
    cache. If the face is unknown, an ``identifier`` callback is used to
    determine the identity and confidence level. Subsequent calls with the same
    face reuse the cached result and avoid invoking ``identifier`` again.
    """

    def __init__(
        self,
        *,
        encoder: Optional[Any] = None,
        identifier: Optional[Callable[[Any], Tuple[str, float]]] = None,
    ) -> None:
        self._encoder = encoder if encoder is not None else _fr
        self._identifier = identifier or (lambda encoding: ("Unknown", 0.0))
        self._known_encodings: List[Any] = []
        self._known_identities: List[str] = []
        self._known_confidences: List[float] = []

    def train(self, image: Any, identity: str, confidence: float = 1.0) -> None:
        """Add ``image`` to the cache as ``identity``.

        The image is encoded using the configured ``encoder`` and stored with
        the provided identity and confidence. If the encoder is unavailable or
        no encodings are produced, the call is ignored.
        """
        if self._encoder is None:
            return
        try:
            encodings = self._encoder.face_encodings(image)
        except Exception as exc:  # pragma: no cover - defensive programming
            logging.error("Failed to encode face for training: %s", exc)
            return
        if not encodings:
            return
        encoding = encodings[0]
        self._known_encodings.append(encoding)
        self._known_identities.append(identity)
        self._known_confidences.append(confidence)

    def recognize(self, image: Any) -> Tuple[str, float]:
        """Return the identity and confidence for ``image``.

        Parameters
        ----------
        image:
            Image data understood by the ``encoder``. Tests may provide a stub
            encoder that treats the image object as the encoding directly.
        """

        if self._encoder is None:
            return "Unknown", 0.0

        try:
            encodings = self._encoder.face_encodings(image)
        except Exception as exc:  # pragma: no cover - defensive programming
            logging.error("Failed to encode face for recognition: %s", exc)
            return "Unknown", 0.0
        if not encodings:
            return "Unknown", 0.0
        encoding = encodings[0]

        matches = self._encoder.compare_faces(self._known_encodings, encoding)
        if True in matches:
            index = matches.index(True)
            identity = self._known_identities[index]
            confidence = self._known_confidences[index]
        else:
            identity, confidence = self._identifier(encoding)
            if identity != "Unknown":
                self._known_encodings.append(encoding)
                self._known_identities.append(identity)
                self._known_confidences.append(confidence)

        logging.info(
            "Checked face identity: %s (confidence %.2f)", identity, confidence
        )
        return identity, confidence
