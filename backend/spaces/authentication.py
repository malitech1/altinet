"""Authentication utilities for API views."""

from __future__ import annotations

from rest_framework.authentication import SessionAuthentication


class CsrfExemptSessionAuthentication(SessionAuthentication):
    """Session authentication that skips CSRF enforcement.

    The frontend is a single-page application that communicates with the
    backend using the Django session cookie. Generating CSRF tokens in that
    context adds friction, so we explicitly relax the CSRF requirement for the
    authentication endpoints that operate over HTTPS.
    """

    def enforce_csrf(self, request):  # type: ignore[override]
        return None


__all__ = ["CsrfExemptSessionAuthentication"]

