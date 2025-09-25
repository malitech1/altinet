"""API views for basic session authentication flows."""

from __future__ import annotations

from django.contrib.auth import authenticate, get_user_model, login, logout
from django.db import IntegrityError, transaction
from django.utils.translation import gettext_lazy as _
from rest_framework import status
from rest_framework.permissions import AllowAny, IsAuthenticated
from rest_framework.request import Request
from rest_framework.response import Response
from rest_framework.views import APIView

from .authentication import CsrfExemptSessionAuthentication

User = get_user_model()


def _serialise_user(user: User) -> dict[str, str]:
    return {
        "id": str(user.pk),
        "username": user.get_username(),
        "first_name": user.first_name,
        "last_name": user.last_name,
        "email": user.email,
    }


class AuthStatusView(APIView):
    """Return authentication status and whether any users exist."""

    permission_classes = [AllowAny]
    authentication_classes = [CsrfExemptSessionAuthentication]

    def get(self, request: Request) -> Response:
        has_users = User.objects.exists()
        payload: dict[str, object] = {
            "has_users": has_users,
            "is_authenticated": request.user.is_authenticated,
        }
        if request.user.is_authenticated:
            payload["user"] = _serialise_user(request.user)
        else:
            payload["user"] = None
        return Response(payload)


class RegisterView(APIView):
    """Create the initial user when none exist."""

    permission_classes = [AllowAny]
    authentication_classes: list[type] = []

    def post(self, request: Request) -> Response:
        if User.objects.exists():
            return Response(
                {"detail": _("Registration is disabled once an operator exists.")},
                status=status.HTTP_403_FORBIDDEN,
            )

        username = (request.data.get("username") or "").strip()
        password = request.data.get("password") or ""
        email = (request.data.get("email") or "").strip()
        first_name = (request.data.get("first_name") or "").strip()
        last_name = (request.data.get("last_name") or "").strip()

        if not username or not password:
            return Response(
                {"detail": _("Username and password are required.")},
                status=status.HTTP_400_BAD_REQUEST,
            )

        try:
            with transaction.atomic():
                user = User.objects.create_user(
                    username=username,
                    password=password,
                    email=email,
                    first_name=first_name,
                    last_name=last_name,
                )
        except IntegrityError:
            return Response(
                {"detail": _("Unable to create user with the provided credentials.")},
                status=status.HTTP_400_BAD_REQUEST,
            )

        login(request, user)
        return Response(_serialise_user(user), status=status.HTTP_201_CREATED)


class LoginView(APIView):
    """Authenticate an existing user and begin a session."""

    permission_classes = [AllowAny]
    authentication_classes = [CsrfExemptSessionAuthentication]

    def post(self, request: Request) -> Response:
        username = (request.data.get("username") or "").strip()
        password = request.data.get("password") or ""
        if not username or not password:
            return Response(
                {"detail": _("Username and password are required.")},
                status=status.HTTP_400_BAD_REQUEST,
            )

        user = authenticate(request, username=username, password=password)
        if user is None:
            return Response(
                {"detail": _("Invalid username or password.")},
                status=status.HTTP_400_BAD_REQUEST,
            )

        login(request, user)
        return Response(_serialise_user(user))


class LogoutView(APIView):
    """Terminate the active session."""

    permission_classes = [IsAuthenticated]
    authentication_classes = [CsrfExemptSessionAuthentication]

    def post(self, request: Request) -> Response:
        logout(request)
        return Response(status=status.HTTP_204_NO_CONTENT)


__all__ = [
    "AuthStatusView",
    "RegisterView",
    "LoginView",
    "LogoutView",
]

