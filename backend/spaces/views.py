"""Viewsets and API endpoints for rooms and cameras."""

from __future__ import annotations

from typing import Any, Dict

import json

from django.conf import settings
from django.db import transaction
from django.utils import timezone
from django_filters.rest_framework import DjangoFilterBackend
from rest_framework import status, viewsets
from rest_framework.authentication import BasicAuthentication, SessionAuthentication
from rest_framework.decorators import action
from rest_framework.filters import OrderingFilter, SearchFilter
from rest_framework.permissions import IsAuthenticated
from rest_framework.response import Response
from rest_framework.views import APIView

from .floorplan import FloorplanGenerator
from .models import Camera, CameraCalibrationRun, FaceEmbedding, FaceSnapshot, Identity, Room
from .ros_bridge import ROSBridgeClient
from .serializers import (CalibrationCancelSerializer,
                          CalibrationStartSerializer,
                          CameraCalibrationRunSerializer, CameraSerializer,
                          CameraTestConnectionSerializer, FaceEmbeddingSerializer,
                          FaceSnapshotSerializer, FloorplanUploadSerializer,
                          IdentitySerializer, RoomRecenterSerializer, RoomSerializer,
                          SaveIntrinsicsSerializer)


class FloorplanRenderView(APIView):
    authentication_classes = [SessionAuthentication, BasicAuthentication]
    permission_classes = [IsAuthenticated]

    def post(self, request) -> Response:
        serializer = FloorplanUploadSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        plan_payload = json.loads(json.dumps(serializer.data))
        generator = FloorplanGenerator(
            plan_payload,
            obj_output_path=settings.FLOORPLAN_OBJ_OUTPUT_PATH,
            plan_storage_path=settings.FLOORPLAN_PLAN_STORAGE_PATH,
            wall_height=settings.FLOORPLAN_WALL_HEIGHT,
            wall_thickness=settings.FLOORPLAN_WALL_THICKNESS,
            storey_height=settings.FLOORPLAN_STOREY_HEIGHT,
            floor_thickness=settings.FLOORPLAN_FLOOR_THICKNESS,
            fallback_width=settings.FLOORPLAN_FALLBACK_WIDTH,
            fallback_depth=settings.FLOORPLAN_FALLBACK_DEPTH,
        )
        try:
            result = generator.generate()
        except ValueError as exc:  # invalid floorplan data
            return Response({"detail": str(exc)}, status=status.HTTP_400_BAD_REQUEST)
        return Response(result, status=status.HTTP_202_ACCEPTED)


class RoomViewSet(viewsets.ModelViewSet):
    queryset = Room.objects.all()
    serializer_class = RoomSerializer
    filter_backends = [DjangoFilterBackend, SearchFilter, OrderingFilter]
    filterset_fields = ["name", "level"]
    ordering_fields = ["name", "level"]
    search_fields = ["name"]

    @action(detail=True, methods=["post"])
    def recenter(self, request, pk: str | None = None) -> Response:
        room = self.get_object()
        serializer = RoomRecenterSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        for field, value in serializer.validated_data.items():
            setattr(room, field, value)
        room.save(
            update_fields=["origin_x_mm", "origin_y_mm", "rotation_deg", "updated_at"]
        )
        return Response(
            RoomSerializer(room, context=self.get_serializer_context()).data
        )


class CameraViewSet(viewsets.ModelViewSet):
    queryset = Camera.objects.select_related("room").all()
    serializer_class = CameraSerializer
    filter_backends = [DjangoFilterBackend, SearchFilter, OrderingFilter]
    filterset_fields = ["room", "last_health", "calibration_state"]
    ordering_fields = ["name", "created_at"]
    search_fields = ["name", "make", "model"]

    def get_bridge(self) -> ROSBridgeClient:
        return ROSBridgeClient()

    @action(detail=True, methods=["post"], url_path="test-connection")
    def test_connection(self, request, pk: str | None = None) -> Response:
        camera = self.get_object()
        payload = self.get_bridge().probe_camera(str(camera.id))
        serializer = CameraTestConnectionSerializer(data=payload)
        if serializer.is_valid():
            return Response(serializer.validated_data)
        return Response(payload)

    @action(detail=True, methods=["post"], url_path="calibration/start")
    def calibration_start(self, request, pk: str | None = None) -> Response:
        camera = self.get_object()
        serializer = CalibrationStartSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        run = CameraCalibrationRun.objects.create(
            camera=camera,
            method=serializer.validated_data["method"],
            board_spec=serializer.validated_data["board_spec"],
            status=CameraCalibrationRun.Status.RUNNING,
            started_at=timezone.now(),
        )
        camera.calibration_state = Camera.CalibrationState.IN_PROGRESS
        camera.save(update_fields=["calibration_state", "updated_at"])
        payload = {**serializer.validated_data, "run_id": str(run.id)}
        bridge_response = self.get_bridge().start_calibration(str(camera.id), payload)
        response_payload: Dict[str, Any] = {
            "run": CameraCalibrationRunSerializer(
                run, context=self.get_serializer_context()
            ).data,
            "bridge": bridge_response,
        }
        return Response(response_payload, status=status.HTTP_202_ACCEPTED)

    @action(detail=True, methods=["post"], url_path="calibration/cancel")
    def calibration_cancel(self, request, pk: str | None = None) -> Response:
        camera = self.get_object()
        serializer = CalibrationCancelSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        run = None
        if serializer.validated_data.get("run_id"):
            run = camera.calibration_runs.filter(
                id=serializer.validated_data["run_id"]
            ).first()
        if run is None:
            run = camera.calibration_runs.filter(
                status__in=[
                    CameraCalibrationRun.Status.PENDING,
                    CameraCalibrationRun.Status.RUNNING,
                ]
            ).first()
        if run:
            run.status = CameraCalibrationRun.Status.CANCELLED
            run.finished_at = timezone.now()
            run.save(update_fields=["status", "finished_at", "updated_at"])
        bridge_response = self.get_bridge().cancel_calibration(
            str(camera.id), {"run_id": str(run.id) if run else None}
        )
        camera.calibration_state = Camera.CalibrationState.FAILED
        camera.save(update_fields=["calibration_state", "updated_at"])
        serialized_run = (
            CameraCalibrationRunSerializer(
                run, context=self.get_serializer_context()
            ).data
            if run
            else None
        )
        return Response({"run": serialized_run, "bridge": bridge_response})

    @action(detail=True, methods=["get"], url_path="calibration/runs")
    def calibration_runs(self, request, pk: str | None = None) -> Response:
        camera = self.get_object()
        runs = camera.calibration_runs.all()
        serializer = CameraCalibrationRunSerializer(
            runs, many=True, context=self.get_serializer_context()
        )
        return Response(serializer.data)

    @action(detail=True, methods=["post"], url_path="calibration/save")
    def calibration_save(self, request, pk: str | None = None) -> Response:
        camera = self.get_object()
        serializer = SaveIntrinsicsSerializer(
            data=request.data, context={"camera": camera}
        )
        serializer.is_valid(raise_exception=True)
        camera = serializer.save()
        return Response(
            CameraSerializer(camera, context=self.get_serializer_context()).data
        )


class IdentityViewSet(viewsets.ModelViewSet):
    queryset = Identity.objects.all()
    serializer_class = IdentitySerializer
    filter_backends = [DjangoFilterBackend, SearchFilter, OrderingFilter]
    filterset_fields = ["is_active", "external_id"]
    ordering_fields = ["display_name", "created_at"]
    search_fields = ["display_name", "external_id"]


class FaceEmbeddingViewSet(viewsets.ModelViewSet):
    queryset = FaceEmbedding.objects.select_related("identity", "camera").all()
    serializer_class = FaceEmbeddingSerializer
    filter_backends = [DjangoFilterBackend, SearchFilter, OrderingFilter]
    filterset_fields = ["identity", "camera"]
    ordering_fields = ["captured_at", "created_at"]
    search_fields = ["embedding_id", "identity__display_name"]

    def get_bridge(self) -> ROSBridgeClient:
        return ROSBridgeClient()

    def perform_create(self, serializer: FaceEmbeddingSerializer) -> None:
        with transaction.atomic():
            embedding: FaceEmbedding = serializer.save()
            payload = {
                "identity": str(embedding.identity_id),
                "embedding_id": embedding.embedding_id,
                "vector": embedding.vector,
                "quality": float(embedding.quality),
                "camera_id": str(embedding.camera_id) if embedding.camera_id else None,
                "track_id": embedding.track_id,
                "captured_at": embedding.captured_at.isoformat(),
                "metadata": embedding.metadata,
            }
            self.get_bridge().upload_face_embedding(payload)


class FaceSnapshotViewSet(viewsets.ModelViewSet):
    queryset = FaceSnapshot.objects.select_related("identity", "camera", "embedding").all()
    serializer_class = FaceSnapshotSerializer
    filter_backends = [DjangoFilterBackend, SearchFilter, OrderingFilter]
    filterset_fields = ["identity", "camera"]
    ordering_fields = ["captured_at", "created_at"]
    search_fields = ["identity__display_name", "embedding__embedding_id"]
