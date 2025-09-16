import React, { useEffect, useMemo, useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";

import {
  Camera,
  CameraCalibrationRun,
  createRoom,
  deleteCamera,
  deleteRoom,
  listCameras,
  listCalibrationRuns,
  listRooms,
  recenterRoom,
  Room,
  updateCamera,
  updateRoom
} from "../../lib/api";
import { createCameraHealthSocket } from "../../lib/ws";
import RoomListPanel from "../../components/spaces/RoomListPanel";
import RoomEditor from "../../components/spaces/RoomEditor";
import CameraList from "../../components/spaces/CameraList";
import CameraForm from "../../components/spaces/CameraForm";
import Floorplan from "../../components/spaces/Floorplan";
import CalibrationWizard from "../../components/spaces/CalibrationWizard";
import CalibrationRunsTable from "../../components/spaces/CalibrationRunsTable";

const Tabs = ["Rooms", "Cameras", "Calibration"] as const;

type Tab = (typeof Tabs)[number];

const SpacesPage: React.FC = () => {
  const queryClient = useQueryClient();
  const { data: rooms = [] } = useQuery({ queryKey: ["rooms"], queryFn: listRooms });
  const { data: cameras = [] } = useQuery({ queryKey: ["cameras"], queryFn: () => listCameras({ page_size: 200 }) });
  const [tab, setTab] = useState<Tab>("Rooms");
  const [selectedRoomId, setSelectedRoomId] = useState<string | null>(null);
  const [draftRoom, setDraftRoom] = useState<Room | null>(null);
  const [selectedCameraId, setSelectedCameraId] = useState<string | null>(null);
  const [isCreatingCamera, setIsCreatingCamera] = useState(false);
  const [calibrationRuns, setCalibrationRuns] = useState<CameraCalibrationRun[]>([]);

  useEffect(() => {
    if (!selectedRoomId && rooms.length) {
      setSelectedRoomId(rooms[0].id);
    }
  }, [rooms, selectedRoomId]);

  useEffect(() => {
    if (!selectedCameraId && cameras.length) {
      setSelectedCameraId(cameras[0].id);
    }
  }, [cameras, selectedCameraId]);

  useEffect(() => {
    const socket = createCameraHealthSocket((payload) => {
      queryClient.setQueryData(["cameras"], (prev: Camera[] | undefined) => {
        if (!prev) return prev;
        return prev.map((camera) =>
          camera.id === payload.camera_id
            ? { ...camera, last_health: payload.last_health, last_seen_at: payload.last_seen_at }
            : camera
        );
      });
    });
    return () => socket.close();
  }, [queryClient]);

  useEffect(() => {
    if (!selectedCameraId) return;
    listCalibrationRuns(selectedCameraId).then(setCalibrationRuns);
  }, [selectedCameraId]);

  const selectedRoom = useMemo(() => {
    if (draftRoom) return draftRoom;
    return rooms.find((room) => room.id === selectedRoomId) ?? null;
  }, [rooms, selectedRoomId, draftRoom]);

  const displayPolygon = useMemo(() => {
    if (!selectedRoom) return [];
    const polygon = selectedRoom.polygon_mm ?? [];
    if (polygon.length > 1) {
      const first = polygon[0];
      const last = polygon[polygon.length - 1];
      if (first.x_mm === last.x_mm && first.y_mm === last.y_mm) {
        return polygon.slice(0, -1);
      }
    }
    return polygon;
  }, [selectedRoom]);

  const selectedCamera = useMemo(() => {
    if (isCreatingCamera) return null;
    return cameras.find((camera) => camera.id === selectedCameraId) ?? null;
  }, [cameras, selectedCameraId, isCreatingCamera]);

  const createRoomMutation = useMutation({
    mutationFn: (payload: Partial<Room>) => createRoom(payload),
    onSuccess: async (room) => {
      setDraftRoom(null);
      setSelectedRoomId(room.id);
      await queryClient.invalidateQueries({ queryKey: ["rooms"] });
    }
  });

  const updateRoomMutation = useMutation({
    mutationFn: ({ id, payload }: { id: string; payload: Partial<Room> }) => updateRoom(id, payload),
    onSuccess: async () => {
      await queryClient.invalidateQueries({ queryKey: ["rooms"] });
    }
  });

  const deleteRoomMutation = useMutation({
    mutationFn: deleteRoom,
    onSuccess: async () => {
      await queryClient.invalidateQueries({ queryKey: ["rooms"] });
      await queryClient.invalidateQueries({ queryKey: ["cameras"] });
    }
  });

  const deleteCameraMutation = useMutation({
    mutationFn: deleteCamera,
    onSuccess: async () => {
      await queryClient.invalidateQueries({ queryKey: ["cameras"] });
    }
  });

  const handleRoomSave = async (payload: Partial<Room>) => {
    if (draftRoom) {
      await createRoomMutation.mutateAsync(payload);
    } else if (selectedRoomId) {
      await updateRoomMutation.mutateAsync({ id: selectedRoomId, payload });
    }
  };

  const handleRecenter = async (payload: { origin_x_mm: number; origin_y_mm: number; rotation_deg: number }) => {
    if (draftRoom) {
      setDraftRoom({ ...draftRoom, ...payload });
    } else if (selectedRoomId) {
      await recenterRoom(selectedRoomId, payload);
      await queryClient.invalidateQueries({ queryKey: ["rooms"] });
    }
  };

  const handleRoomDelete = async (id: string) => {
    await deleteRoomMutation.mutateAsync(id);
    if (selectedRoomId === id) {
      setSelectedRoomId(null);
    }
  };

  const handleCameraDelete = async (ids: string[]) => {
    await Promise.all(ids.map((id) => deleteCameraMutation.mutateAsync(id)));
  };

  const handleCameraPosition = async (cameraId: string, position: { x_mm: number; y_mm: number }) => {
    const camera = cameras.find((item) => item.id === cameraId);
    if (!camera) return;
    await updateCamera(cameraId, { position_mm: { ...camera.position_mm, ...position } });
    await queryClient.invalidateQueries({ queryKey: ["cameras"] });
  };

  const handleCameraSaved = async (camera: Camera) => {
    setIsCreatingCamera(false);
    setSelectedCameraId(camera.id);
    await queryClient.invalidateQueries({ queryKey: ["cameras"] });
  };

  return (
    <div className="flex min-h-screen flex-col gap-6 bg-slate-950 p-8 text-slate-100">
      <header>
        <h1 className="text-2xl font-semibold">Spaces &amp; Cameras</h1>
        <p className="text-sm text-slate-400">Configure rooms, place cameras and run calibrations.</p>
      </header>

      <nav className="flex gap-3">
        {Tabs.map((label) => (
          <button
            key={label}
            onClick={() => setTab(label)}
            className={`rounded-full px-4 py-2 text-sm font-semibold ${
              tab === label ? "bg-emerald-500 text-slate-900" : "bg-slate-800 text-slate-200 hover:bg-slate-700"
            }`}
          >
            {label}
          </button>
        ))}
      </nav>

      {tab === "Rooms" && (
        <div className="grid gap-6 lg:grid-cols-[300px,1fr]">
          <RoomListPanel
            rooms={rooms}
            selectedId={draftRoom ? draftRoom.id : selectedRoomId}
            onSelect={(id) => {
              setDraftRoom(null);
              setSelectedRoomId(id);
            }}
            onCreate={() => {
              const template: Room = {
                id: "draft",
                name: "New Room",
                level: 1,
                origin_x_mm: 0,
                origin_y_mm: 0,
                rotation_deg: 0,
                polygon_mm: [
                  { x_mm: 0, y_mm: 0 },
                  { x_mm: 4000, y_mm: 0 },
                  { x_mm: 4000, y_mm: 3500 }
                ],
                ceiling_height_mm: 2500,
                metadata: {},
                created_at: "",
                updated_at: ""
              } as unknown as Room;
              setDraftRoom(template);
              setSelectedRoomId(template.id);
            }}
            onDelete={handleRoomDelete}
          />
          <div className="rounded border border-slate-800 bg-slate-900 p-6">
            {selectedRoom ? (
              <RoomEditor room={selectedRoom} onSave={handleRoomSave} onRecenter={handleRecenter} isSaving={createRoomMutation.isPending || updateRoomMutation.isPending} />
            ) : (
              <p className="text-sm text-slate-400">Select a room to edit its geometry.</p>
            )}
          </div>
        </div>
      )}

      {tab === "Cameras" && (
        <div className="grid gap-6 lg:grid-cols-[1fr,400px]">
          <div className="rounded border border-slate-800 bg-slate-900 p-6">
            <div className="mb-4 flex items-center justify-between">
              <h2 className="text-lg font-semibold">Floorplan</h2>
              <button
                className="rounded bg-emerald-500 px-3 py-2 text-sm font-semibold text-slate-900"
                onClick={() => {
                  setIsCreatingCamera(true);
                  setSelectedCameraId(null);
                }}
              >
                Add camera
              </button>
            </div>
            <Floorplan
              polygon={displayPolygon}
              cameras={cameras}
              onCameraChange={handleCameraPosition}
              onSelectCamera={(id) => {
                setIsCreatingCamera(false);
                setSelectedCameraId(id);
              }}
              selectedCameraId={selectedCameraId}
            />
          </div>
          <div className="space-y-6">
            <CameraList
              cameras={cameras}
              selectedId={selectedCameraId}
              onSelect={(id) => {
                setIsCreatingCamera(false);
                setSelectedCameraId(id);
              }}
              onDeleteSelected={handleCameraDelete}
            />
            <div className="rounded border border-slate-800 bg-slate-900 p-6">
              <CameraForm
                rooms={rooms.map((room) => ({ id: room.id, name: room.name }))}
                camera={isCreatingCamera ? null : selectedCamera}
                onSaved={handleCameraSaved}
                onRunCalibration={(camera, run) => {
                  setTab("Calibration");
                  setSelectedCameraId(camera.id);
                  setCalibrationRuns((prev) => [run, ...prev]);
                }}
              />
            </div>
          </div>
        </div>
      )}

      {tab === "Calibration" && (
        <div className="grid gap-6 lg:grid-cols-[1fr,350px]">
          <div className="rounded border border-slate-800 bg-slate-900 p-6">
            <CalibrationWizard
              cameras={cameras}
              onIntrinsicsSaved={async (cameraId) => {
                await queryClient.invalidateQueries({ queryKey: ["cameras"] });
                const runs = await listCalibrationRuns(cameraId);
                setCalibrationRuns(runs);
              }}
            />
          </div>
          <div className="rounded border border-slate-800 bg-slate-900 p-6">
            <h3 className="mb-3 text-sm font-semibold text-slate-100">Recent runs</h3>
            <CalibrationRunsTable runs={calibrationRuns} />
          </div>
        </div>
      )}
    </div>
  );
};

export default SpacesPage;
