import axios from "axios";

const API_BASE = import.meta.env.VITE_API_URL ?? "http://localhost:8000/api";

export const api = axios.create({
  baseURL: API_BASE,
  headers: {
    "Content-Type": "application/json"
  }
});

export interface Room {
  id: string;
  name: string;
  level: number | null;
  origin_x_mm: number;
  origin_y_mm: number;
  rotation_deg: number;
  polygon_mm: Array<{ x_mm: number; y_mm: number }>;
  ceiling_height_mm: number;
  metadata: Record<string, unknown>;
  created_at?: string;
  updated_at?: string;
}

export interface Camera {
  id: string;
  name: string;
  room: string;
  room_name?: string;
  make: string;
  model: string;
  webrtc_url?: string | null;
  resolution_w: number;
  resolution_h: number;
  fps: number;
  fov_deg: number;
  position_mm: { x_mm: number; y_mm: number; z_mm: number };
  yaw_deg: number;
  pitch_deg: number;
  roll_deg: number;
  calibration_state: string;
  last_health: string;
  last_seen_at?: string | null;
  metadata: Record<string, any>;
  rtsp_url_masked?: string;
  created_at?: string;
  updated_at?: string;
}

export interface CameraCalibrationRun {
  id: string;
  camera: string;
  camera_name?: string;
  status: string;
  method: string;
  board_spec: Record<string, any>;
  error_rms?: number | null;
  started_at?: string | null;
  finished_at?: string | null;
  logs?: string;
}

export async function listRooms() {
  const response = await api.get<{ results: Room[] }>("/rooms/");
  return response.data.results;
}

export async function createRoom(payload: Partial<Room>) {
  const response = await api.post<Room>("/rooms/", payload);
  return response.data;
}

export async function updateRoom(id: string, payload: Partial<Room>) {
  const response = await api.patch<Room>(`/rooms/${id}/`, payload);
  return response.data;
}

export async function deleteRoom(id: string) {
  await api.delete(`/rooms/${id}/`);
}

export async function recenterRoom(id: string, payload: { origin_x_mm: number; origin_y_mm: number; rotation_deg: number }) {
  const response = await api.post<Room>(`/rooms/${id}/recenter/`, payload);
  return response.data;
}

export async function listCameras(params?: Record<string, any>) {
  const response = await api.get<{ results: Camera[] }>("/cameras/", { params });
  return response.data.results;
}

export async function createCamera(payload: Partial<Camera> & { rtsp_url?: string }) {
  const response = await api.post<Camera>("/cameras/", payload);
  return response.data;
}

export async function updateCamera(id: string, payload: Partial<Camera> & { rtsp_url?: string }) {
  const response = await api.patch<Camera>(`/cameras/${id}/`, payload);
  return response.data;
}

export async function deleteCamera(id: string) {
  await api.delete(`/cameras/${id}/`);
}

export async function testCameraConnection(id: string) {
  const response = await api.post(`/cameras/${id}/test-connection/`, {});
  return response.data;
}

export async function startCalibration(id: string, payload: { method: string; board_spec: Record<string, any> }) {
  const response = await api.post(`/cameras/${id}/calibration/start/`, payload);
  return response.data;
}

export async function cancelCalibration(id: string, payload: { run_id?: string }) {
  const response = await api.post(`/cameras/${id}/calibration/cancel/`, payload);
  return response.data;
}

export async function saveIntrinsics(id: string, payload: { intrinsics: Record<string, any>; error_rms?: number }) {
  const response = await api.post(`/cameras/${id}/calibration/save/`, payload);
  return response.data;
}

export async function listCalibrationRuns(id: string) {
  const response = await api.get<CameraCalibrationRun[]>(`/cameras/${id}/calibration/runs/`);
  return response.data;
}
