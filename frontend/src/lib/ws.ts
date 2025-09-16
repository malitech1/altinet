export type CameraHealthMessage = {
  camera_id: string;
  last_health: string;
  last_seen_at?: string;
};

export type CalibrationMessage = {
  run_id: string;
  pct: number;
  msg: string;
};

const WS_BASE = (import.meta.env.VITE_WS_URL as string | undefined) ?? "ws://localhost:8000";

export function createCameraHealthSocket(onMessage: (msg: CameraHealthMessage) => void) {
  const socket = new WebSocket(`${WS_BASE}/ws/cameras/`);
  socket.onmessage = (event) => {
    onMessage(JSON.parse(event.data));
  };
  return socket;
}

export function createCalibrationSocket(cameraId: string, onMessage: (msg: CalibrationMessage) => void) {
  const socket = new WebSocket(`${WS_BASE}/ws/calibration/${cameraId}`);
  socket.onmessage = (event) => onMessage(JSON.parse(event.data));
  return socket;
}
