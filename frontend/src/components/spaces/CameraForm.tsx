import React, { useEffect, useState } from "react";
import { z } from "zod";

import {
  Camera,
  CameraCalibrationRun,
  createCamera,
  startCalibration,
  testCameraConnection,
  updateCamera
} from "../../lib/api";

const cameraSchema = z.object({
  name: z.string().min(1),
  room: z.string().uuid(),
  make: z.string().min(1),
  model: z.string().min(1),
  rtsp_url: z.string().url().or(z.literal("")).optional(),
  webrtc_url: z.string().url().or(z.literal("")).nullable().optional(),
  resolution_w: z.number().min(1),
  resolution_h: z.number().min(1),
  fps: z.number().min(1),
  fov_deg: z.number().min(10).max(160),
  position_mm: z.object({ x_mm: z.number(), y_mm: z.number(), z_mm: z.number() }),
  yaw_deg: z.number(),
  pitch_deg: z.number(),
  roll_deg: z.number()
});

interface Props {
  rooms: Array<{ id: string; name: string }>;
  camera: Camera | null;
  onSaved: (camera: Camera) => void;
  onRunCalibration: (camera: Camera, run: CameraCalibrationRun) => void;
}

const CameraForm: React.FC<Props> = ({ rooms, camera, onSaved, onRunCalibration }) => {
  const buildDefault = () => ({
    id: "",
    name: "",
    room: rooms[0]?.id ?? "",
    make: "Reolink",
    model: "",
    rtsp_url: "",
    webrtc_url: "",
    resolution_w: 2560,
    resolution_h: 1920,
    fps: 15,
    fov_deg: 90,
    position_mm: { x_mm: 0, y_mm: 0, z_mm: 2500 },
    yaw_deg: 45,
    pitch_deg: -15,
    roll_deg: 0,
    calibration_state: "UNSET",
    last_health: "UNKNOWN",
    metadata: {}
  });
  const [form, setForm] = useState(() => camera ?? buildDefault());
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isSaving, setIsSaving] = useState(false);
  const [connectionResult, setConnectionResult] = useState<string | null>(null);

  useEffect(() => {
    if (camera) {
      setForm(camera);
    } else {
      setForm(buildDefault());
    }
    setErrors({});
    setConnectionResult(null);
  }, [camera, rooms]);

  const handleChange = (field: string, value: any) => {
    setForm((prev) => ({ ...prev, [field]: value }));
  };

  const handleNestedChange = (key: "x_mm" | "y_mm" | "z_mm", value: number) => {
    setForm((prev) => ({ ...prev, position_mm: { ...prev.position_mm, [key]: value } }));
  };

  const submit = async () => {
    setIsSaving(true);
    setErrors({});
    try {
      const payload = cameraSchema.parse({
        name: form.name,
        room: form.room,
        make: form.make,
        model: form.model,
        rtsp_url: (form as any).rtsp_url ?? "",
        webrtc_url: form.webrtc_url ?? "",
        resolution_w: form.resolution_w,
        resolution_h: form.resolution_h,
        fps: form.fps,
        fov_deg: form.fov_deg,
        position_mm: form.position_mm,
        yaw_deg: form.yaw_deg,
        pitch_deg: form.pitch_deg,
        roll_deg: form.roll_deg
      });
      const response = form.id
        ? await updateCamera(form.id, payload)
        : await createCamera(payload as any);
      onSaved(response);
      setConnectionResult(null);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const map: Record<string, string> = {};
        error.errors.forEach((item) => {
          map[item.path.join(".")] = item.message;
        });
        setErrors(map);
      }
    } finally {
      setIsSaving(false);
    }
  };

  const probeConnection = async () => {
    if (!form.id) {
      setConnectionResult("Save camera first to test connection");
      return;
    }
    const result = await testCameraConnection(form.id);
    setConnectionResult(result.ok ? "Camera reachable" : `Connection failed: ${result.message ?? "unknown"}`);
  };

  const runCalibration = async () => {
    if (!form.id) {
      setConnectionResult("Save camera before running calibration");
      return;
    }
    const response = await startCalibration(form.id, {
      method: "CHECKERBOARD",
      board_spec: { squaresX: 7, squaresY: 5, squareLength_mm: 30 }
    });
    onRunCalibration(form as Camera, response.run as CameraCalibrationRun);
  };

  return (
    <div className="flex flex-col gap-4">
      <div className="grid grid-cols-2 gap-4 md:grid-cols-3">
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Name</span>
          <input
            value={form.name}
            onChange={(event) => handleChange("name", event.target.value)}
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
          />
          {errors.name && <span className="text-xs text-red-400">{errors.name}</span>}
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Room</span>
          <select
            value={form.room}
            onChange={(event) => handleChange("room", event.target.value)}
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
          >
            {rooms.map((room) => (
              <option key={room.id} value={room.id}>
                {room.name}
              </option>
            ))}
          </select>
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Make</span>
          <input
            value={form.make}
            onChange={(event) => handleChange("make", event.target.value)}
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
          />
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Model</span>
          <input
            value={form.model}
            onChange={(event) => handleChange("model", event.target.value)}
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
          />
        </label>
        <label className="flex flex-col text-xs col-span-2">
          <span className="mb-1 text-slate-400">RTSP URL</span>
          <input
            value={(form as any).rtsp_url ?? ""}
            onChange={(event) => handleChange("rtsp_url", event.target.value)}
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            placeholder="rtsp://user:pass@host/stream"
          />
          {errors.rtsp_url && <span className="text-xs text-red-400">{errors.rtsp_url}</span>}
        </label>
        <label className="flex flex-col text-xs col-span-2">
          <span className="mb-1 text-slate-400">WebRTC URL (optional)</span>
          <input
            value={(form.webrtc_url as string | null) ?? ""}
            onChange={(event) => handleChange("webrtc_url", event.target.value)}
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
          />
        </label>
      </div>

      <div className="grid grid-cols-2 gap-4 md:grid-cols-4">
        <NumberField label="Resolution W" value={form.resolution_w} onChange={(value) => handleChange("resolution_w", value)} />
        <NumberField label="Resolution H" value={form.resolution_h} onChange={(value) => handleChange("resolution_h", value)} />
        <NumberField label="FPS" value={form.fps} onChange={(value) => handleChange("fps", value)} />
        <NumberField label="FOV (deg)" value={form.fov_deg} onChange={(value) => handleChange("fov_deg", value)} />
        <NumberField label="Pos X (mm)" value={form.position_mm.x_mm} onChange={(value) => handleNestedChange("x_mm", value)} />
        <NumberField label="Pos Y (mm)" value={form.position_mm.y_mm} onChange={(value) => handleNestedChange("y_mm", value)} />
        <NumberField label="Pos Z (mm)" value={form.position_mm.z_mm} onChange={(value) => handleNestedChange("z_mm", value)} />
        <NumberField label="Yaw" value={form.yaw_deg} onChange={(value) => handleChange("yaw_deg", value)} />
        <NumberField label="Pitch" value={form.pitch_deg} onChange={(value) => handleChange("pitch_deg", value)} />
        <NumberField label="Roll" value={form.roll_deg} onChange={(value) => handleChange("roll_deg", value)} />
      </div>

      <div className="flex flex-col gap-2">
        <div className="flex flex-wrap gap-2 text-sm">
          <button
            onClick={submit}
            className="rounded bg-emerald-500 px-4 py-2 font-semibold text-slate-900 hover:bg-emerald-400 disabled:opacity-50"
            disabled={isSaving}
          >
            {isSaving ? "Saving..." : camera ? "Update camera" : "Create camera"}
          </button>
          <button
            onClick={probeConnection}
            className="rounded bg-indigo-500 px-4 py-2 font-semibold text-white hover:bg-indigo-400"
          >
            Test connection
          </button>
          <button onClick={runCalibration} className="rounded bg-amber-500 px-4 py-2 font-semibold text-slate-900 hover:bg-amber-400">
            Start calibration
          </button>
        </div>
        {connectionResult && <span className="text-xs text-slate-300">{connectionResult}</span>}
      </div>
    </div>
  );
};

interface NumberFieldProps {
  label: string;
  value: number;
  onChange: (value: number) => void;
}

const NumberField: React.FC<NumberFieldProps> = ({ label, value, onChange }) => (
  <label className="flex flex-col text-xs">
    <span className="mb-1 text-slate-400">{label}</span>
    <input
      type="number"
      value={value}
      onChange={(event) => onChange(Number(event.target.value))}
      className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
    />
  </label>
);

export default CameraForm;
