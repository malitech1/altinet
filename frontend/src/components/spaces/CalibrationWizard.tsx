import React, { useEffect, useMemo, useState } from "react";

import {
  Camera,
  CameraCalibrationRun,
  cancelCalibration,
  listCalibrationRuns,
  saveIntrinsics,
  startCalibration
} from "../../lib/api";
import { createCalibrationSocket } from "../../lib/ws";
import LivePreview from "./LivePreview";

interface Props {
  cameras: Camera[];
  onIntrinsicsSaved: (cameraId: string) => void;
}

const methods = ["CHECKERBOARD", "CHARUCO", "APRILTAG"] as const;

type Method = (typeof methods)[number];

const CalibrationWizard: React.FC<Props> = ({ cameras, onIntrinsicsSaved }) => {
  const [step, setStep] = useState(0);
  const [selectedCameraId, setSelectedCameraId] = useState<string>("");
  const [method, setMethod] = useState<Method>("CHECKERBOARD");
  const [boardSpec, setBoardSpec] = useState({ squaresX: 7, squaresY: 5, squareLength_mm: 30 });
  const [run, setRun] = useState<CameraCalibrationRun | null>(null);
  const [progress, setProgress] = useState<{ pct: number; msg: string } | null>(null);
  const [intrinsics, setIntrinsics] = useState("{\n  \"fx\": 1000,\n  \"fy\": 1000,\n  \"cx\": 960,\n  \"cy\": 540\n}");
  const [runs, setRuns] = useState<CameraCalibrationRun[]>([]);
  const [intrinsicsError, setIntrinsicsError] = useState<string | null>(null);

  useEffect(() => {
    if (!selectedCameraId) return;
    listCalibrationRuns(selectedCameraId).then(setRuns);
  }, [selectedCameraId, run?.status]);

  useEffect(() => {
    if (!run || !selectedCameraId) return;
    const socket = createCalibrationSocket(selectedCameraId, (payload) => {
      setProgress({ pct: payload.pct, msg: payload.msg });
    });
    return () => socket.close();
  }, [selectedCameraId, run?.id]);

  const selectedCamera = useMemo(() => cameras.find((cam) => cam.id === selectedCameraId) ?? null, [cameras, selectedCameraId]);

  const start = async () => {
    if (!selectedCameraId) return;
    const response = await startCalibration(selectedCameraId, { method, board_spec: boardSpec });
    setRun(response.run as CameraCalibrationRun);
    setProgress({ pct: 5, msg: "Queued" });
    setStep(3);
  };

  const cancel = async () => {
    if (!selectedCameraId) return;
    await cancelCalibration(selectedCameraId, { run_id: run?.id });
    setProgress({ pct: 0, msg: "Cancelled" });
  };

  const persistIntrinsics = async () => {
    if (!selectedCameraId) return;
    try {
      const parsed = JSON.parse(intrinsics);
      await saveIntrinsics(selectedCameraId, { intrinsics: parsed, error_rms: run?.error_rms ?? 0.5 });
      onIntrinsicsSaved(selectedCameraId);
      setIntrinsicsError(null);
      setStep(0);
      setRun(null);
      setProgress(null);
    } catch (error) {
      setIntrinsicsError("Invalid JSON payload");
    }
  };

  return (
    <div className="flex flex-col gap-4">
      <ol className="flex items-center gap-3 text-xs uppercase tracking-wide text-slate-400">
        {["Camera", "Board", "Capture", "Solve", "Save"].map((label, idx) => (
          <li key={label} className={`rounded-full px-3 py-1 ${step >= idx ? "bg-emerald-500 text-slate-900" : "bg-slate-800"}`}>
            {label}
          </li>
        ))}
      </ol>

      {step === 0 && (
        <div className="grid grid-cols-2 gap-3">
          {cameras.map((camera) => (
            <button
              key={camera.id}
              onClick={() => {
                setSelectedCameraId(camera.id);
                setStep(1);
              }}
              className={`rounded border px-3 py-3 text-left ${
                selectedCameraId === camera.id ? "border-emerald-500" : "border-slate-700 hover:border-slate-500"
              }`}
            >
              <div className="text-sm font-semibold text-slate-100">{camera.name}</div>
              <div className="text-xs text-slate-400">Room: {camera.room_name}</div>
              <div className="text-xs text-slate-400">Health: {camera.last_health}</div>
            </button>
          ))}
        </div>
      )}

      {step === 1 && (
        <div className="grid gap-4 md:grid-cols-2">
          <div className="flex flex-col gap-3">
            <label className="flex flex-col text-xs">
              <span className="mb-1 text-slate-400">Calibration method</span>
              <select
                value={method}
                onChange={(event) => setMethod(event.target.value as Method)}
                className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
              >
                {methods.map((option) => (
                  <option key={option} value={option}>
                    {option}
                  </option>
                ))}
              </select>
            </label>
            <label className="flex flex-col text-xs">
              <span className="mb-1 text-slate-400">Squares X</span>
              <input
                type="number"
                value={boardSpec.squaresX}
                onChange={(event) => setBoardSpec((prev) => ({ ...prev, squaresX: Number(event.target.value) }))}
                className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
              />
            </label>
            <label className="flex flex-col text-xs">
              <span className="mb-1 text-slate-400">Squares Y</span>
              <input
                type="number"
                value={boardSpec.squaresY}
                onChange={(event) => setBoardSpec((prev) => ({ ...prev, squaresY: Number(event.target.value) }))}
                className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
              />
            </label>
            <label className="flex flex-col text-xs">
              <span className="mb-1 text-slate-400">Square length (mm)</span>
              <input
                type="number"
                value={boardSpec.squareLength_mm}
                onChange={(event) => setBoardSpec((prev) => ({ ...prev, squareLength_mm: Number(event.target.value) }))}
                className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
              />
            </label>
            <div className="flex gap-2">
              <button className="rounded bg-emerald-500 px-4 py-2 text-sm font-semibold text-slate-900" onClick={() => setStep(2)}>
                Next: capture
              </button>
              <button className="rounded bg-slate-800 px-4 py-2 text-sm text-slate-200" onClick={() => setStep(0)}>
                Back
              </button>
            </div>
          </div>
          <div className="rounded border border-slate-800 bg-slate-900 p-4 text-xs text-slate-300">
            <p className="mb-2 font-semibold text-slate-100">Board tips</p>
            <p>Use a matte checkerboard print. Keep the board flat and visible across the entire frame for best results.</p>
          </div>
        </div>
      )}

      {step === 2 && (
        <div className="flex flex-col gap-3">
          <LivePreview url={selectedCamera?.webrtc_url ?? null} />
          <p className="text-xs text-slate-300">
            Move the board through the field of view. When ready, continue to solving. Frames are captured automatically every 2
            seconds in this demo.
          </p>
          <div className="flex gap-2">
            <button className="rounded bg-emerald-500 px-4 py-2 text-sm font-semibold text-slate-900" onClick={start}>
              Run solve
            </button>
            <button className="rounded bg-slate-800 px-4 py-2 text-sm text-slate-200" onClick={() => setStep(1)}>
              Back
            </button>
          </div>
        </div>
      )}

      {step === 3 && (
        <div className="flex flex-col gap-3">
          <div className="rounded border border-slate-800 bg-slate-900 p-4">
            <p className="text-sm font-semibold text-slate-100">Calibration run in progress</p>
            <p className="text-xs text-slate-300">{progress?.msg ?? "Waiting for frames"}</p>
            <div className="mt-3 h-2 w-full overflow-hidden rounded bg-slate-800">
              <div className="h-full bg-emerald-500" style={{ width: `${Math.min(100, progress?.pct ?? 0)}%` }} />
            </div>
          </div>
          <div className="flex gap-2">
            <button className="rounded bg-emerald-500 px-4 py-2 text-sm font-semibold text-slate-900" onClick={() => setStep(4)}>
              Continue
            </button>
            <button className="rounded bg-red-500 px-4 py-2 text-sm font-semibold text-white" onClick={cancel}>
              Cancel
            </button>
          </div>
        </div>
      )}

      {step === 4 && (
        <div className="grid gap-4 md:grid-cols-2">
          <div className="flex flex-col gap-3">
            <p className="text-sm font-semibold text-slate-100">Calibration results</p>
            <p className="text-xs text-slate-300">RMS error: {run?.error_rms ?? "0.4"}</p>
            <textarea
              value={intrinsics}
              onChange={(event) => setIntrinsics(event.target.value)}
              rows={8}
              className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-xs font-mono text-slate-200"
            />
            <div className="flex gap-2">
              <button className="rounded bg-emerald-500 px-4 py-2 text-sm font-semibold text-slate-900" onClick={persistIntrinsics}>
                Save intrinsics
              </button>
              <button className="rounded bg-slate-800 px-4 py-2 text-sm text-slate-200" onClick={() => setStep(0)}>
                Close
              </button>
            </div>
            {intrinsicsError && <p className="text-xs text-red-400">{intrinsicsError}</p>}
          </div>
          <div className="rounded border border-slate-800 bg-slate-900 p-4 text-xs text-slate-300">
            <p className="mb-2 font-semibold text-slate-100">Recent runs</p>
            <ul className="space-y-2">
              {runs.map((item) => (
                <li key={item.id} className="rounded bg-slate-800 px-3 py-2">
                  <div className="text-xs text-slate-200">{item.method}</div>
                  <div className="text-[10px] text-slate-400">Status: {item.status}</div>
                  {item.error_rms && <div className="text-[10px] text-slate-400">RMS: {item.error_rms.toFixed(3)}</div>}
                </li>
              ))}
            </ul>
          </div>
        </div>
      )}
    </div>
  );
};

export default CalibrationWizard;
