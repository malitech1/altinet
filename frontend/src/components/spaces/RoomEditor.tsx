import React, { useEffect, useMemo, useState } from "react";
import { z } from "zod";

import { Room } from "../../lib/api";
import { useHistoryState } from "../../lib/useHistoryState";
import Floorplan, { FloorplanPoint } from "./Floorplan";

const roomSchema = z.object({
  name: z.string().min(1),
  level: z.number().nullable(),
  ceiling_height_mm: z.number().min(1800),
  origin_x_mm: z.number(),
  origin_y_mm: z.number(),
  rotation_deg: z.number(),
  polygon_mm: z.array(z.object({ x_mm: z.number(), y_mm: z.number() })).min(3)
});

interface Props {
  room: Room | null;
  onSave: (payload: Partial<Room>) => Promise<void>;
  onRecenter: (payload: { origin_x_mm: number; origin_y_mm: number; rotation_deg: number }) => Promise<void>;
  isSaving?: boolean;
}

const RoomEditor: React.FC<Props> = ({ room, onSave, onRecenter, isSaving }) => {
  const initialPolygon: FloorplanPoint[] = useMemo(() => {
    if (!room) return [];
    const points = room.polygon_mm ?? [];
    if (points.length && points[0].x_mm === points[points.length - 1].x_mm && points[0].y_mm === points[points.length - 1].y_mm) {
      return points.slice(0, -1);
    }
    return points;
  }, [room]);
  const polygonHistory = useHistoryState<FloorplanPoint[]>(initialPolygon);
  const [form, setForm] = useState(() =>
    room
      ? {
          name: room.name,
          level: room.level,
          ceiling_height_mm: room.ceiling_height_mm,
          origin_x_mm: room.origin_x_mm,
          origin_y_mm: room.origin_y_mm,
          rotation_deg: room.rotation_deg
        }
      : {
          name: "",
          level: 1,
          ceiling_height_mm: 2500,
          origin_x_mm: 0,
          origin_y_mm: 0,
          rotation_deg: 0
        }
  );
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [addMode, setAddMode] = useState(false);
  const [showMetres, setShowMetres] = useState(false);

  useEffect(() => {
    if (room) {
      setForm({
        name: room.name,
        level: room.level,
        ceiling_height_mm: room.ceiling_height_mm,
        origin_x_mm: room.origin_x_mm,
        origin_y_mm: room.origin_y_mm,
        rotation_deg: room.rotation_deg
      });
      polygonHistory.replace(initialPolygon);
    }
  }, [room, initialPolygon]);

  const handlePolygonChange = (points: FloorplanPoint[]) => {
    if (points.length !== polygonHistory.value.length) {
      polygonHistory.setValue(points);
    } else {
      polygonHistory.updateCurrent(points);
    }
  };

  const handleSave = async () => {
    setErrors({});
    try {
      const payload = roomSchema.parse({ ...form, polygon_mm: polygonHistory.value });
      const closedPolygon = [...polygonHistory.value];
      if (closedPolygon.length && (closedPolygon[0].x_mm !== closedPolygon[closedPolygon.length - 1].x_mm || closedPolygon[0].y_mm !== closedPolygon[closedPolygon.length - 1].y_mm)) {
        closedPolygon.push({ ...closedPolygon[0] });
      }
      await onSave({ ...payload, polygon_mm: closedPolygon, metadata: room?.metadata ?? {} });
      setAddMode(false);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const map: Record<string, string> = {};
        error.errors.forEach((item) => {
          if (item.path[0]) {
            map[item.path[0].toString()] = item.message;
          }
        });
        setErrors(map);
      }
    }
  };

  const handleRecenter = async () => {
    await onRecenter({
      origin_x_mm: form.origin_x_mm,
      origin_y_mm: form.origin_y_mm,
      rotation_deg: form.rotation_deg
    });
  };

  return (
    <div className="flex h-full flex-col gap-4">
      <div className="flex flex-wrap gap-3">
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Name</span>
          <input
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            value={form.name}
            onChange={(event) => setForm((prev) => ({ ...prev, name: event.target.value }))}
          />
          {errors.name && <span className="text-xs text-red-400">{errors.name}</span>}
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Level</span>
          <input
            type="number"
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            value={form.level ?? ""}
            onChange={(event) => setForm((prev) => ({ ...prev, level: event.target.value === "" ? null : Number(event.target.value) }))}
          />
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Ceiling height (mm)</span>
          <input
            type="number"
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            value={form.ceiling_height_mm}
            onChange={(event) => setForm((prev) => ({ ...prev, ceiling_height_mm: Number(event.target.value) }))}
          />
          {errors.ceiling_height_mm && <span className="text-xs text-red-400">{errors.ceiling_height_mm}</span>}
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Origin X (mm)</span>
          <input
            type="number"
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            value={form.origin_x_mm}
            onChange={(event) => setForm((prev) => ({ ...prev, origin_x_mm: Number(event.target.value) }))}
          />
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Origin Y (mm)</span>
          <input
            type="number"
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            value={form.origin_y_mm}
            onChange={(event) => setForm((prev) => ({ ...prev, origin_y_mm: Number(event.target.value) }))}
          />
        </label>
        <label className="flex flex-col text-xs">
          <span className="mb-1 text-slate-400">Rotation (deg)</span>
          <input
            type="number"
            className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
            value={form.rotation_deg}
            onChange={(event) => setForm((prev) => ({ ...prev, rotation_deg: Number(event.target.value) }))}
          />
        </label>
        <button
          onClick={handleRecenter}
          className="self-end rounded bg-indigo-500 px-3 py-2 text-xs font-semibold text-white hover:bg-indigo-400"
        >
          Apply origin/rotation
        </button>
      </div>

      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2 text-xs text-slate-400">
          <button
            className={`rounded px-3 py-2 text-sm font-medium ${addMode ? "bg-emerald-500 text-slate-900" : "bg-slate-800 text-slate-200"}`}
            onClick={() => setAddMode((prev) => !prev)}
          >
            {addMode ? "Click to place vertices" : "Add vertices"}
          </button>
          <button
            className="rounded bg-slate-800 px-3 py-2 text-sm text-slate-200 disabled:opacity-30"
            onClick={polygonHistory.undo}
            disabled={!polygonHistory.canUndo}
          >
            Undo
          </button>
          <button
            className="rounded bg-slate-800 px-3 py-2 text-sm text-slate-200 disabled:opacity-30"
            onClick={polygonHistory.redo}
            disabled={!polygonHistory.canRedo}
          >
            Redo
          </button>
        </div>
        <label className="flex items-center gap-2 text-xs text-slate-400">
          <input type="checkbox" checked={showMetres} onChange={() => setShowMetres((prev) => !prev)} />
          View in metres
        </label>
      </div>

      <Floorplan
        polygon={polygonHistory.value}
        onPolygonChange={handlePolygonChange}
        addPointMode={addMode}
        showMetres={showMetres}
      />

      <div className="flex justify-end gap-3">
        <button
          onClick={handleSave}
          className="rounded bg-emerald-500 px-4 py-2 text-sm font-semibold text-slate-900 hover:bg-emerald-400 disabled:opacity-50"
          disabled={isSaving}
        >
          {isSaving ? "Saving..." : "Save room"}
        </button>
      </div>
    </div>
  );
};

export default RoomEditor;
