import React, { useMemo, useState } from "react";
import { Camera } from "../../lib/api";

interface Props {
  cameras: Camera[];
  selectedId: string | null;
  onSelect: (id: string) => void;
  onDeleteSelected: (ids: string[]) => void;
}

const CameraList: React.FC<Props> = ({ cameras, selectedId, onSelect, onDeleteSelected }) => {
  const [selected, setSelected] = useState<Record<string, boolean>>({});
  const [search, setSearch] = useState("");

  const filtered = useMemo(() => {
    const query = search.toLowerCase();
    return cameras.filter(
      (camera) =>
        camera.name.toLowerCase().includes(query) ||
        (camera.room_name ?? "").toLowerCase().includes(query) ||
        camera.last_health.toLowerCase().includes(query)
    );
  }, [cameras, search]);

  const toggleSelected = (id: string) => {
    setSelected((prev) => ({ ...prev, [id]: !prev[id] }));
  };

  const selectedIds = useMemo(() => Object.keys(selected).filter((key) => selected[key]), [selected]);

  return (
    <div className="flex flex-col gap-3">
      <div className="flex items-center gap-2">
        <input
          type="text"
          value={search}
          onChange={(event) => setSearch(event.target.value)}
          placeholder="Search cameras"
          className="flex-1 rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm"
        />
        <button
          className="rounded bg-red-500 px-3 py-2 text-sm font-medium text-white disabled:opacity-40"
          disabled={selectedIds.length === 0}
          onClick={() => onDeleteSelected(selectedIds)}
        >
          Delete selected ({selectedIds.length})
        </button>
      </div>
      <div className="overflow-x-auto rounded border border-slate-800">
        <table className="min-w-full divide-y divide-slate-800 text-sm">
          <thead className="bg-slate-900 text-xs uppercase tracking-wide text-slate-400">
            <tr>
              <th className="px-3 py-2 text-left">Select</th>
              <th className="px-3 py-2 text-left">Name</th>
              <th className="px-3 py-2 text-left">Room</th>
              <th className="px-3 py-2 text-left">Health</th>
              <th className="px-3 py-2 text-left">Calibration</th>
              <th className="px-3 py-2 text-left">Last seen</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-900">
            {filtered.map((camera) => (
              <tr
                key={camera.id}
                className={`cursor-pointer ${selectedId === camera.id ? "bg-slate-800" : "hover:bg-slate-900"}`}
                onClick={() => onSelect(camera.id)}
              >
                <td className="px-3 py-2">
                  <input
                    type="checkbox"
                    checked={Boolean(selected[camera.id])}
                    onChange={(event) => {
                      event.stopPropagation();
                      toggleSelected(camera.id);
                    }}
                  />
                </td>
                <td className="px-3 py-2 font-medium text-slate-200">{camera.name}</td>
                <td className="px-3 py-2 text-slate-300">{camera.room_name}</td>
                <td className="px-3 py-2">
                  <span className={`rounded px-2 py-1 text-xs font-semibold ${healthColor(camera.last_health)}`}>
                    {camera.last_health}
                  </span>
                </td>
                <td className="px-3 py-2 text-slate-300">{camera.calibration_state}</td>
                <td className="px-3 py-2 text-slate-400">{camera.last_seen_at ?? "–"}</td>
              </tr>
            ))}
            {filtered.length === 0 && (
              <tr>
                <td colSpan={6} className="px-3 py-6 text-center text-xs text-slate-500">
                  No cameras match your filters.
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
};

function healthColor(status: string) {
  switch (status) {
    case "OK":
      return "bg-emerald-500/20 text-emerald-300";
    case "DEGRADED":
      return "bg-amber-500/20 text-amber-300";
    case "OFFLINE":
      return "bg-red-500/20 text-red-300";
    default:
      return "bg-slate-700/40 text-slate-300";
  }
}

export default CameraList;
