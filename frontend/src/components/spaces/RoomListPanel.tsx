import React, { useMemo, useState } from "react";
import { Room } from "../../lib/api";

interface Props {
  rooms: Room[];
  selectedId: string | null;
  onSelect: (id: string) => void;
  onCreate: () => void;
  onDelete: (id: string) => void;
}

const RoomListPanel: React.FC<Props> = ({ rooms, selectedId, onSelect, onCreate, onDelete }) => {
  const [search, setSearch] = useState("");
  const filtered = useMemo(() => {
    if (!search) return rooms;
    return rooms.filter((room) => room.name.toLowerCase().includes(search.toLowerCase()));
  }, [rooms, search]);

  return (
    <div className="flex flex-col h-full gap-3">
      <div className="flex items-center gap-2">
        <input
          type="text"
          value={search}
          onChange={(event) => setSearch(event.target.value)}
          placeholder="Search rooms"
          className="flex-1 rounded bg-slate-900 border border-slate-700 px-3 py-2 text-sm"
        />
        <button
          onClick={onCreate}
          className="rounded bg-emerald-500 px-3 py-2 text-sm font-medium text-slate-900 hover:bg-emerald-400"
        >
          New
        </button>
      </div>
      <div className="flex-1 overflow-y-auto rounded border border-slate-800 bg-slate-950">
        <ul className="divide-y divide-slate-800">
          {filtered.map((room) => (
            <li
              key={room.id}
              className={`flex items-center justify-between px-3 py-3 text-sm transition ${
                selectedId === room.id ? "bg-slate-800" : "hover:bg-slate-900"
              }`}
            >
              <button className="flex flex-col text-left" onClick={() => onSelect(room.id)}>
                <span className="font-semibold text-slate-200">{room.name}</span>
                <span className="text-xs text-slate-400">Ceiling {room.ceiling_height_mm} mm</span>
              </button>
              <button
                className="rounded bg-red-500 px-2 py-1 text-xs font-medium text-slate-100 hover:bg-red-400"
                onClick={() => onDelete(room.id)}
              >
                Delete
              </button>
            </li>
          ))}
          {filtered.length === 0 && (
            <li className="px-3 py-6 text-center text-xs text-slate-500">No rooms yet</li>
          )}
        </ul>
      </div>
    </div>
  );
};

export default RoomListPanel;
