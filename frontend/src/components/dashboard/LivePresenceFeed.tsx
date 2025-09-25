import React, { useEffect, useMemo, useState } from "react";

import { Room } from "../../lib/api";
import { createPersonTrackSocket, PersonTrackMessage, RoomPresenceMessage } from "../../lib/ws";

type PersonState = {
  trackId: string;
  roomId: string;
  centroid: [number, number];
  lastSeen: number;
};

const STALE_TIMEOUT_MS = 12_000;

const clamp = (value: number, min: number, max: number) => Math.min(Math.max(value, min), max);

const normaliseCentroid = (value: unknown): [number, number] => {
  if (Array.isArray(value) && value.length >= 2) {
    const x = Number.isFinite(Number(value[0])) ? Number(value[0]) : 0.5;
    const y = Number.isFinite(Number(value[1])) ? Number(value[1]) : 0.5;
    return [x, y];
  }
  return [0.5, 0.5];
};

const LivePresenceFeed: React.FC<{ rooms: Room[] }> = ({ rooms }) => {
  const [people, setPeople] = useState<Record<string, PersonState>>({});

  useEffect(() => {
    const socket = createPersonTrackSocket((message) => {
      setPeople((current) => {
        if ((message as PersonTrackMessage).event === "TRACK") {
          const trackMessage = message as PersonTrackMessage;
          const centroid = normaliseCentroid(trackMessage.centroid);
          return {
            ...current,
            [trackMessage.track_id]: {
              trackId: trackMessage.track_id,
              roomId: trackMessage.room_id,
              centroid,
              lastSeen: Date.now()
            }
          };
        }

        const presenceMessage = message as RoomPresenceMessage;
        const allowed = new Set(presenceMessage.track_ids);
        const next: Record<string, PersonState> = {};
        for (const [trackId, person] of Object.entries(current)) {
          if (person.roomId !== presenceMessage.room_id) {
            next[trackId] = person;
          } else if (allowed.has(trackId)) {
            next[trackId] = person;
          }
        }
        return next;
      });
    });
    return () => socket.close();
  }, []);

  useEffect(() => {
    const interval = window.setInterval(() => {
      setPeople((current) => {
        const now = Date.now();
        let hasChanges = false;
        const next: Record<string, PersonState> = {};
        for (const [trackId, person] of Object.entries(current)) {
          if (now - person.lastSeen > STALE_TIMEOUT_MS) {
            hasChanges = true;
            continue;
          }
          next[trackId] = person;
        }
        return hasChanges ? next : current;
      });
    }, 4_000);
    return () => window.clearInterval(interval);
  }, []);

  const roomsWithPeople = useMemo(() => {
    const lookup: Record<string, PersonState[]> = {};
    for (const person of Object.values(people)) {
      lookup[person.roomId] = lookup[person.roomId] ?? [];
      lookup[person.roomId].push(person);
    }
    return rooms.map((room) => ({
      ...room,
      occupants: lookup[room.id] ?? []
    }));
  }, [people, rooms]);

  const totalPeople = Object.keys(people).length;

  return (
    <div className="space-y-8">
      <div className="flex flex-wrap items-end justify-between gap-4">
        <div>
          <h2 className="text-2xl font-semibold text-slate-100">Live presence</h2>
          <p className="text-sm text-slate-400">
            Tracking nodes stream positions in real time. Each dot represents a detected person inside a calibrated room.
          </p>
        </div>
        <div className="rounded-full border border-slate-700/60 bg-slate-900/80 px-5 py-2 text-sm font-semibold text-slate-200">
          {totalPeople === 0 ? "No people detected" : `${totalPeople} ${totalPeople === 1 ? "person" : "people"} inside`}
        </div>
      </div>
      <div className="grid gap-6 md:grid-cols-2 xl:grid-cols-3">
        {roomsWithPeople.map((room) => (
          <div key={room.id} className="relative overflow-hidden rounded-3xl border border-slate-800/60 bg-slate-900/60 p-6">
            <div className="flex items-baseline justify-between">
              <div>
                <h3 className="text-lg font-semibold text-slate-100">{room.name}</h3>
                {room.level !== null && (
                  <p className="text-xs uppercase tracking-[0.3em] text-slate-500">Level {room.level}</p>
                )}
              </div>
              <span className="text-sm text-slate-400">
                {room.occupants.length} {room.occupants.length === 1 ? "person" : "people"}
              </span>
            </div>
            <div className="mt-5 aspect-square rounded-2xl border border-slate-800/60 bg-gradient-to-br from-slate-950 via-slate-900 to-slate-950">
              <div className="relative h-full w-full">
                <div className="absolute inset-0 bg-[radial-gradient(circle_at_center,_rgba(56,189,248,0.08),_transparent_60%)]" />
                {room.occupants.map((person) => {
                  const x = clamp(person.centroid[0] * 100, 5, 95);
                  const y = clamp(person.centroid[1] * 100, 5, 95);
                  return (
                    <div
                      key={person.trackId}
                      className="absolute flex h-8 w-8 -translate-x-1/2 -translate-y-1/2 items-center justify-center rounded-full bg-indigo-500/70 text-xs font-semibold text-white shadow-lg shadow-indigo-500/30"
                      style={{ left: `${x}%`, top: `${y}%` }}
                    >
                      #{person.trackId}
                    </div>
                  );
                })}
              </div>
            </div>
            <div className="mt-4 space-y-2 text-xs text-slate-500">
              {room.occupants.length === 0 ? (
                <p>No presence detected.</p>
              ) : (
                room.occupants.map((person) => {
                  const secondsAgo = Math.round((Date.now() - person.lastSeen) / 1000);
                  return (
                    <div key={`meta-${person.trackId}`} className="flex justify-between">
                      <span className="font-medium text-slate-300">Track #{person.trackId}</span>
                      <span>{secondsAgo <= 1 ? "moments ago" : `${secondsAgo}s ago`}</span>
                    </div>
                  );
                })
              )}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default LivePresenceFeed;

