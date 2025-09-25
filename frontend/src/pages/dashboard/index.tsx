import React from "react";
import { useQuery } from "@tanstack/react-query";

import AppShell from "../../components/layout/AppShell";
import LivePresenceFeed from "../../components/dashboard/LivePresenceFeed";
import { listRooms } from "../../lib/api";
import { useAuthStatus } from "../../lib/useAuth";

const DashboardPage: React.FC = () => {
  const { data: auth } = useAuthStatus();
  const { data: rooms = [], isLoading } = useQuery({ queryKey: ["rooms"], queryFn: listRooms });

  return (
    <AppShell>
      <section className="space-y-12">
        <div className="rounded-3xl border border-slate-800/60 bg-slate-900/60 p-8 text-slate-100 shadow-[0_0_45px_-30px_rgba(79,70,229,0.8)]">
          <p className="text-sm uppercase tracking-[0.5em] text-indigo-200/80">Control centre</p>
          <h1 className="mt-4 text-3xl font-semibold md:text-4xl">
            {auth?.user ? `Welcome back, ${auth.user.first_name || auth.user.username}` : "Altinet dashboard"}
          </h1>
          <p className="mt-4 max-w-3xl text-sm text-slate-400 md:text-base">
            Monitor detections, confirm occupancy and orchestrate your sensing network. The live feed below shows every track broadcast by the perception stack in real time.
          </p>
        </div>
        {isLoading ? (
          <div className="flex min-h-[320px] items-center justify-center rounded-3xl border border-slate-800/60 bg-slate-900/60 text-sm uppercase tracking-[0.4em] text-slate-500">
            Loading rooms…
          </div>
        ) : (
          <LivePresenceFeed rooms={rooms} />
        )}
      </section>
    </AppShell>
  );
};

export default DashboardPage;

