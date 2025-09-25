import React from "react";
import { Link, useLocation } from "react-router-dom";

import { useAuthActions, useAuthStatus } from "../../lib/useAuth";

const navItems = [
  { label: "Dashboard", to: "/dashboard" },
  { label: "Spaces", to: "/spaces" }
];

const AppShell: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const location = useLocation();
  const { data } = useAuthStatus();
  const { logout } = useAuthActions();

  const handleLogout = async () => {
    try {
      await logout.mutateAsync();
    } catch (error) {
      // eslint-disable-next-line no-console
      console.error("Failed to sign out", error);
    }
  };

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100">
      <div className="pointer-events-none absolute inset-0 overflow-hidden">
        <div className="pointer-events-none absolute -top-64 right-1/2 h-96 w-96 translate-x-1/2 rounded-full bg-indigo-500/20 blur-3xl" />
        <div className="pointer-events-none absolute bottom-0 left-1/3 h-72 w-72 -translate-x-1/2 rounded-full bg-sky-500/10 blur-3xl" />
      </div>
      <header className="relative border-b border-slate-800/60 bg-slate-950/70 backdrop-blur">
        <div className="mx-auto flex max-w-6xl flex-wrap items-center justify-between gap-4 px-6 py-4">
          <Link to="/dashboard" className="flex items-center gap-3 text-lg font-semibold tracking-wide">
            <span className="flex h-10 w-10 items-center justify-center rounded-2xl bg-indigo-500/20 text-base font-bold text-indigo-300">
              AI
            </span>
            Altinet Control
          </Link>
          <nav className="flex items-center gap-3 text-sm">
            {navItems.map((item) => {
              const isActive = location.pathname.startsWith(item.to);
              return (
                <Link
                  key={item.to}
                  to={item.to}
                  className={`rounded-full px-4 py-2 transition ${
                    isActive
                      ? "bg-slate-800/90 text-white shadow-inner"
                      : "text-slate-400 hover:bg-slate-800/60 hover:text-white"
                  }`}
                >
                  {item.label}
                </Link>
              );
            })}
          </nav>
          <div className="flex items-center gap-4 text-sm">
            {data?.user && (
              <div className="hidden text-right md:block">
                <p className="font-medium text-slate-200">{data.user.username}</p>
                <p className="text-xs text-slate-500">Operator</p>
              </div>
            )}
            <button
              type="button"
              onClick={handleLogout}
              disabled={logout.isPending}
              className="rounded-full border border-slate-700/70 px-4 py-2 text-sm font-medium text-slate-300 transition hover:border-slate-500 hover:text-white disabled:cursor-not-allowed disabled:opacity-60"
            >
              {logout.isPending ? "Signing out…" : "Sign out"}
            </button>
          </div>
        </div>
      </header>
      <main className="relative z-10 mx-auto w-full max-w-6xl px-6 py-10">{children}</main>
    </div>
  );
};

export default AppShell;

