import React from "react";
import { Link, Navigate } from "react-router-dom";

import { useAuthStatus } from "../../lib/useAuth";

const LandingPage: React.FC = () => {
  const { data, isLoading, error } = useAuthStatus();
  const hasError = Boolean(error);

  if (isLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-slate-950 text-slate-100">
        <span className="text-sm uppercase tracking-[0.4em] text-slate-500">Loading…</span>
      </div>
    );
  }

  if (data?.is_authenticated) {
    return <Navigate to="/dashboard" replace />;
  }

  if (data?.has_users) {
    return <Navigate to="/login" replace />;
  }

  return (
    <div className="relative min-h-screen overflow-hidden bg-slate-950 text-slate-100">
      <div className="pointer-events-none absolute inset-0">
        <div className="absolute -top-40 left-1/2 h-[28rem] w-[28rem] -translate-x-1/2 rounded-full bg-indigo-500/20 blur-3xl" />
        <div className="absolute bottom-10 right-20 h-72 w-72 rounded-full bg-sky-500/10 blur-3xl" />
      </div>
      <main className="relative z-10 mx-auto flex min-h-screen max-w-4xl flex-col items-center justify-center px-6 py-16 text-center">
        <span className="mb-6 rounded-full border border-indigo-500/30 bg-indigo-500/10 px-4 py-1 text-xs uppercase tracking-[0.4em] text-indigo-200">
          Welcome to Altinet
        </span>
        <h1 className="text-4xl font-semibold leading-tight text-slate-50 md:text-5xl">
          Secure perception for intelligent homes
        </h1>
        <p className="mt-6 max-w-2xl text-base text-slate-400 md:text-lg">
          Altinet orchestrates cameras, tracking and identity services so you can monitor every room with confidence. Create the first operator account to begin configuring rooms, calibrating cameras and visualising live presence.
        </p>
        {hasError && (
          <p className="mt-6 text-sm text-rose-300/80">
            Unable to contact the server. Please check your connection and try again.
          </p>
        )}
        <div className="mt-12 flex flex-col gap-4 sm:flex-row">
          <Link
            to="/register"
            className="inline-flex items-center justify-center rounded-full bg-indigo-500 px-8 py-3 text-sm font-semibold uppercase tracking-wider text-white shadow-lg shadow-indigo-500/30 transition hover:shadow-indigo-500/50"
          >
            Create operator account
          </Link>
          <Link
            to="/login"
            className="inline-flex items-center justify-center rounded-full border border-slate-700/60 px-8 py-3 text-sm font-semibold uppercase tracking-wider text-slate-200 transition hover:border-slate-500 hover:text-white"
          >
            Already onboarded? Sign in
          </Link>
        </div>
      </main>
    </div>
  );
};

export default LandingPage;

