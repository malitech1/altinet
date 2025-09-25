import React, { FormEvent, useState } from "react";
import { Link, Navigate, useLocation } from "react-router-dom";

import { useAuthActions, useAuthStatus } from "../../lib/useAuth";

const LoginPage: React.FC = () => {
  const { data, isLoading } = useAuthStatus();
  const { login } = useAuthActions();
  const location = useLocation();
  const [credentials, setCredentials] = useState({ username: "", password: "" });
  const [error, setError] = useState<string | null>(null);

  if (isLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-slate-950 text-slate-100">
        <span className="text-sm uppercase tracking-[0.4em] text-slate-500">Loading…</span>
      </div>
    );
  }

  if (!data?.has_users) {
    return <Navigate to="/register" replace />;
  }

  if (data?.is_authenticated) {
    const redirectTo = (location.state as { from?: string } | null)?.from ?? "/dashboard";
    return <Navigate to={redirectTo} replace />;
  }

  const handleSubmit = async (event: FormEvent) => {
    event.preventDefault();
    setError(null);
    try {
      await login.mutateAsync(credentials);
    } catch (err: any) {
      const detail = err?.response?.data?.detail;
      setError(typeof detail === "string" ? detail : "Incorrect username or password.");
    }
  };

  return (
    <div className="relative min-h-screen overflow-hidden bg-slate-950 text-slate-100">
      <div className="pointer-events-none absolute inset-0">
        <div className="absolute -top-40 right-1/3 h-72 w-72 rounded-full bg-indigo-500/20 blur-3xl" />
        <div className="absolute bottom-0 left-16 h-96 w-96 rounded-full bg-sky-500/10 blur-3xl" />
      </div>
      <div className="relative z-10 mx-auto flex min-h-screen w-full max-w-3xl flex-col justify-center px-6 py-16">
        <div className="mb-10 text-sm text-slate-400">
          <Link to="/" className="text-indigo-300 transition hover:text-indigo-200">← Back to overview</Link>
        </div>
        <h1 className="text-3xl font-semibold text-slate-50 md:text-4xl">Sign in to continue</h1>
        <p className="mt-3 max-w-xl text-base text-slate-400">
          Access the Altinet dashboard to view live presence, manage rooms and calibrate cameras.
        </p>
        <form onSubmit={handleSubmit} className="mt-10 grid gap-6 rounded-3xl border border-slate-800/60 bg-slate-900/60 p-10 backdrop-blur">
          <div className="grid gap-2">
            <label htmlFor="username" className="text-sm font-medium text-slate-200">
              Username
            </label>
            <input
              id="username"
              name="username"
              required
              value={credentials.username}
              onChange={(event) => setCredentials((prev) => ({ ...prev, username: event.target.value }))}
              className="rounded-xl border border-slate-700/60 bg-slate-950/60 px-4 py-3 text-slate-100 placeholder:text-slate-500 focus:border-indigo-400 focus:outline-none"
              placeholder="operator"
              autoComplete="username"
            />
          </div>
          <div className="grid gap-2">
            <label htmlFor="password" className="text-sm font-medium text-slate-200">
              Password
            </label>
            <input
              id="password"
              name="password"
              type="password"
              required
              value={credentials.password}
              onChange={(event) => setCredentials((prev) => ({ ...prev, password: event.target.value }))}
              className="rounded-xl border border-slate-700/60 bg-slate-950/60 px-4 py-3 text-slate-100 placeholder:text-slate-500 focus:border-indigo-400 focus:outline-none"
              placeholder="••••••••"
              autoComplete="current-password"
            />
          </div>
          {error && <p className="text-sm text-rose-300/80">{error}</p>}
          <button
            type="submit"
            disabled={login.isPending}
            className="mt-4 inline-flex items-center justify-center rounded-full bg-indigo-500 px-8 py-3 text-sm font-semibold uppercase tracking-wider text-white shadow-lg shadow-indigo-500/30 transition hover:shadow-indigo-500/50 disabled:cursor-not-allowed disabled:opacity-60"
          >
            {login.isPending ? "Signing in…" : "Sign in"}
          </button>
        </form>
      </div>
    </div>
  );
};

export default LoginPage;

