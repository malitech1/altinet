import React, { FormEvent, useState } from "react";
import { Link, Navigate } from "react-router-dom";

import { useAuthActions, useAuthStatus } from "../../lib/useAuth";

const RegisterPage: React.FC = () => {
  const { data, isLoading } = useAuthStatus();
  const { register } = useAuthActions();
  const [form, setForm] = useState({
    username: "",
    password: "",
    email: "",
    first_name: "",
    last_name: ""
  });
  const [error, setError] = useState<string | null>(null);

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

  const handleSubmit = async (event: FormEvent) => {
    event.preventDefault();
    setError(null);
    try {
      await register.mutateAsync({
        username: form.username,
        password: form.password,
        email: form.email || undefined,
        first_name: form.first_name || undefined,
        last_name: form.last_name || undefined
      });
    } catch (err: any) {
      const detail = err?.response?.data?.detail;
      setError(typeof detail === "string" ? detail : "Could not create your account. Please try again.");
    }
  };

  return (
    <div className="relative min-h-screen overflow-hidden bg-slate-950 text-slate-100">
      <div className="pointer-events-none absolute inset-0">
        <div className="absolute -top-56 left-24 h-80 w-80 rounded-full bg-indigo-500/20 blur-3xl" />
        <div className="absolute bottom-10 right-0 h-96 w-96 rounded-full bg-sky-500/10 blur-3xl" />
      </div>
      <div className="relative z-10 mx-auto flex min-h-screen w-full max-w-4xl flex-col justify-center px-6 py-16">
        <div className="mb-10 text-sm text-slate-400">
          <Link to="/" className="text-indigo-300 transition hover:text-indigo-200">← Back to overview</Link>
        </div>
        <h1 className="text-3xl font-semibold text-slate-50 md:text-4xl">Create the first operator</h1>
        <p className="mt-3 max-w-xl text-base text-slate-400">
          This account manages the entire perception stack. Choose a strong password and keep it secure.
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
              value={form.username}
              onChange={(event) => setForm((prev) => ({ ...prev, username: event.target.value }))}
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
              value={form.password}
              onChange={(event) => setForm((prev) => ({ ...prev, password: event.target.value }))}
              className="rounded-xl border border-slate-700/60 bg-slate-950/60 px-4 py-3 text-slate-100 placeholder:text-slate-500 focus:border-indigo-400 focus:outline-none"
              placeholder="••••••••"
              autoComplete="new-password"
            />
          </div>
          <div className="grid gap-2 sm:grid-cols-2 sm:gap-6">
            <div className="grid gap-2">
              <label htmlFor="first_name" className="text-sm font-medium text-slate-200">
                First name (optional)
              </label>
              <input
                id="first_name"
                name="first_name"
                value={form.first_name}
                onChange={(event) => setForm((prev) => ({ ...prev, first_name: event.target.value }))}
                className="rounded-xl border border-slate-700/60 bg-slate-950/60 px-4 py-3 text-slate-100 placeholder:text-slate-500 focus:border-indigo-400 focus:outline-none"
                placeholder="Alex"
                autoComplete="given-name"
              />
            </div>
            <div className="grid gap-2">
              <label htmlFor="last_name" className="text-sm font-medium text-slate-200">
                Last name (optional)
              </label>
              <input
                id="last_name"
                name="last_name"
                value={form.last_name}
                onChange={(event) => setForm((prev) => ({ ...prev, last_name: event.target.value }))}
                className="rounded-xl border border-slate-700/60 bg-slate-950/60 px-4 py-3 text-slate-100 placeholder:text-slate-500 focus:border-indigo-400 focus:outline-none"
                placeholder="Rivera"
                autoComplete="family-name"
              />
            </div>
          </div>
          <div className="grid gap-2">
            <label htmlFor="email" className="text-sm font-medium text-slate-200">
              Email (optional)
            </label>
            <input
              id="email"
              name="email"
              type="email"
              value={form.email}
              onChange={(event) => setForm((prev) => ({ ...prev, email: event.target.value }))}
              className="rounded-xl border border-slate-700/60 bg-slate-950/60 px-4 py-3 text-slate-100 placeholder:text-slate-500 focus:border-indigo-400 focus:outline-none"
              placeholder="ops@example.com"
              autoComplete="email"
            />
          </div>
          {error && <p className="text-sm text-rose-300/80">{error}</p>}
          <button
            type="submit"
            disabled={register.isPending}
            className="mt-4 inline-flex items-center justify-center rounded-full bg-indigo-500 px-8 py-3 text-sm font-semibold uppercase tracking-wider text-white shadow-lg shadow-indigo-500/30 transition hover:shadow-indigo-500/50 disabled:cursor-not-allowed disabled:opacity-60"
          >
            {register.isPending ? "Creating…" : "Create account"}
          </button>
        </form>
      </div>
    </div>
  );
};

export default RegisterPage;

