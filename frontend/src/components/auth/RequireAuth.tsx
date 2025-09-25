import React from "react";
import { Navigate, useLocation } from "react-router-dom";

import { useAuthStatus } from "../../lib/useAuth";

const RequireAuth: React.FC<{ children: React.ReactElement }> = ({ children }) => {
  const location = useLocation();
  const { data, isLoading } = useAuthStatus();

  if (isLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-slate-950 text-slate-200">
        <span className="text-sm uppercase tracking-[0.4em] text-slate-500">Loading…</span>
      </div>
    );
  }

  if (!data?.is_authenticated) {
    return <Navigate to="/login" replace state={{ from: location.pathname }} />;
  }

  return children;
};

export default RequireAuth;

