import React from "react";
import ReactDOM from "react-dom/client";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Navigate, Route, Routes } from "react-router-dom";

import "./index.css";
import SpacesPage from "./pages/spaces";

const queryClient = new QueryClient();

const App = () => (
  <BrowserRouter>
    <Routes>
      <Route path="/spaces" element={<SpacesPage />} />
      <Route path="*" element={<Navigate to="/spaces" replace />} />
    </Routes>
  </BrowserRouter>
);

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <QueryClientProvider client={queryClient}>
      <App />
    </QueryClientProvider>
  </React.StrictMode>
);
