import { defineConfig } from "vite";

const createReactPlugin = async () => {
  try {
    const plugin = (await import("@vitejs/plugin-react")).default;
    return plugin();
  } catch (error) {
    console.warn(
      "@vitejs/plugin-react is unavailable; falling back to a minimal JSX transform."
    );

    return {
      name: "fallback-react-jsx-transform",
      config: () => ({
        esbuild: {
          jsx: "automatic",
          jsxImportSource: "react",
        },
      }),
    };
  }
};

export default defineConfig(async ({ command }) => ({
  base: command === "serve" ? "/" : "/static/",
  plugins: [await createReactPlugin()],
  server: {
    port: 5173,
    host: "0.0.0.0",
  },
}));
