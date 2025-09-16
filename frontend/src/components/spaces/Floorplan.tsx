import React, { useCallback, useMemo, useRef, useState } from "react";

export type FloorplanPoint = { x_mm: number; y_mm: number };

export type FloorplanCamera = {
  id: string;
  name: string;
  position_mm: { x_mm: number; y_mm: number; z_mm: number };
  yaw_deg: number;
  fov_deg: number;
  metadata?: Record<string, any>;
};

interface FloorplanProps {
  width?: number;
  height?: number;
  polygon: FloorplanPoint[];
  onPolygonChange?: (points: FloorplanPoint[]) => void;
  addPointMode?: boolean;
  cameras?: FloorplanCamera[];
  onCameraChange?: (id: string, position: { x_mm: number; y_mm: number }) => void;
  onSelectCamera?: (id: string) => void;
  selectedCameraId?: string | null;
  showMetres?: boolean;
}

const GRID_SPACING_MM = 500;

function ensureClosed(points: FloorplanPoint[]): FloorplanPoint[] {
  if (points.length === 0) return points;
  const first = points[0];
  const last = points[points.length - 1];
  if (first.x_mm === last.x_mm && first.y_mm === last.y_mm) {
    return points;
  }
  return [...points, first];
}

export const Floorplan: React.FC<FloorplanProps> = ({
  width = 800,
  height = 520,
  polygon,
  onPolygonChange,
  addPointMode,
  cameras = [],
  onCameraChange,
  onSelectCamera,
  selectedCameraId,
  showMetres = false
}) => {
  const [scale, setScale] = useState(0.08);
  const [offset, setOffset] = useState({ x: width / 2, y: height / 2 });
  const draggingVertex = useRef<number | null>(null);
  const draggingCamera = useRef<string | null>(null);
  const panning = useRef(false);
  const panStart = useRef<{ x: number; y: number }>({ x: 0, y: 0 });
  const offsetStart = useRef({ x: offset.x, y: offset.y });

  const closedPolygon = useMemo(() => ensureClosed(polygon), [polygon]);

  const handleWheel = useCallback((event: React.WheelEvent<SVGSVGElement>) => {
    event.preventDefault();
    setScale((prev) => {
      const delta = event.deltaY > 0 ? 0.9 : 1.1;
      return Math.min(0.4, Math.max(0.02, prev * delta));
    });
  }, []);

  const svgToMm = useCallback(
    (event: React.PointerEvent<SVGSVGElement>) => {
      const rect = (event.currentTarget as SVGSVGElement).getBoundingClientRect();
      const svgX = event.clientX - rect.left;
      const svgY = event.clientY - rect.top;
      const x_mm = (svgX - offset.x) / scale;
      const y_mm = -(svgY - offset.y) / scale;
      return { x_mm, y_mm };
    },
    [offset.x, offset.y, scale]
  );

  const handlePointerDown = useCallback(
    (event: React.PointerEvent<SVGSVGElement>) => {
      if (event.button === 1 || event.shiftKey) {
        panning.current = true;
        panStart.current = { x: event.clientX, y: event.clientY };
        offsetStart.current = { ...offset };
        return;
      }
      if (addPointMode && onPolygonChange) {
        const { x_mm, y_mm } = svgToMm(event);
        onPolygonChange([...polygon, { x_mm: Math.round(x_mm), y_mm: Math.round(y_mm) }]);
      }
    },
    [addPointMode, onPolygonChange, offset, polygon, svgToMm]
  );

  const handlePointerMove = useCallback(
    (event: React.PointerEvent<SVGSVGElement>) => {
      if (panning.current) {
        setOffset({
          x: offsetStart.current.x + (event.clientX - panStart.current.x),
          y: offsetStart.current.y + (event.clientY - panStart.current.y)
        });
        return;
      }
      if (draggingVertex.current !== null && onPolygonChange) {
        const { x_mm, y_mm } = svgToMm(event);
        const updated = polygon.map((pt, idx) =>
          idx === draggingVertex.current ? { x_mm: Math.round(x_mm), y_mm: Math.round(y_mm) } : pt
        );
        onPolygonChange(updated);
      } else if (draggingCamera.current && onCameraChange) {
        const { x_mm, y_mm } = svgToMm(event);
        onCameraChange(draggingCamera.current, {
          x_mm: Math.round(x_mm),
          y_mm: Math.round(y_mm)
        });
      }
    },
    [onPolygonChange, onCameraChange, polygon, svgToMm]
  );

  const handlePointerUp = useCallback(() => {
    draggingVertex.current = null;
    draggingCamera.current = null;
    panning.current = false;
  }, []);

  const renderGrid = () => {
    const lines: React.ReactNode[] = [];
    const range = 20000; // +/-10m grid
    for (let mm = -range; mm <= range; mm += GRID_SPACING_MM) {
      const x = offset.x + mm * scale;
      const y = offset.y + mm * scale;
      lines.push(
        <line key={`v-${mm}`} x1={x} y1={0} x2={x} y2={height} stroke="#1e293b" strokeWidth={0.5} />
      );
      lines.push(
        <line key={`h-${mm}`} x1={0} y1={y} x2={width} y2={y} stroke="#1e293b" strokeWidth={0.5} />
      );
    }
    return lines;
  };

  const renderPolygon = () => {
    if (closedPolygon.length === 0) return null;
    const path = closedPolygon
      .map((point, index) => {
        const x = offset.x + point.x_mm * scale;
        const y = offset.y - point.y_mm * scale;
        return `${index === 0 ? "M" : "L"}${x},${y}`;
      })
      .join(" ");
    return (
      <>
        <path d={`${path} Z`} fill="rgba(59,130,246,0.1)" stroke="#3b82f6" strokeWidth={2} />
        {polygon.map((point, idx) => {
          const x = offset.x + point.x_mm * scale;
          const y = offset.y - point.y_mm * scale;
          return (
            <circle
              key={`vertex-${idx}`}
              cx={x}
              cy={y}
              r={7}
              fill="#38bdf8"
              className="cursor-move"
              onPointerDown={(event) => {
                draggingVertex.current = idx;
                (event.target as Element).setPointerCapture(event.pointerId);
              }}
            />
          );
        })}
      </>
    );
  };

  const renderCameras = () =>
    cameras.map((camera) => {
      const x = offset.x + camera.position_mm.x_mm * scale;
      const y = offset.y - camera.position_mm.y_mm * scale;
      const yawRad = (camera.yaw_deg * Math.PI) / 180;
      const range = (camera.metadata?.range_mm as number | undefined) ?? 8000;
      const fovHalf = (camera.fov_deg / 2) * (Math.PI / 180);
      const leftX = camera.position_mm.x_mm + Math.cos(yawRad - fovHalf) * range;
      const leftY = camera.position_mm.y_mm + Math.sin(yawRad - fovHalf) * range;
      const rightX = camera.position_mm.x_mm + Math.cos(yawRad + fovHalf) * range;
      const rightY = camera.position_mm.y_mm + Math.sin(yawRad + fovHalf) * range;
      const path = [
        `${offset.x + camera.position_mm.x_mm * scale},${offset.y - camera.position_mm.y_mm * scale}`,
        `${offset.x + leftX * scale},${offset.y - leftY * scale}`,
        `${offset.x + rightX * scale},${offset.y - rightY * scale}`
      ].join(" ");
      return (
        <g key={camera.id}>
          <polygon points={path} fill="rgba(251, 191, 36, 0.15)" stroke="#fbbf24" strokeWidth={1.5} />
          <circle
            cx={x}
            cy={y}
            r={10}
            fill={selectedCameraId === camera.id ? "#f97316" : "#facc15"}
            stroke="#1f2937"
            strokeWidth={2}
            onPointerDown={(event) => {
              draggingCamera.current = camera.id;
              (event.target as Element).setPointerCapture(event.pointerId);
              onSelectCamera?.(camera.id);
            }}
          />
          <text x={x + 12} y={y - 12} className="fill-slate-300 text-xs">
            {camera.name}
          </text>
        </g>
      );
    });

  return (
    <svg
      width={width}
      height={height}
      className="bg-slate-900 rounded-lg shadow-inner"
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerLeave={handlePointerUp}
      onWheel={handleWheel}
    >
      <rect x={0} y={0} width={width} height={height} fill="#0f172a" />
      {renderGrid()}
      <line x1={0} y1={offset.y} x2={width} y2={offset.y} stroke="#64748b" strokeWidth={1} />
      <line x1={offset.x} y1={0} x2={offset.x} y2={height} stroke="#64748b" strokeWidth={1} />
      {renderPolygon()}
      {renderCameras()}
      <text x={16} y={24} className="fill-slate-400 text-xs">
        Scale: {showMetres ? `${(1 / scale / 1000).toFixed(2)} m per px` : `${(1 / scale).toFixed(0)} mm per px`}
      </text>
    </svg>
  );
};

export default Floorplan;
