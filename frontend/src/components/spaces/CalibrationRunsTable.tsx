import React from "react";
import { CameraCalibrationRun } from "../../lib/api";

interface Props {
  runs: CameraCalibrationRun[];
}

const CalibrationRunsTable: React.FC<Props> = ({ runs }) => (
  <div className="overflow-x-auto rounded border border-slate-800">
    <table className="min-w-full divide-y divide-slate-800 text-sm">
      <thead className="bg-slate-900 text-xs uppercase tracking-wide text-slate-400">
        <tr>
          <th className="px-3 py-2 text-left">Method</th>
          <th className="px-3 py-2 text-left">Status</th>
          <th className="px-3 py-2 text-left">Started</th>
          <th className="px-3 py-2 text-left">Finished</th>
          <th className="px-3 py-2 text-left">RMS</th>
        </tr>
      </thead>
      <tbody className="divide-y divide-slate-900">
        {runs.map((run) => (
          <tr key={run.id}>
            <td className="px-3 py-2 text-slate-200">{run.method}</td>
            <td className="px-3 py-2 text-slate-300">{run.status}</td>
            <td className="px-3 py-2 text-slate-400">{run.started_at ?? "–"}</td>
            <td className="px-3 py-2 text-slate-400">{run.finished_at ?? "–"}</td>
            <td className="px-3 py-2 text-slate-300">{run.error_rms?.toFixed(3) ?? "–"}</td>
          </tr>
        ))}
        {runs.length === 0 && (
          <tr>
            <td colSpan={5} className="px-3 py-6 text-center text-xs text-slate-500">
              No calibration runs yet
            </td>
          </tr>
        )}
      </tbody>
    </table>
  </div>
);

export default CalibrationRunsTable;
