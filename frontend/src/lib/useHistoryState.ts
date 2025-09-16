import { useCallback, useState } from "react";

type HistoryState<T> = {
  history: T[];
  pointer: number;
};

export function useHistoryState<T>(initial: T) {
  const [state, setState] = useState<HistoryState<T>>({ history: [initial], pointer: 0 });

  const setValue = useCallback((value: T) => {
    setState((prev) => {
      const nextHistory = prev.history.slice(0, prev.pointer + 1);
      nextHistory.push(value);
      return { history: nextHistory, pointer: nextHistory.length - 1 };
    });
  }, []);

  const updateCurrent = useCallback((value: T) => {
    setState((prev) => {
      const nextHistory = [...prev.history];
      nextHistory[prev.pointer] = value;
      return { history: nextHistory, pointer: prev.pointer };
    });
  }, []);

  const undo = useCallback(() => {
    setState((prev) => ({
      history: prev.history,
      pointer: prev.pointer > 0 ? prev.pointer - 1 : prev.pointer
    }));
  }, []);

  const redo = useCallback(() => {
    setState((prev) => ({
      history: prev.history,
      pointer: prev.pointer < prev.history.length - 1 ? prev.pointer + 1 : prev.pointer
    }));
  }, []);

  const replace = useCallback((value: T) => {
    setState({ history: [value], pointer: 0 });
  }, []);

  return {
    value: state.history[state.pointer],
    setValue,
    updateCurrent,
    undo,
    redo,
    replace,
    canUndo: state.pointer > 0,
    canRedo: state.pointer < state.history.length - 1
  } as const;
}
