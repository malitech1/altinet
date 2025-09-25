import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";

import {
  AuthStatus,
  fetchAuthStatus,
  loginUser,
  logoutUser,
  registerUser
} from "./api";

export const authQueryKey = ["auth", "status"] as const;

export function useAuthStatus() {
  return useQuery({ queryKey: authQueryKey, queryFn: fetchAuthStatus, staleTime: 30_000 });
}

export function useAuthActions() {
  const queryClient = useQueryClient();

  const register = useMutation({
    mutationFn: registerUser,
    onSuccess: (user) => {
      const next: AuthStatus = {
        has_users: true,
        is_authenticated: true,
        user
      };
      queryClient.setQueryData(authQueryKey, next);
    }
  });

  const login = useMutation({
    mutationFn: loginUser,
    onSuccess: (user) => {
      const next: AuthStatus = {
        has_users: true,
        is_authenticated: true,
        user
      };
      queryClient.setQueryData(authQueryKey, next);
    }
  });

  const logout = useMutation({
    mutationFn: logoutUser,
    onSuccess: () => {
      queryClient.setQueryData<AuthStatus | undefined>(authQueryKey, (prev) => ({
        has_users: prev?.has_users ?? true,
        is_authenticated: false,
        user: null
      }));
    }
  });

  return { register, login, logout };
}

