/**
 * API utilities for communicating with the backend (ai_api.py)
 */

export interface ObjectInfo {
  object_code: string;
  class: string;
  cluster_path: string;
  uobb_path?: string;
  mesh_path?: string;
  csv_data?: Record<string, any>;
}

export interface RoomInfo {
  room_code: string;
  csv_path: string;
  shell_path: string;
  shell_uobb_path?: string;
  clusters_dir?: string;
}

export interface ResolveResponse {
  success: boolean;
  data?: ObjectInfo | RoomInfo;
  error?: string;
}

/**
 * Base URL for the backend API (can be configured via environment variable)
 */
const API_BASE_URL = (import.meta as any).env?.VITE_API_URL || 'http://localhost:8090';

/**
 * Resolve object information by object_code (e.g., "0-7-3")
 */
export async function resolveObject(objectCode: string): Promise<ObjectInfo> {
  const response = await fetch(`${API_BASE_URL}/api/resolve-object?code=${encodeURIComponent(objectCode)}`);
  if (!response.ok) {
    throw new Error(`Failed to resolve object: ${response.statusText}`);
  }
  const result: ResolveResponse = await response.json();
  if (!result.success || !result.data) {
    throw new Error(result.error || 'Unknown error');
  }
  return result.data as ObjectInfo;
}

/**
 * Resolve room information by room_code (e.g., "0-7")
 */
export async function resolveRoom(roomCode: string): Promise<RoomInfo> {
  const response = await fetch(`${API_BASE_URL}/api/resolve-room?code=${encodeURIComponent(roomCode)}`);
  if (!response.ok) {
    throw new Error(`Failed to resolve room: ${response.statusText}`);
  }
  const result: ResolveResponse = await response.json();
  if (!result.success || !result.data) {
    throw new Error(result.error || 'Unknown error');
  }
  return result.data as RoomInfo;
}

/**
 * Generic resolve function that handles both objects and rooms
 */
export async function resolveCode(code: string): Promise<ObjectInfo | RoomInfo> {
  // Determine if it's an object code (3 parts) or room code (2 parts)
  const parts = code.split('-');
  if (parts.length === 3) {
    return resolveObject(code);
  } else if (parts.length === 2) {
    return resolveRoom(code);
  } else {
    throw new Error(`Invalid code format: ${code}`);
  }
}
