import type { UnifiedManifest } from '../types';

export async function loadManifest(url: string): Promise<UnifiedManifest> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status} for ${url}`);
  const json = (await res.json()) as UnifiedManifest;
  return json;
}
