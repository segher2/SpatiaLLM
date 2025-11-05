import React, { useEffect, useMemo, useState } from 'react';
import PointCloudView from './viewer/PointCloudView2';
import { loadManifest } from './utils/manifest';
import type { UnifiedManifest } from './types';

function useQuery() {
  return useMemo(() => new URLSearchParams(window.location.search), []);
}

export default function App() {
  const query = useQuery();
  const [manifest, setManifest] = useState<UnifiedManifest | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Build manifest URL: if query param doesn't start with /, prepend /manifests/
  const manifestParam = query.get('manifest');
  const manifestUrl = manifestParam
    ? (manifestParam.startsWith('/') ? manifestParam : `/manifests/${manifestParam}`)
    : `/manifests/room_007.json`;

  useEffect(() => {
    loadManifest(manifestUrl)
      .then((m) => setManifest(m))
      .catch((e) => setError(String(e)));
  }, [manifestUrl]);

  // Generate smart title from manifest
  const title = useMemo(() => {
    if (!manifest) return manifestUrl;
    if (manifest.title) return manifest.title;
    
    // Generate title from manifest content
    const items = manifest.items || [];
    if (items.length === 0) return 'Empty Scene';
    
    // Count by type
    const typeCounts = new Map<string, number>();
    items.forEach(item => {
      if (item.type) typeCounts.set(item.type, (typeCounts.get(item.type) || 0) + 1);
    });
    
    // If only one item, use its name
    if (items.length === 1 && items[0].name) {
      return items[0].name;
    }
    
    // Build descriptive title from type counts
    if (typeCounts.size > 0) {
      const typeStrs = Array.from(typeCounts.entries())
        .sort(([, a], [, b]) => b - a) // Sort by count descending
        .slice(0, 3) // Top 3 types
        .map(([type, count]) => count > 1 ? `${count} ${type}s` : `${count} ${type}`);
      return typeStrs.join(', ');
    }
    
    // Fallback: just item count
    return `${items.length} item${items.length !== 1 ? 's' : ''}`;
  }, [manifest, manifestUrl]);

  return (
    <div style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <header style={{ padding: '8px 12px', borderBottom: '1px solid #eee' }}>
        <strong>Point Cloud Viewer</strong>
        <span style={{ marginLeft: 8, color: '#666' }}>{title}</span>
      </header>
      <div style={{ flex: 1, minHeight: 0 }}>
        {error && (
          <div style={{ padding: 12, color: '#b00' }}>Failed to load manifest: {error}</div>
        )}
        {manifest && (
          <PointCloudView manifest={manifest} />
        )}
      </div>
    </div>
  );
}
