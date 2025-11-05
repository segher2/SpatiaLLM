import React, { useEffect, useMemo, useState } from 'react';
import DeckGL from '@deck.gl/react';
import { OrbitView, COORDINATE_SYSTEM } from '@deck.gl/core';
import { PointCloudLayer } from '@deck.gl/layers';
import { SimpleMeshLayer } from '@deck.gl/mesh-layers';
import { loadPly } from '../loaders/ply';
import { loadMeshPly } from '../loaders/mesh';
import type { UnifiedManifest, ManifestItem, LoadedPointCloud } from '../types';
import { resolveCode } from '../utils/api';

type Props = { manifest: UnifiedManifest };

type LayerEntry = ManifestItem & { cloud?: LoadedPointCloud; mesh?: { positions: Float32Array; normals?: Float32Array; indices?: Uint16Array | Uint32Array } };

const INITIAL_VIEW_STATE = {
  target: [0, 0, 0],
  rotationOrbit: 0,
  rotationX: 30,
  zoom: 6
};

function updateBoundsFrom(positions: Float32Array, bounds: { min: number[]; max: number[] }) {
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1], z = positions[i + 2];
    if (x < bounds.min[0]) bounds.min[0] = x; if (x > bounds.max[0]) bounds.max[0] = x;
    if (y < bounds.min[1]) bounds.min[1] = y; if (y > bounds.max[1]) bounds.max[1] = y;
    if (z < bounds.min[2]) bounds.min[2] = z; if (z > bounds.max[2]) bounds.max[2] = z;
  }
}

function centerFromBounds(bounds: { min: number[]; max: number[] }): [number, number, number] {
  return [
    0.5 * (bounds.min[0] + bounds.max[0]),
    0.5 * (bounds.min[1] + bounds.max[1]),
    0.5 * (bounds.min[2] + bounds.max[2])
  ];
}

function sizeFromBounds(bounds: { min: number[]; max: number[] }): [number, number, number] {
  return [
    bounds.max[0] - bounds.min[0],
    bounds.max[1] - bounds.min[1],
    bounds.max[2] - bounds.min[2]
  ];
}

function buildBinaryAccessors(cloud: LoadedPointCloud, { includeColor = true }: { includeColor?: boolean } = {}) {
  const attrs: any = {
    getPosition: { value: cloud.attributes.positions, size: 3 }
  };
  if (includeColor && cloud.attributes.colors) {
    const isUint8 = cloud.attributes.colors instanceof Uint8Array;
    attrs.getColor = { value: cloud.attributes.colors, size: 3, normalized: isUint8 };
  }
  if (cloud.attributes.normals) attrs.getNormal = { value: cloud.attributes.normals, size: 3 };
  return attrs;
}

export default function PointCloudView({ manifest }: Props) {
  const [items, setItems] = useState<LayerEntry[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [center, setCenter] = useState<[number, number, number] | null>(null);
  const [sceneSize, setSceneSize] = useState<[number, number, number] | null>(null);
  const [viewState, setViewState] = useState<any>(INITIAL_VIEW_STATE);
  const [globalPointSize, setGlobalPointSize] = useState<number>(4);
  const [uobbOpacity, setUobbOpacity] = useState<number>(0.2); // Default 20%
  // Multi-select support: store array of picked items
  const [pickedItems, setPickedItems] = useState<Array<{ itemId: string; index: number; attrs: Record<string, any> }>>([]);
  const [selectedCloudIds, setSelectedCloudIds] = useState<Set<string>>(new Set()); // Track selected point clouds for highlighting
  const [sourceFileInfos, setSourceFileInfos] = useState<any[]>([]); // Store resolved source file information for all selected items
  const [loadingSourceFile, setLoadingSourceFile] = useState(false);
  const [sourceFileError, setSourceFileError] = useState<string | null>(null);
  const [layersError, setLayersError] = useState<string | null>(null);
  const [debugInfo, setDebugInfo] = useState<string>('');

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        setLoading(true);
        setError(null);
        const entries: LayerEntry[] = (manifest.items || []).map((it: ManifestItem) => ({ ...it }));
        for (let i = 0; i < entries.length; i++) {
          const it = entries[i];
          if (it.kind === 'pointcloud') {
            const include = it.filters?.labelInclude ? new Set<number>(it.filters.labelInclude) : undefined;
            const exclude = it.filters?.labelExclude ? new Set<number>(it.filters.labelExclude) : undefined;
            const dropColor = it.style?.colorMode === 'constant';
            const cloud = await loadPly(it.source.url, { filterLabelIn: include, filterLabelNotIn: exclude, dropColor });
            if (cancelled) return;
            entries[i] = { ...it, cloud };
          } else if (it.kind === 'mesh') {
            const mesh = await loadMeshPly(it.source.url);
            if (cancelled) return;
            entries[i] = { ...it, mesh };
          }
        }
        if (cancelled) return;

        const bounds = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
        let anyGeom = false;
        for (const e of entries) {
          if (e.cloud?.attributes?.positions) { updateBoundsFrom(e.cloud.attributes.positions, bounds); anyGeom = true; }
          else if (e.mesh?.positions) { updateBoundsFrom(e.mesh.positions, bounds); anyGeom = true; }
        }
        if (anyGeom) {
          const c = centerFromBounds(bounds);
          const sz = sizeFromBounds(bounds);
          console.log('[Viewer] Scene bounds:', { bounds, center: c, size: sz });
          
          // TEMP: DISABLE recentering to test if coordinate system is the issue
          const SKIP_RECENTER = true;
          if (!SKIP_RECENTER) {
            // CRITICAL: Recenter all point clouds to origin for CARTESIAN coordinate system
            for (const e of entries) {
              if (e.cloud?.attributes?.positions) {
                const pos = e.cloud.attributes.positions;
                for (let i = 0; i < pos.length; i += 3) {
                  pos[i] -= c[0];
                  pos[i + 1] -= c[1];
                  pos[i + 2] -= c[2];
                }
              }
              if (e.mesh?.positions) {
                const pos = e.mesh.positions;
                for (let i = 0; i < pos.length; i += 3) {
                  pos[i] -= c[0];
                  pos[i + 1] -= c[1];
                  pos[i + 2] -= c[2];
                }
              }
            }
            console.log('[Viewer] Recentered all geometries to origin');
          } else {
            console.log('[Viewer] SKIP_RECENTER=true, keeping original coordinates');
          }
          
          // Debug: verify coordinates
          const firstPts = entries[0]?.cloud?.attributes?.positions;
          const firstMesh = entries[0]?.mesh?.positions;
          if (firstPts) {
            console.log('[Viewer] First cloud point:', [firstPts[0], firstPts[1], firstPts[2]]);
          }
          if (firstMesh) {
            console.log('[Viewer] First mesh vertex:', [firstMesh[0], firstMesh[1], firstMesh[2]]);
            console.log('[Viewer] All mesh vertices:', Array.from(firstMesh));
          }
          
          setCenter(c);
          setSceneSize(sz);
          const maxDim = Math.max(sz[0], sz[1], sz[2]);
          // Calculate zoom: aim to fit scene in viewport
          // For CARTESIAN, zoom relates to camera distance from target
          const zoom = Math.max(0, Math.min(12, 8 - Math.log2(Math.max(1e-6, maxDim))));
          const target = SKIP_RECENTER ? [c[0], c[1], c[2]] as [number, number, number] : [0, 0, 0] as [number, number, number];
          console.log('[Viewer] Auto-fitting view to scene:', { 
            center: c, 
            size: sz, 
            maxDim, 
            target, 
            zoom,
            skipRecenter: SKIP_RECENTER
          });
          setViewState((vs: any) => ({ ...vs, target, zoom, rotationX: 45, rotationOrbit: 0 }));
        }

        // Debug: log loaded item stats
        try {
          const itemStats = entries.map(e => ({ id: e.id, kind: e.kind, pts: e.cloud?.length || 0, verts: e.mesh ? (e.mesh.positions.length/3)|0 : 0, tris: e.mesh?.indices ? ((e.mesh.indices.length/3)|0) : 0 }));
          // eslint-disable-next-line no-console
          console.log('[Viewer] items loaded:', itemStats);
          setDebugInfo(`Loaded ${entries.length} items: ${itemStats.filter(s => s.pts > 0 || s.verts > 0).length} with data`);
        } catch {}
        
        setItems(entries);
      } catch (e: any) {
        if (!cancelled) setError(String(e?.message || e));
      } finally {
        if (!cancelled) setLoading(false);
      }
    })();
    return () => { cancelled = true; };
  }, [manifest]);

  const layers = useMemo(() => {
    try {
      setLayersError(null);
      const list: any[] = [];
      let layerCount = 0;
      for (const entry of items) {
        if (entry.visible === false) continue;
        if (entry.kind === 'pointcloud') {
          const c = entry.cloud;
          if (!c) continue;
          if (!c.attributes?.positions || c.length === 0) {
            console.warn('[Layer] Skipping empty cloud:', entry.id);
            continue;
          }
          // Check if this cloud is selected for highlighting
          const isSelected = selectedCloudIds.has(entry.id);
          
          // If selected, use highlight color; otherwise use original color strategy
          const includeFileColor = !isSelected && entry.style?.colorMode !== 'constant';
          const constantColor = (entry.style?.color ?? [128, 128, 128]) as [number, number, number];
          const accessors = buildBinaryAccessors(c, { includeColor: includeFileColor });
          
          console.log('[Layer] Creating PointCloudLayer', entry.id, { 
            pts: c.length, 
            hasPos: Boolean(accessors.getPosition), 
            hasColor: Boolean(accessors.getColor),
            posSize: accessors.getPosition?.size,
            posValueLength: accessors.getPosition?.value?.length,
            isSelected
          });
          // Store entry metadata in a way that can be accessed during picking
          const layerData = {
            entry,
            cloud: c
          };
          
          // Highlight color: bright yellow-gold for selection
          const highlightColor = [255, 220, 0] as [number, number, number];
          
          // Build layer config
          const layerConfig: any = {
            id: entry.id,
            data: {
              length: c.length,
              attributes: accessors,
            },
            coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
            pointSize: isSelected ? (globalPointSize + 1) : globalPointSize || (entry.role === 'shell' ? 2 : 3),
            opacity: 1.0,
            pickable: true,
            autoHighlight: !isSelected, // Disable autoHighlight when already selected
            highlightColor: [255, 255, 0, 200],
            // Store metadata for picking
            _layerData: layerData
          };
          
          // When selected, override color with highlight color
          if (isSelected) {
            layerConfig.getColor = highlightColor;
          } else if (!includeFileColor) {
            layerConfig.getColor = constantColor;
          }
          
          list.push(new PointCloudLayer(layerConfig));
          layerCount++;
        } else if (entry.kind === 'mesh') {
          if (!entry.mesh) continue;
          const baseColor = entry.style?.color ?? [30, 144, 255];
          const alpha = Math.max(0, Math.min(1, uobbOpacity));
          const rgba = [...baseColor, Math.round(alpha * 255)];
          const attributes: any = { positions: { value: entry.mesh.positions, size: 3 } };
          if (entry.mesh.normals) attributes.normals = { value: entry.mesh.normals, size: 3 };
          const mesh = { attributes, indices: entry.mesh.indices, mode: 4 } as any; // 4 = GL.TRIANGLES
          console.log('[Layer] Creating SimpleMeshLayer', entry.id, { 
            verts: entry.mesh.positions.length/3, 
            tris: entry.mesh.indices ? entry.mesh.indices.length/3 : 0,
            color: rgba,
            opacity: alpha,
            firstVert: [entry.mesh.positions[0], entry.mesh.positions[1], entry.mesh.positions[2]],
            meshObj: mesh
          });
          list.push(new SimpleMeshLayer({
            id: entry.id,
            data: [{position: [0, 0, 0]}], // SimpleMeshLayer needs data points
            mesh,
            getPosition: (d: any) => d.position,
            getColor: rgba as any,
            opacity: alpha,
            pickable: false, // UOBB meshes are not selectable
            autoHighlight: false,
            coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
            wireframe: entry.style?.wireframe ?? false,
            // Fix rendering issues:
            // - depthTest: true - enables depth testing so meshes don't cover point clouds incorrectly
            // - blend: true - enables transparency blending
            // - cull: false - CRITICAL: disables backface culling so both sides of faces are visible
            // - depthMask: false - CRITICAL: don't write to depth buffer when transparent, allows points behind to show
            parameters: { 
              depthTest: true, 
              blend: true, 
              cull: false,
              depthMask: alpha >= 0.99 // only write depth if fully opaque
            },
            material: { ambient: 1.0, diffuse: 0.0, shininess: 0, specularColor: [0, 0, 0] }
          }));
          layerCount++;
        }
      }
      console.log('[Layers] Created', layerCount, 'layers from', items.length, 'items');
      console.log('[Layers] Returning list with', list.length, 'layer objects:', list.map(l => l.id));
      return list;
    } catch (e: any) {
      console.error('[Layers] Error creating layers:', e);
      setLayersError(String(e?.message || e));
      return [];
    }
  }, [items, globalPointSize, uobbOpacity, selectedCloudIds]);

  const view = useMemo(() => new OrbitView({ far: 100000 }), []);

  // Strategy: delay rendering layers until after first DeckGL mount
  const [readyLayers, setReadyLayers] = useState<any[]>([]);
  const [deckReady, setDeckReady] = useState(false);
  
  useEffect(() => {
    console.log('[Render] Layers computed, count:', layers.length, 'ids:', layers.map((l: any) => l.id));
    // Wait for DeckGL to be ready before passing non-empty layers
    if (layers.length > 0 && deckReady) {
      console.log('[Render] Passing', layers.length, 'layers to DeckGL (deckReady=true)');
      setReadyLayers(layers);
    } else if (layers.length === 0) {
      setReadyLayers([]);
    }
  }, [layers, deckReady]);

  const grouped = useMemo(() => {
    const g = new Map<string, LayerEntry[]>();
    for (const it of items) {
      const key = it.group || 'ungrouped';
      if (!g.has(key)) g.set(key, []);
      g.get(key)!.push(it);
    }
    return g;
  }, [items]);

  return (
    <div style={{ height: '100%', position: 'relative' }}>
      {loading && <div style={{ position: 'absolute', top: 8, left: 12, background: 'rgba(255,255,255,0.9)', padding: '4px 8px', borderRadius: 4, zIndex: 10 }}>Loading...</div>}
      {error && <div style={{ position: 'absolute', top: 8, left: 12, color: '#b00', background: 'rgba(255,255,255,0.9)', padding: '4px 8px', borderRadius: 4, zIndex: 10 }}>Error: {error}</div>}
      {layersError && <div style={{ position: 'absolute', top: 36, left: 12, color: '#b00', background: 'rgba(255,255,255,0.95)', padding: '4px 8px', border: '1px solid #f2caca', borderRadius: 4, zIndex: 10 }}>Layer error: {layersError}</div>}

      <DeckGL
        key={`deck-${items.length}`}
        views={view}
        viewState={viewState}
        controller={true}
        onViewStateChange={(e: any) => setViewState(e.viewState)}
        layers={readyLayers}
        style={{ position: 'absolute', inset: '0' }}
        onClick={(info: any, event: any) => {
          console.log('[DeckGL] onClick info:', {
            picked: info.picked,
            index: info.index,
            layerId: info.layer?.id,
            layerProps: info.layer?.props,
            object: info.object
          });
          
          // Accumulative selection mode: always add to selection unless clicking on already selected item
          if (info.picked && info.index >= 0 && info.layer) {
            const layerData = (info.layer.props as any)?._layerData;
            console.log('[DeckGL] layerData:', layerData);
            
            if (layerData) {
              const { entry, cloud } = layerData;
              
              // Extract object_code from filename or id
              // Expected formats: 
              // - Object: "0-7-3_chair_cluster", "chair (object_id: 0-7-3)"
              // - Room shell: "0-7-0_shell", "room_shell (room_id: 0-7)"
              let objectCode: string | null = null;
              const name = entry.name || entry.id;
              
              // Try patterns in order of specificity
              // 1. Match "object_id: X-X-X" or "room_id: X-X" in name
              const idMatch = name.match(/(?:object_id|room_id):\s*(\d+-\d+(?:-\d+)?)/);
              if (idMatch) {
                objectCode = idMatch[1];
              }
              // 2. Match "X-X-X_" at start (object code)
              else {
                const objMatch = name.match(/^(\d+-\d+-\d+)_/);
                if (objMatch) {
                  objectCode = objMatch[1];
                }
                // 3. Match "X-X-X_" after "cluster-" prefix
                else {
                  const clusterMatch = name.match(/cluster-(\d+-\d+-\d+)_/);
                  if (clusterMatch) {
                    objectCode = clusterMatch[1];
                  }
                  // 4. For shells: match "X-X-X_shell" format
                  else {
                    const shellMatch = name.match(/^(\d+-\d+)-\d+_shell/);
                    if (shellMatch) {
                      objectCode = shellMatch[1]; // room code like "0-7"
                    }
                    // 5. For shells in manifest: "shell-room_XXX" -> try to extract from ID
                    else if (entry.role === 'shell' && entry.id) {
                      const shellIdMatch = entry.id.match(/shell-(.+)/);
                      if (shellIdMatch) {
                        // Try to find room code from the manifest data or source URL
                        const urlMatch = entry.source?.url?.match(/(\d+-\d+)/);
                        if (urlMatch) {
                          objectCode = urlMatch[1];
                        }
                      }
                    }
                  }
                }
              }
              
              const attrs: Record<string, any> = {
                name: entry.name || entry.id,
                role: entry.role,
                sourceUrl: entry.source?.url,
              };
              if (objectCode) attrs.objectCode = objectCode;
              if (entry.group) attrs.group = entry.group;
              
              // Extract point_id and label if they exist
              if (cloud.attributes.point_ids && info.index < cloud.attributes.point_ids.length) {
                attrs.point_id = cloud.attributes.point_ids[info.index];
              }
              if (cloud.attributes.labels && info.index < cloud.attributes.labels.length) {
                attrs.label = cloud.attributes.labels[info.index];
              }
              
              const pickedItem = {
                itemId: entry.id,
                index: info.index,
                attrs
              };
              
              // Accumulative selection: toggle if already selected, otherwise add to selection
              const isAlreadySelected = selectedCloudIds.has(entry.id);
              if (isAlreadySelected) {
                // Deselect: remove from selection
                const newSelected = new Set(selectedCloudIds);
                newSelected.delete(entry.id);
                setSelectedCloudIds(newSelected);
                setPickedItems(pickedItems.filter(p => p.itemId !== entry.id));
                console.log('[DeckGL] Deselected cloud:', entry.id);
              } else {
                // Add to selection
                const newSelected = new Set(selectedCloudIds);
                newSelected.add(entry.id);
                setSelectedCloudIds(newSelected);
                setPickedItems([...pickedItems, pickedItem]);
                console.log('[DeckGL] Added cloud to selection:', entry.id);
              }
            } else {
              // Fallback if no layerData
              const pickedItem = {
                itemId: info.layer.id || 'unknown',
                index: info.index,
                attrs: { name: info.layer.id || 'unknown', role: 'unknown' }
              };
              
              const isAlreadySelected = selectedCloudIds.has(info.layer.id);
              if (isAlreadySelected) {
                const newSelected = new Set(selectedCloudIds);
                newSelected.delete(info.layer.id);
                setSelectedCloudIds(newSelected);
                setPickedItems(pickedItems.filter(p => p.itemId !== info.layer.id));
              } else {
                const newSelected = new Set(selectedCloudIds);
                newSelected.add(info.layer.id);
                setSelectedCloudIds(newSelected);
                setPickedItems([...pickedItems, pickedItem]);
              }
            }
          } else {
            // Clicking on empty space - do nothing (keep selection)
            console.log('[DeckGL] Clicked empty space - keeping current selection');
          }
        }}
        onWebGLInitialized={(gl: any) => {
          console.log('[DeckGL] WebGL initialized:', gl ? 'OK' : 'FAILED');
          setDeckReady(true);
        }}
        onLoad={() => console.log('[DeckGL] onLoad called, readyLayers:', readyLayers.length)}
        onError={(error: any) => console.error('[DeckGL] Error:', error)}
      />

      {/* Debug info overlay */}
      {debugInfo && (
        <div style={{ position: 'absolute', bottom: 8, left: 12, background: 'rgba(0,0,0,0.7)', color: '#fff', padding: '4px 8px', borderRadius: 4, zIndex: 10, fontSize: 11, fontFamily: 'monospace' }}>
          {debugInfo}
        </div>
      )}
      
      {/* Hint overlay when nothing is visible */}
      {(!loading && !error && items.length > 0 && items.every(e => (e.visible === false) || (e.kind === 'pointcloud' ? !e.cloud || e.cloud.length === 0 : !e.mesh || e.mesh.positions.length === 0))) && (
        <div style={{ position: 'absolute', top: 8, left: 12, background: 'rgba(255,255,200,0.95)', padding: '6px 10px', border: '1px solid #ddd', borderRadius: 4, zIndex: 10 }}>
          No visible geometry. Check manifest URLs and network responses.
        </div>
      )}

  <div style={{ position: 'absolute', right: 12, top: 12, width: 380, background: 'rgba(255,255,255,0.95)', border: '1px solid #ddd', borderRadius: 6, padding: 8 }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
          <div style={{ fontWeight: 600 }}>Layers</div>
          <button onClick={() => {
            // Calculate proper zoom based on scene size
            if (center && sceneSize) {
              const maxDim = Math.max(sceneSize[0], sceneSize[1], sceneSize[2]);
              const zoom = Math.max(2, Math.min(14, 8 - Math.log2(Math.max(1e-6, maxDim))));
              setViewState((vs: any) => ({ ...vs, target: center, zoom, rotationOrbit: 0, rotationX: 45 }));
            }
          }}>
            Reset View
          </button>
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'auto 1fr auto', alignItems: 'center', gap: 8, marginBottom: 8 }}>
          <div style={{ fontSize: 12, color: '#555' }}>Point Size</div>
          <input type="range" min={1} max={10} step={1} value={globalPointSize} onChange={(e) => setGlobalPointSize(Number(e.target.value))} />
          <div style={{ fontFamily: 'ui-monospace, Menlo, monospace', fontSize: 12, width: 28, textAlign: 'right' }}>{globalPointSize}</div>
          {items.some(i => i.kind === 'mesh') && (
            <>
              <div style={{ fontSize: 12, color: '#555' }}>UOBB Opacity</div>
              <input type="range" min={0} max={1} step={0.05} value={uobbOpacity} onChange={(e) => setUobbOpacity(Number(e.target.value))} />
              <div style={{ fontFamily: 'ui-monospace, Menlo, monospace', fontSize: 12, width: 28, textAlign: 'right' }}>{Math.round(uobbOpacity * 100)}%</div>
            </>
          )}
        </div>
        <div style={{ maxHeight: 220, overflow: 'auto', marginBottom: 8 }}>
          {[...grouped.entries()].map(([group, arr]) => (
            <div key={group} style={{ marginBottom: 6 }}>
              <div style={{ fontWeight: 600, fontSize: 12, color: '#555', margin: '4px 0' }}>{group}</div>
              {arr.map((it) => (
                <label key={it.id} style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 4 }}>
                  <input
                    type="checkbox"
                    checked={it.visible !== false}
                    onChange={(e) => {
                      const copy = items.map((x) => x.id === it.id ? { ...x, visible: e.target.checked } : x);
                      setItems(copy);
                    }}
                  />
                  <span style={{ whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{it.name}</span>
                  {it.style?.colorMode === 'constant' && (
                    <span style={{ fontSize: 11, color: '#888' }}>gray</span>
                  )}
                </label>
              ))}
            </div>
          ))}
        </div>
        <div style={{ fontWeight: 600, marginBottom: 6 }}>Stats</div>
        <div style={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace', fontSize: 12, marginBottom: 8 }}>
          <div>items loaded: {items.filter((x) => x.cloud || x.mesh).length} / {items.length}</div>
          <div>total points: {items.reduce((s, x) => s + (x.cloud?.length || 0), 0)}</div>
          {sceneSize && (<div>scene size: {sceneSize.map((v) => v.toFixed(2)).join(' × ')}</div>)}
          {items.some(i => i.kind === 'mesh') && (
            <div>mesh tris: {items.reduce((s, x) => s + (x.mesh?.indices ? (x.mesh.indices.length/3)|0 : 0), 0)}</div>
          )}
        </div>
        <div style={{ fontWeight: 600, marginBottom: 6 }}>Inspector</div>
        {pickedItems.length === 0 && (
          <div style={{ color: '#666', fontSize: 12 }}>
            Click objects to add to selection. Click selected object again to remove.
          </div>
        )}
        {pickedItems.length > 0 && (
          <div style={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace', fontSize: 12 }}>
            <div style={{ marginBottom: 4, paddingBottom: 4, borderBottom: '1px solid #ddd', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <strong>Selected ({pickedItems.length})</strong>
              <button 
                onClick={() => {
                  setSelectedCloudIds(new Set());
                  setPickedItems([]);
                  setSourceFileInfos([]);
                  setSourceFileError(null);
                }}
                style={{ 
                  padding: '2px 6px', 
                  fontSize: 10, 
                  cursor: 'pointer',
                  background: '#f0f0f0',
                  color: '#666',
                  border: '1px solid #ccc',
                  borderRadius: 3
                }}
              >
                Clear All
              </button>
            </div>
            
            {/* List of selected items */}
            <div style={{ maxHeight: 200, overflowY: 'auto', marginBottom: 8 }}>
              {pickedItems.map((picked, idx) => (
                <div key={picked.itemId} style={{ 
                  padding: '6px 8px', 
                  marginBottom: 4, 
                  background: '#f8f8f8', 
                  borderRadius: 4,
                  border: '1px solid #e0e0e0'
                }}>
                  <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'start', marginBottom: 4 }}>
                    <div style={{ fontWeight: 600, fontSize: 11, flex: 1 }}>
                      {picked.attrs.name ?? picked.itemId}
                    </div>
                    <button 
                      onClick={() => {
                        const newSelected = new Set(selectedCloudIds);
                        newSelected.delete(picked.itemId);
                        setSelectedCloudIds(newSelected);
                        setPickedItems(pickedItems.filter(p => p.itemId !== picked.itemId));
                        setSourceFileInfos(sourceFileInfos.filter((_, i) => i !== idx));
                      }}
                      style={{ 
                        padding: '1px 4px', 
                        fontSize: 9, 
                        cursor: 'pointer',
                        background: '#fff',
                        color: '#999',
                        border: '1px solid #ccc',
                        borderRadius: 2,
                        marginLeft: 4
                      }}
                      title="Remove from selection"
                    >
                      ✕
                    </button>
                  </div>
                  {picked.attrs.objectCode && (
                    <div style={{ color: '#0066cc', fontWeight: 600, fontSize: 11 }}>
                      {picked.attrs.objectCode}
                    </div>
                  )}
                  {picked.attrs.role && <div style={{ fontSize: 10, color: '#666' }}>{picked.attrs.role}</div>}
                </div>
              ))}
            </div>
            
            {/* Batch Actions */}
            <div style={{ marginTop: 12, paddingTop: 8, borderTop: '1px solid #eee' }}>
              <button 
                onClick={async () => {
                  // Check if all selected items have object codes
                  const itemsWithCodes = pickedItems.filter(p => p.attrs.objectCode);
                  if (itemsWithCodes.length === 0) {
                    alert('No valid objects selected. Please select objects with valid codes.');
                    return;
                  }
                  
                  setLoadingSourceFile(true);
                  setSourceFileError(null);
                  setSourceFileInfos([]);
                  
                  try {
                    // Resolve all object codes in parallel
                    const results = await Promise.all(
                      itemsWithCodes.map(async (picked) => {
                        try {
                          const info = await resolveCode(picked.attrs.objectCode);
                          return {
                            timestamp: new Date().toISOString(),
                            selection: {
                              code: picked.attrs.objectCode,
                              type: picked.attrs.role,
                              name: picked.attrs.name,
                              viewer_url: picked.attrs.sourceUrl,
                              clicked_point: {
                                index: picked.index,
                                point_id: picked.attrs.point_id,
                                label: picked.attrs.label
                              }
                            },
                            source_files: info
                          };
                        } catch (err: any) {
                          return {
                            error: true,
                            code: picked.attrs.objectCode,
                            message: err.message || String(err)
                          };
                        }
                      })
                    );
                    
                    // Separate successful and failed results
                    const successful = results.filter(r => !(r as any).error);
                    const failed = results.filter(r => (r as any).error);
                    
                    setSourceFileInfos(successful);
                    
                    // Output to console for debugging
                    console.log('=== Multi-Object Confirmation Results ===');
                    console.log(JSON.stringify({ 
                      total: itemsWithCodes.length,
                      successful: successful.length,
                      failed: failed.length,
                      results: successful 
                    }, null, 2));
                    console.log('==========================================');
                    
                    // Submit selection to backend (real-time notification)
                    if (successful.length > 0) {
                      try {
                        const selectionData = successful.map((s: any) => ({
                          itemCode: s.selection?.code || 'unknown',
                          displayName: s.selection?.name || 'unknown',
                          type: s.selection?.type || 'unknown',
                          sourceFile: s.source_files?.mesh_path || s.source_files?.cluster_path || '',
                          timestamp: s.timestamp
                        }));
                        
                        const response = await fetch('http://localhost:8090/api/submit-selection', {
                          method: 'POST',
                          headers: { 'Content-Type': 'application/json' },
                          body: JSON.stringify(selectionData)
                        });
                        
                        if (!response.ok) {
                          console.warn('Failed to submit selection to backend:', response.statusText);
                        } else {
                          console.log('✓ Selection submitted to backend successfully');
                        }
                      } catch (submitErr) {
                        console.warn('Could not submit selection to backend:', submitErr);
                      }
                    }
                    
                    if (failed.length > 0) {
                      console.error('Failed confirmations:', failed);
                      alert(`Warning: ${failed.length} object(s) failed to confirm:\n${failed.map((f: any) => f.code).join(', ')}`);
                    } else {
                      alert(`Successfully confirmed ${successful.length} object(s)!`);
                    }
                    
                  } catch (err: any) {
                    const errorMsg = err.message || String(err);
                    setSourceFileError(errorMsg);
                    console.error('[Multi-Object Confirmation Failed]', err);
                    alert(`Batch confirmation failed: ${errorMsg}\n\nPlease ensure the API server is running (port 8090)`);
                  } finally {
                    setLoadingSourceFile(false);
                  }
                }}
                disabled={pickedItems.filter(p => p.attrs.objectCode).length === 0 || loadingSourceFile}
                style={{ 
                  width: '100%',
                  padding: '8px 12px', 
                  fontSize: 12, 
                  fontWeight: 600,
                  cursor: pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile ? 'pointer' : 'not-allowed',
                  background: pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile ? '#28a745' : '#ccc',
                  color: '#fff',
                  border: 'none',
                  borderRadius: 4,
                  marginBottom: 6,
                  transition: 'background 0.2s'
                }}
                onMouseEnter={(e) => {
                  if (pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile) {
                    e.currentTarget.style.background = '#218838';
                  }
                }}
                onMouseLeave={(e) => {
                  if (pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile) {
                    e.currentTarget.style.background = '#28a745';
                  }
                }}
              >
                {loadingSourceFile ? '⏳ Confirming...' : `✓ Confirm All (${pickedItems.filter(p => p.attrs.objectCode).length})`}
              </button>
              
              <button 
                onClick={async () => {
                  // Use confirmed source file infos if available, otherwise fetch them
                  let infosToDownload = sourceFileInfos;
                  
                  if (infosToDownload.length === 0) {
                    // Need to fetch first
                    const itemsWithCodes = pickedItems.filter(p => p.attrs.objectCode);
                    if (itemsWithCodes.length === 0) {
                      alert('No valid objects to download');
                      return;
                    }
                    
                    setLoadingSourceFile(true);
                    setSourceFileError(null);
                    
                    try {
                      const results = await Promise.all(
                        itemsWithCodes.map(async (picked) => {
                          try {
                            const info = await resolveCode(picked.attrs.objectCode);
                            return {
                              selection: {
                                code: picked.attrs.objectCode,
                                name: picked.attrs.name
                              },
                              source_files: info
                            };
                          } catch (err: any) {
                            return null;
                          }
                        })
                      );
                      
                      infosToDownload = results.filter(r => r !== null) as any[];
                    } catch (err: any) {
                      alert(`Failed to fetch file info: ${err.message || String(err)}`);
                      setLoadingSourceFile(false);
                      return;
                    }
                    setLoadingSourceFile(false);
                  }
                  
                  // Download all files
                  if (infosToDownload.length === 0) {
                    alert('No valid files to download');
                    return;
                  }
                  
                  for (const info of infosToDownload) {
                    let filePath: string | undefined;
                    if ('cluster_path' in info.source_files) {
                      filePath = info.source_files.cluster_path;
                    } else if ('shell_path' in info.source_files) {
                      filePath = info.source_files.shell_path;
                    }
                    
                    if (filePath) {
                      const filename = filePath.split('/').pop() || 'object.ply';
                      const downloadUrl = `http://localhost:8090/api/download-file?path=${encodeURIComponent(filePath)}`;
                      
                      // Trigger download (stagger to avoid browser blocking)
                      const a = document.createElement('a');
                      a.href = downloadUrl;
                      a.download = filename;
                      a.click();
                      
                      // Small delay between downloads
                      await new Promise(resolve => setTimeout(resolve, 200));
                    }
                  }
                  
                  alert(`Downloading ${infosToDownload.length} file(s)`);
                }}
                disabled={pickedItems.filter(p => p.attrs.objectCode).length === 0 || loadingSourceFile}
                style={{ 
                  width: '100%',
                  padding: '8px 12px', 
                  fontSize: 12, 
                  fontWeight: 600,
                  cursor: pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile ? 'pointer' : 'not-allowed',
                  background: pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile ? '#2196f3' : '#ccc',
                  color: '#fff',
                  border: 'none',
                  borderRadius: 4,
                  marginBottom: 6,
                  transition: 'background 0.2s'
                }}
                onMouseEnter={(e) => {
                  if (pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile) {
                    e.currentTarget.style.background = '#1976d2';
                  }
                }}
                onMouseLeave={(e) => {
                  if (pickedItems.filter(p => p.attrs.objectCode).length > 0 && !loadingSourceFile) {
                    e.currentTarget.style.background = '#2196f3';
                  }
                }}
                title="Download all selected objects as PLY files"
              >
                {loadingSourceFile ? '⏳ Downloading...' : `💾 Download All (${pickedItems.filter(p => p.attrs.objectCode).length})`}
              </button>
            </div>
            
            {/* Error Display */}
            {sourceFileError && (
              <div style={{ 
                marginTop: 8, 
                padding: 8, 
                background: '#fee', 
                border: '1px solid #fcc', 
                borderRadius: 4,
                fontSize: 11,
                color: '#c00'
              }}>
                <strong>Error:</strong> {sourceFileError}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
 