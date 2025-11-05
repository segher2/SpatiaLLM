/**
 * UOBB (Upright Oriented Bounding Box) computation using convex hull + rotating calipers.
 * This computes the minimum area bounding rectangle in XY plane, then combines with Z range.
 */

// Cross product for 2D vectors (returns z component)
function cross2d(a, b) {
  return a.x * b.y - a.y * b.x;
}

// Compute 2D distance
function dist2d(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}

// Andrew's monotone chain algorithm for 2D convex hull
function convexHull2D(points) {
  if (points.length < 3) return points;
  
  // Sort points lexicographically (first by x, then by y)
  const sorted = [...points].sort((a, b) => {
    return a.x !== b.x ? a.x - b.x : a.y - b.y;
  });
  
  // Remove duplicates
  const unique = [];
  for (let i = 0; i < sorted.length; i++) {
    if (i === 0 || dist2d(sorted[i], sorted[i - 1]) > 1e-6) {
      unique.push(sorted[i]);
    }
  }
  
  if (unique.length < 3) return unique;
  
  const hull = [];
  
  // Build lower hull
  for (const p of unique) {
    while (hull.length >= 2) {
      const v1 = {
        x: hull[hull.length - 1].x - hull[hull.length - 2].x,
        y: hull[hull.length - 1].y - hull[hull.length - 2].y
      };
      const v2 = {
        x: p.x - hull[hull.length - 1].x,
        y: p.y - hull[hull.length - 1].y
      };
      if (cross2d(v1, v2) <= 0) {
        hull.pop();
      } else {
        break;
      }
    }
    hull.push(p);
  }
  
  // Build upper hull
  const lowerSize = hull.length;
  for (let i = unique.length - 2; i >= 0; i--) {
    const p = unique[i];
    while (hull.length > lowerSize) {
      const v1 = {
        x: hull[hull.length - 1].x - hull[hull.length - 2].x,
        y: hull[hull.length - 1].y - hull[hull.length - 2].y
      };
      const v2 = {
        x: p.x - hull[hull.length - 1].x,
        y: p.y - hull[hull.length - 1].y
      };
      if (cross2d(v1, v2) <= 0) {
        hull.pop();
      } else {
        break;
      }
    }
    hull.push(p);
  }
  
  // Remove the last point (duplicate of first)
  hull.pop();
  
  return hull;
}

// Dot product for 2D vectors
function dot2d(a, b) {
  return a.x * b.x + a.y * b.y;
}

// Rotating calipers to find minimum area bounding rectangle
function rotatingCalipers(hull) {
  if (hull.length === 0) {
    return { theta: 0, min_u: 0, max_u: 0, min_v: 0, max_v: 0, area: 0 };
  }
  if (hull.length === 1) {
    return { theta: 0, min_u: hull[0].x, max_u: hull[0].x, min_v: hull[0].y, max_v: hull[0].y, area: 0 };
  }
  
  let best = {
    theta: 0,
    min_u: 0,
    max_u: 0,
    min_v: 0,
    max_v: 0,
    area: Infinity
  };
  
  const n = hull.length;
  
  // Try each edge direction
  for (let i = 0; i < n; i++) {
    const edge = {
      x: hull[(i + 1) % n].x - hull[i].x,
      y: hull[(i + 1) % n].y - hull[i].y
    };
    const edgeLen = Math.sqrt(edge.x * edge.x + edge.y * edge.y);
    if (edgeLen < 1e-9) continue;
    
    // Unit vectors for this edge direction
    const u = { x: edge.x / edgeLen, y: edge.y / edgeLen };
    const v = { x: -u.y, y: u.x };
    
    const theta = Math.atan2(u.y, u.x);
    
    // Project all hull points onto u and v
    let min_u = Infinity;
    let max_u = -Infinity;
    let min_v = Infinity;
    let max_v = -Infinity;
    
    for (const p of hull) {
      const proj_u = dot2d(u, p);
      const proj_v = dot2d(v, p);
      min_u = Math.min(min_u, proj_u);
      max_u = Math.max(max_u, proj_u);
      min_v = Math.min(min_v, proj_v);
      max_v = Math.max(max_v, proj_v);
    }
    
    const L = max_u - min_u;
    const W = max_v - min_v;
    const area = L * W;
    
    if (area < best.area) {
      best = { theta, min_u, max_u, min_v, max_v, area };
    }
  }
  
  return best;
}

/**
 * Compute UOBB from point cloud positions.
 * @param {Float32Array} positions - Flat array of [x, y, z, x, y, z, ...]
 * @returns {Object} UOBB with center, size, yaw, and corners
 */
export function computeUOBB(positions) {
  if (!positions || positions.length < 3) {
    return null;
  }
  
  const numPoints = positions.length / 3;
  
  // Project points to XY plane and find Z range
  const points2d = [];
  let zmin = Infinity;
  let zmax = -Infinity;
  
  for (let i = 0; i < numPoints; i++) {
    const x = positions[i * 3];
    const y = positions[i * 3 + 1];
    const z = positions[i * 3 + 2];
    points2d.push({ x, y });
    zmin = Math.min(zmin, z);
    zmax = Math.max(zmax, z);
  }
  
  // Compute 2D convex hull
  const hull = convexHull2D(points2d);
  
  // Find minimum area rectangle using rotating calipers
  const rect = rotatingCalipers(hull);
  
  // Compute center BEFORE swapping (use original rect values)
  const cx_2d = (rect.min_u + rect.max_u) * 0.5;
  const cy_2d = (rect.min_v + rect.max_v) * 0.5;
  
  // Ensure L >= W (length along e1 >= width along e2)
  let yaw = rect.theta;
  let lx = rect.max_u - rect.min_u;
  let ly = rect.max_v - rect.min_v;
  
  if (ly > lx) {
    // Swap L and W, rotate by 90 degrees
    [lx, ly] = [ly, lx];
    yaw += Math.PI / 2;
  }
  
  // Normalize yaw to [-pi, pi]
  while (yaw > Math.PI) yaw -= 2 * Math.PI;
  while (yaw < -Math.PI) yaw += 2 * Math.PI;
  
  // Compute rotation matrix for the ORIGINAL angle (rect.theta)
  const cos_theta = Math.cos(rect.theta);
  const sin_theta = Math.sin(rect.theta);
  
  // Transform center from rotated frame back to world frame
  const cx = cos_theta * cx_2d - sin_theta * cy_2d;
  const cy = sin_theta * cx_2d + cos_theta * cy_2d;
  
  // Compute final rotation matrix for yaw
  const cos_yaw = Math.cos(yaw);
  const sin_yaw = Math.sin(yaw);
  
  // Height from z-range
  const lz = zmax - zmin;
  const cz = (zmin + zmax) * 0.5;
  
  // Compute 8 corners
  const e1 = { x: cos_yaw, y: sin_yaw, z: 0 };
  const e2 = { x: -sin_yaw, y: cos_yaw, z: 0 };
  const ez = { x: 0, y: 0, z: 1 };
  
  const hx = lx * 0.5;
  const hy = ly * 0.5;
  const hz = lz * 0.5;
  
  const base = [
    { x: cx + e1.x * hx + e2.x * hy, y: cy + e1.y * hx + e2.y * hy, z: cz - hz },
    { x: cx - e1.x * hx + e2.x * hy, y: cy - e1.y * hx + e2.y * hy, z: cz - hz },
    { x: cx - e1.x * hx - e2.x * hy, y: cy - e1.y * hx - e2.y * hy, z: cz - hz },
    { x: cx + e1.x * hx - e2.x * hy, y: cy + e1.y * hx - e2.y * hy, z: cz - hz }
  ];
  
  const corners = [
    ...base,
    { x: base[0].x, y: base[0].y, z: base[0].z + lz },
    { x: base[1].x, y: base[1].y, z: base[1].z + lz },
    { x: base[2].x, y: base[2].y, z: base[2].z + lz },
    { x: base[3].x, y: base[3].y, z: base[3].z + lz }
  ];
  
  return {
    center: { x: cx, y: cy, z: cz },
    size: { x: lx, y: ly, z: lz },
    yaw,
    corners
  };
}

/**
 * Write UOBB as binary PLY mesh (8 vertices, 12 triangles)
 * @param {Object} uobb - UOBB object from computeUOBB
 * @returns {Buffer} Binary PLY data
 */
export function writeUOBBPly(uobb) {
  if (!uobb || !uobb.corners || uobb.corners.length !== 8) {
    throw new Error('Invalid UOBB object');
  }
  
  // Triangle faces (12 triangles)
  const faces = [
    [0, 1, 2], [0, 2, 3], // bottom
    [4, 5, 6], [4, 6, 7], // top
    [0, 1, 5], [0, 5, 4], // side 1
    [1, 2, 6], [1, 6, 5], // side 2
    [2, 3, 7], [2, 7, 6], // side 3
    [3, 0, 4], [3, 4, 7]  // side 4
  ];
  
  // Build header
  const header = [
    'ply',
    'format binary_little_endian 1.0',
    'element vertex 8',
    'property float x',
    'property float y',
    'property float z',
    'element face 12',
    'property list uchar int vertex_indices',
    'end_header\n'
  ].join('\n');
  
  // Calculate buffer size
  const headerBuffer = Buffer.from(header, 'utf-8');
  const vertexBytes = 8 * 3 * 4; // 8 vertices * 3 floats * 4 bytes
  const faceBytes = 12 * (1 + 3 * 4); // 12 faces * (1 byte count + 3 ints * 4 bytes)
  const totalSize = headerBuffer.length + vertexBytes + faceBytes;
  
  const buffer = Buffer.alloc(totalSize);
  let offset = 0;
  
  // Write header
  headerBuffer.copy(buffer, offset);
  offset += headerBuffer.length;
  
  // Write vertices
  for (const corner of uobb.corners) {
    buffer.writeFloatLE(corner.x, offset); offset += 4;
    buffer.writeFloatLE(corner.y, offset); offset += 4;
    buffer.writeFloatLE(corner.z, offset); offset += 4;
  }
  
  // Write faces
  for (const face of faces) {
    buffer.writeUInt8(3, offset); offset += 1; // vertex count
    buffer.writeInt32LE(face[0], offset); offset += 4;
    buffer.writeInt32LE(face[1], offset); offset += 4;
    buffer.writeInt32LE(face[2], offset); offset += 4;
  }
  
  return buffer;
}
