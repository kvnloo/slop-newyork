// Lightweight XZ collision helpers. Headings rotate local +Z toward world +X.
// Normals point from body B toward body A, matching the existing impact audio events.

export function footprintContact(a, b) {
  const sa = Math.sin(a.yaw), ca = Math.cos(a.yaw);
  const sb = Math.sin(b.yaw), cb = Math.cos(b.yaw);
  const dx = a.x - b.x, dz = a.z - b.z;
  let depth = Infinity, nx = 0, nz = 0;

  // Separating-axis test for two rotated rectangles. AABBs alone incorrectly
  // collide with the empty corners of diagonal Manhattan building footprints.
  for (let i = 0; i < 4; i++) {
    const ax = i === 0 ? ca : i === 1 ? sa : i === 2 ? cb : sb;
    const az = i === 0 ? -sa : i === 1 ? ca : i === 2 ? -sb : cb;
    const distance = dx * ax + dz * az;
    const ra = a.halfWidth * Math.abs(ca * ax - sa * az)
      + a.halfLength * Math.abs(sa * ax + ca * az);
    const rb = b.halfWidth * Math.abs(cb * ax - sb * az)
      + b.halfLength * Math.abs(sb * ax + cb * az);
    const penetration = ra + rb - Math.abs(distance);
    if (penetration <= 0) return null;
    if (penetration < depth) {
      depth = penetration;
      const sign = distance < 0 ? -1 : 1;
      nx = ax * sign;
      nz = az * sign;
    }
  }
  return { nx, nz, depth };
}

export function buildingContact(body, collider) {
  const o = collider.obb;
  const building = o
    ? { x: o.cx, z: o.cz, yaw: Math.atan2(o.sn, o.cs), halfWidth: o.hx, halfLength: o.hz }
    : {
      x: (collider.minX + collider.maxX) * 0.5,
      z: (collider.minZ + collider.maxZ) * 0.5,
      yaw: 0,
      halfWidth: (collider.maxX - collider.minX) * 0.5,
      halfLength: (collider.maxZ - collider.minZ) * 0.5
    };
  return footprintContact(body, building);
}


// A pedestrian, bicycle or helicopter is approximated by a horizontal
// cylinder, not a square. Closest-point distance avoids phantom contacts at
// diagonal building corners and matches the teleport clearance calculation.
export function circleBuildingContact(x, z, radius, collider) {
  const o = collider.obb;
  const cs = o ? o.cs : 1, sn = o ? o.sn : 0;
  const cx = o ? o.cx : (collider.minX + collider.maxX) * 0.5;
  const cz = o ? o.cz : (collider.minZ + collider.maxZ) * 0.5;
  const hx = o ? o.hx : (collider.maxX - collider.minX) * 0.5;
  const hz = o ? o.hz : (collider.maxZ - collider.minZ) * 0.5;
  const dx = x - cx, dz = z - cz;
  const lx = dx * cs - dz * sn, lz = dx * sn + dz * cs;
  const px = Math.max(-hx, Math.min(hx, lx));
  const pz = Math.max(-hz, Math.min(hz, lz));
  let nx = lx - px, nz = lz - pz;
  const distance = Math.hypot(nx, nz);
  if (distance >= radius) return null;
  let depth;
  if (distance > 1e-8) {
    nx /= distance;
    nz /= distance;
    depth = radius - distance;
  } else {
    // Center inside the facade: project out along the closest face.
    const edgeX = hx - Math.abs(lx), edgeZ = hz - Math.abs(lz);
    if (edgeX < edgeZ) {
      nx = lx < 0 ? -1 : 1; nz = 0; depth = radius + edgeX;
    } else {
      nx = 0; nz = lz < 0 ? -1 : 1; depth = radius + edgeZ;
    }
  }
  return { nx: nx * cs + nz * sn, nz: -nx * sn + nz * cs, depth };
}

// Friction-free normal impulse. The caller owns road constraints, friction,
// suspension and presentation. Inverse mass zero means an immovable building.
export function impactResponse(aVx, aVz, bVx, bVz, nx, nz, invMassA, invMassB, restitution = 0.18) {
  const closing = -((aVx - bVx) * nx + (aVz - bVz) * nz);
  const invTotal = invMassA + invMassB;
  if (closing <= 0.5 || invTotal <= 0) {
    return { approachSpeed: 0, aVx, aVz, bVx, bVz };
  }
  const impulse = (1 + Math.max(0, Math.min(1, restitution))) * closing / invTotal;
  return {
    approachSpeed: closing,
    aVx: aVx + nx * impulse * invMassA,
    aVz: aVz + nz * impulse * invMassA,
    bVx: bVx - nx * impulse * invMassB,
    bVz: bVz - nz * impulse * invMassB
  };
}
