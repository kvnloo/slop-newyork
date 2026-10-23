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
