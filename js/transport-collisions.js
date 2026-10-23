import { footprintContact, circleBuildingContact, impactResponse } from './collision-physics.js';

// Collide the controlled transport with moving tram cabins or parked vehicles.
// The obstacles remain kinematic; the player takes the separating impulse.
export function resolveTransportContact(entity, obstacle, report = null) {
  const p = entity?.position;
  if (!p || !obstacle || entity === obstacle) return false;
  const aType = entity.type;
  const bType = obstacle.type;
  const aRadius = aType === 'helicopter' ? 4.5 : aType === 'cyclist' ? 0.65
    : aType === 'person' ? 0.48 : 0;
  const bRadius = bType === 'helicopter' ? 4.5 : bType === 'cyclist' ? 0.65
    : bType === 'person' ? 0.48 : 0;
  const aWidth = aRadius || 0.86, aLength = aRadius || 2.18;
  const bWidth = obstacle.halfWidth ?? (bRadius || 0.86);
  const bLength = obstacle.halfLength ?? (bRadius || 2.18);
  const bp = obstacle.position || obstacle;
  if (Math.abs(p.x - bp.x) > aLength + aWidth + bLength + bWidth ||
      Math.abs(p.z - bp.z) > aLength + aWidth + bLength + bWidth) return false;
  const aBottom = p.y - (aType === 'helicopter' ? 0.7 : 0);
  const aTop = p.y + (aType === 'helicopter' ? 3.2 : aType === 'person' ? 1.8 : 1.9);
  const bBottom = obstacle.bottom ?? (bp.y - (bType === 'helicopter' ? 0.7 : 0));
  const bTop = obstacle.top ?? (bp.y + (bType === 'helicopter' ? 3.2 : 1.9));
  if (aBottom >= bTop || aTop <= bBottom) return false;

  const yaw = entity.heading || 0, byaw = obstacle.yaw ?? obstacle.heading ?? 0;
  const bBox = { obb: { cx: bp.x, cz: bp.z, hx: bWidth, hz: bLength,
    cs: Math.cos(byaw), sn: Math.sin(byaw) } };
  let hit;
  if (!aRadius && !bRadius) {
    hit = footprintContact({ x: p.x, z: p.z, yaw, halfWidth: aWidth, halfLength: aLength },
      { x: bp.x, z: bp.z, yaw: byaw, halfWidth: bWidth, halfLength: bLength });
  } else if (aRadius && !bRadius) {
    hit = circleBuildingContact(p.x, p.z, aRadius, bBox);
  } else if (!aRadius && bRadius) {
    const aBox = { obb: { cx: p.x, cz: p.z, hx: aWidth, hz: aLength,
      cs: Math.cos(yaw), sn: Math.sin(yaw) } };
    const reversed = circleBuildingContact(bp.x, bp.z, bRadius, aBox);
    hit = reversed && { nx: -reversed.nx, nz: -reversed.nz, depth: reversed.depth };
  } else {
    const dx = p.x - bp.x, dz = p.z - bp.z, distance = Math.hypot(dx, dz);
    const depth = aRadius + bRadius - distance;
    hit = depth > 0 ? { nx: distance > 1e-6 ? dx / distance : 1,
      nz: distance > 1e-6 ? dz / distance : 0, depth } : null;
  }
  if (!hit) return false;

  const sin = Math.sin(yaw), cos = Math.cos(yaw);
  let nx = hit.nx, nz = hit.nz, ny = 0, impact = 0;
  const rise = bTop - aBottom, fall = aTop - bBottom;
  // Helicopters can collide with the tram roof or underside. Ground
  // transport instead stays on the road and separates horizontally.
  if (aType === 'helicopter' && Math.min(rise, fall) < hit.depth) {
    ny = rise <= fall ? 1 : -1;
    p.y += ny * (Math.min(rise, fall) + 0.002);
    const closing = -((entity.verticalSpeed || 0) - (obstacle.vy || 0)) * ny;
    impact = Math.max(0, closing);
    if (closing > 0.5) entity.verticalSpeed = (obstacle.vy || 0) + ny * closing * 0.16;
    nx = nz = 0;
  } else {
    p.x += nx * (hit.depth + 0.002);
    p.z += nz * (hit.depth + 0.002);
    const avx = sin * (entity.speed || 0) + (entity.vx || 0);
    const avz = cos * (entity.speed || 0) + (entity.vz || 0);
    const response = impactResponse(avx, avz, obstacle.vx || 0, obstacle.vz || 0,
      nx, nz, 1, 0, bType === 'tram' ? 0.12 : 0.18);
    impact = response.approachSpeed;
    if (impact) {
      const limit = aType === 'helicopter' ? 52 : aType === 'car' ? 36
        : aType === 'cyclist' ? 19 : 8.2;
      entity.speed = Math.max(-limit, Math.min(limit, response.aVx * sin + response.aVz * cos));
      entity.vx = response.aVx - sin * entity.speed;
      entity.vz = response.aVz - cos * entity.speed;
      const side = cos * nx - sin * nz;
      if (aType === 'car') {
        entity.angularVel = (entity.angularVel || 0) + side * Math.min(2.5, impact * 0.16);
        entity.rollVel = (entity.rollVel || 0) - side * Math.min(9, impact) * 0.20;
        entity.pitchVel = (entity.pitchVel || 0) + (sin * nx + cos * nz) * Math.min(10, impact) * 0.15;
        entity.heaveVel = (entity.heaveVel || 0) + Math.min(1.3, impact * 0.08);
      } else if (aType === 'cyclist') {
        entity.leanAngle = Math.max(-0.6, Math.min(0.6,
          (entity.leanAngle || 0) + side * Math.min(0.4, impact * 0.045)));
      } else if (aType === 'helicopter') {
        entity.rollAngle = Math.max(-0.45, Math.min(0.45,
          (entity.rollAngle || 0) + side * Math.min(0.3, impact * 0.025)));
      }
    }
  }

  entity.mesh?.position.copy(p);
  if (impact > 0.5) {
    const contact = {
      x: (p.x + bp.x) * 0.5,
      y: Math.min(bTop, Math.max(bBottom, p.y + (aType === 'helicopter' ? 1 : 0.6))),
      z: (p.z + bp.z) * 0.5,
      nx, ny, nz, speed: impact
    };
    if (typeof report === 'function') report(contact);
    if (aType !== 'person' && impact > 2.2) entity.lastCollisionSpark = contact;
  }
  return true;
}
