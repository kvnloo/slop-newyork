const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const test = require('node:test');
const { loadModule } = require('./helpers/module.cjs');

const { footprintContact, buildingContact, impactResponse } = loadModule(
  readFileSync(join(__dirname, '../js/collision-physics.js'), 'utf8')
);

const car = (x, z, yaw = 0) => ({ x, z, yaw, halfWidth: 0.86, halfLength: 2.18 });

test('oriented vehicle bodies separate at the first actual contact', () => {
  assert.equal(footprintContact(car(0, 0), car(0, 4.36)), null);
  const hit = footprintContact(car(0, 0), car(0, 4));
  assert.ok(hit && hit.depth > 0.35 && hit.depth < 0.37);
  assert.ok(hit.nz < -0.99);
  assert.equal(footprintContact(car(0, 0), car(3, 0)), null);
});

test('rotated building footprint does not collide in its empty axis-aligned corner', () => {
  const yaw = Math.PI / 4;
  const obb = { cx: 0, cz: 0, hx: 1, hz: 10, cs: Math.cos(yaw), sn: Math.sin(yaw) };
  const c = { minX: -8, maxX: 8, minZ: -8, maxZ: 8, obb };
  assert.equal(buildingContact(car(7, -7), c), null);
  const hit = buildingContact(car(3, 3), c);
  assert.ok(hit && hit.depth > 0);
});

test('static wall restitution changes velocity without moving the wall', () => {
  const hit = impactResponse(10, 0, 0, 0, -1, 0, 1, 0, 0.25);
  assert.equal(hit.approachSpeed, 10);
  assert.ok(Math.abs(hit.aVx + 2.5) < 1e-8);
  assert.equal(hit.bVx, 0);
});

test('equal-mass cars exchange normal momentum with restitution', () => {
  const hit = impactResponse(10, 0, 0, 0, -1, 0, 1 / 1400, 1 / 1400, 0.2);
  assert.ok(Math.abs(hit.aVx - 4) < 1e-8);
  assert.ok(Math.abs(hit.bVx - 6) < 1e-8);
});

test('separating and low-speed contacts add no energy', () => {
  const separating = impactResponse(-2, 0, 0, 0, -1, 0, 1, 1);
  assert.equal(separating.approachSpeed, 0);
  assert.equal(separating.aVx, -2);
  const staticPair = impactResponse(10, 0, 0, 0, -1, 0, 0, 0);
  assert.equal(staticPair.aVx, 10);
});
