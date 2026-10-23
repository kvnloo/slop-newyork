const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const test = require('node:test');
const { loadModule } = require('./helpers/module.cjs');

const physics = loadModule(readFileSync(join(__dirname, '../js/collision-physics.js'), 'utf8'));
const { resolveTransportContact } = loadModule(
  readFileSync(join(__dirname, '../js/transport-collisions.js'), 'utf8'),
  { imports: { './collision-physics.js': physics } }
);

function player(type, x, y, z, speed = 0, heading = 0) {
  const position = { x, y, z };
  return {
    type, position, heading, speed, vx: 0, vz: 0, verticalSpeed: 0,
    angularVel: 0, pitchVel: 0, rollVel: 0, heaveVel: 0,
    leanAngle: 0, rollAngle: 0,
    mesh: { position: { copy(p) { assert.equal(p, position); } } }
  };
}

function tram(x = 0, z = 0, bottom = 10, top = 13) {
  return { type: 'tram', x, z, y: (bottom + top) / 2, yaw: 0,
    halfWidth: 4, halfLength: 2, bottom, top, vx: 0, vy: 0, vz: 0 };
}

test('ground car passes safely underneath a suspended tram cabin', () => {
  const car = player('car', 0, 0, 0, 20);
  assert.equal(resolveTransportContact(car, tram()), false);
  assert.equal(car.speed, 20);
});

test('helicopter collides with moving tram at cable height, emitting one impact', () => {
  const heli = player('helicopter', 0, 11, 5.5, -18);
  const impacts = [];
  assert.equal(resolveTransportContact(heli, tram(), hit => impacts.push(hit)), true);
  assert.ok(heli.position.z > 6.49, 'helicopter rotor clears the cabin front');
  assert.ok(heli.speed > -18, 'relative normal speed is reduced');
  assert.ok(heli.vz > 0, 'collision preserves sideways recoil');
  assert.equal(impacts.length, 1);
  assert.ok(impacts[0].speed >= 17);
  assert.equal(impacts[0].ny, 0);
  assert.ok(heli.lastCollisionSpark);
});

test('helicopter rising into the underside rebounds downward', () => {
  const heli = player('helicopter', 0, 11, 0);
  heli.verticalSpeed = 10;
  const impacts = [];
  assert.equal(resolveTransportContact(heli, tram(0, 0, 13, 16),
    hit => impacts.push(hit)), true);
  assert.ok(heli.position.y < 10);
  assert.ok(heli.verticalSpeed < 0);
  assert.equal(impacts[0].ny, -1);
  assert.ok(impacts[0].speed >= 10);
});

test('kinematic tram velocity imparts momentum to a hovering helicopter', () => {
  const cabin = tram();
  cabin.vx = 10;
  const heli = player('helicopter', 6, 11, 0, 0, 0);
  assert.equal(resolveTransportContact(heli, cabin), true);
  assert.ok(heli.position.x > 8.49);
  assert.ok(heli.vx > 10, 'helicopter inherits a bounce from moving cabin');
});

test('player car rebounds from an inactive car in the same parking space', () => {
  const a = player('car', 0, 0, 3.9, -12);
  const b = player('car', 0, 0, 0);
  const impacts = [];
  assert.equal(resolveTransportContact(a, b, hit => impacts.push(hit)), true);
  assert.ok(a.position.z > 4.36);
  assert.ok(a.speed > 0);
  assert.equal(impacts.length, 1);
});

test('player car collides with parked bicycle circular shape, not AABB corners', () => {
  const bike = player('cyclist', 0, 0, 0);
  const car = player('car', 0, 0, 2.0, -9);
  assert.equal(resolveTransportContact(car, bike), true);
  assert.ok(car.position.z > 2);
  assert.ok(car.speed > -9);
  const clear = player('car', 1.6, 0, 3.0);
  assert.equal(resolveTransportContact(clear, bike), false);
});

test('parked helicopter at 90 m cannot collide with car at ground level', () => {
  const car = player('car', 0, 0, 0, 15);
  const heli = player('helicopter', 0, 90, 0);
  assert.equal(resolveTransportContact(car, heli), false);
});

test('separating overlap resolves penetration without phantom impact', () => {
  const a = player('car', 0, 0, 3.9, 10);
  const b = player('car', 0, 0, 0);
  let count = 0;
  assert.equal(resolveTransportContact(a, b, () => count++), true);
  assert.equal(count, 0);
  assert.equal(a.speed, 10);
});
