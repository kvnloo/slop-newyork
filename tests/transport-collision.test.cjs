const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const test = require('node:test');
const vm = require('node:vm');
const { loadModule } = require('./helpers/module.cjs');

const physics = loadModule(readFileSync(join(__dirname, '../js/collision-physics.js'), 'utf8'));
const traffic = readFileSync(join(__dirname, '../js/traffic.js'), 'utf8');
const controllerSource = readFileSync(join(__dirname, '../js/controllers.js'), 'utf8');

function trafficFunction(name, context) {
  const match = traffic.match(new RegExp(`  function ${name}\\([^]*?\\n  \\}`));
  assert.ok(match, `Missing production function: ${name}`);
  return vm.runInNewContext(`(${match[0]})`, context);
}

function control(mode, collider) {
  const { ThirdPersonControllerManager } = loadModule(controllerSource, {
    imports: {
      'three/webgpu': {},
      './city-config.js': { ISLAND_BOUNDS: {
        minX: -500, maxX: 500, minZ: -500, maxZ: 500
      }, getIslandBoundsAtZ: null, streetZ: () => 0 },
      './tsl-materials.js': {},
      './actor-models.js': {},
      './pedestrians.js': {},
      './vehicle-lights.js': {},
      './collision-physics.js': physics,
      './landmarks.js': {}
    }
  });
  const controller = Object.create(ThirdPersonControllerManager.prototype);
  controller.activeMode = mode;
  controller.colliders = [collider];
  controller._colliderCells = null;
  const events = [];
  controller.setCollisionListener(event => events.push({ ...event }));
  return { controller, events };
}

test('taxi impact with a facade separates, rebounds and emits a building crash', () => {
  const building = { minX: 0, maxX: 8, minZ: -8, maxZ: 8, maxY: 25 };
  const { controller, events } = control('car', building);
  const position = { x: -1.4, y: 0, z: 0 };
  const taxi = {
    type: 'car', heading: Math.PI / 2, speed: 12,
    vx: 0, vz: 0, angularVel: 0, pitchVel: 0, rollVel: 0, heaveVel: 0
  };
  controller.resolveHorizontalCollision(position, 1.35, 1, taxi);
  assert.ok(position.x < -2.17, 'taxi stays outside the facade');
  assert.ok(taxi.speed < 0, 'head-on hit rebounds');
  assert.equal(events.length, 1);
  assert.equal(events[0].kind, 'building');
  assert.ok(events[0].speed > 10);
});

for (const mode of ['cyclist', 'helicopter']) {
  test(`${mode} crashes into a wall, loses speed and sparks`, () => {
    const building = { minX: 0, maxX: 12, minZ: -8, maxZ: 8, maxY: 50 };
    const { controller, events } = control(mode, building);
    const radius = mode === 'helicopter' ? 4.5 : 0.65;
    const position = { x: -radius * 0.5, y: mode === 'helicopter' ? 8 : 0, z: 0 };
    const entity = {
      type: mode, heading: Math.PI / 2, speed: 12, verticalSpeed: -3,
      pitchAngle: 0, rollAngle: 0, leanAngle: 0
    };
    controller.resolveHorizontalCollision(position, radius, position.y, null, entity);
    assert.ok(position.x < -radius + 0.01);
    assert.ok(entity.speed < 0);
    assert.ok(entity.lastCollisionSpark?.speed > 2.2);
    assert.equal(events[0]?.kind, 'building');
  });
}

test('descending helicopter lands on roof with vertical bounce and crash audio', () => {
  const roof = { minX: -12, maxX: 12, minZ: -12, maxZ: 12, maxY: 25 };
  const { controller, events } = control('helicopter', roof);
  const position = { x: 0, y: 25.4, z: 0 };
  const helicopter = { type: 'helicopter', heading: 0, speed: 0,
    verticalSpeed: -10, rollAngle: 0, pitchAngle: 0 };
  controller.resolveHorizontalCollision(position, 4.5, position.y, null, helicopter);
  assert.ok(Math.abs(position.y - 25.7) < 1e-8);
  assert.ok(helicopter.verticalSpeed > 0, 'vertical velocity rebounds');
  assert.equal(events.length, 1);
  assert.equal(events[0].kind, 'building');
  assert.equal(events[0].ny, 1);
  assert.ok(events[0].speed >= 10);
});

test('airborne helicopter above rooftop ignores the horizontal footprint', () => {
  const { controller, events } = control('helicopter',
    { minX: -5, maxX: 5, minZ: -5, maxZ: 5, maxY: 25 });
  const position = { x: 0, y: 70, z: 0 };
  const heli = { type: 'helicopter', heading: 0, speed: 12 };
  controller.resolveHorizontalCollision(position, 4.5, position.y, null, heli);
  assert.equal(position.x, 0);
  assert.equal(position.z, 0);
  assert.equal(heli.speed, 12);
  assert.equal(events.length, 0);
});

test('knocked city traffic car bounces off a facade instead of clipping', () => {
  const building = { minX: 0, maxX: 10, minZ: -10, maxZ: 10, maxY: 40 };
  const reports = [];
  const collide = trafficFunction('collideWithBuildings', {
    collisionQuery: () => [building],
    buildingContact: physics.buildingContact,
    impactResponse: physics.impactResponse,
    shove: (v, dx, dz) => { v.x += dx; v.z += dz; },
    reportCollision: (...args) => reports.push(args),
    spawnCollisionSparks: () => {}
  });
  const v = {
    x: -0.2, z: 0, yaw: Math.PI / 2, yawOffset: 0,
    tx: 1, tz: 0, speed: 7, vx: 2, vz: 0,
    half: 2.18, halfWidth: 0.86, yawVel: 0, rollVel: 0
  };
  collide(v);
  assert.ok(v.x < -2.17);
  assert.equal(v.speed, 0);
  assert.equal(reports[0].at(-1), 'building');
});

test('highway car transfers momentum to player and plays crash event', () => {
  const hits = [];
  const collide = trafficFunction('collideWithHighwayCar', {
    footprintContact: physics.footprintContact,
    impactResponse: physics.impactResponse,
    collisionListener: event => hits.push({ ...event }),
    collisionEvent: { kind: 'vehicle' }
  });
  const car = { x: 0, z: 0, y: 7.6, yaw: 0, v: 18, kind: 'sedan', lane: { speed: 24 } };
  const player = {
    position: { x: 0, y: 7.6, z: 3.9 },
    mesh: { position: { copy() {} } },
    type: 'car', heading: 0, speed: 0, vx: 0, vz: 0,
    angularVel: 0, pitchVel: 0, rollVel: 0, heaveVel: 0
  };
  collide(car, player, 'car');
  assert.ok(player.position.z > 3.9, 'player is separated from lane car');
  assert.ok(player.speed > 0, 'player picks up forward momentum');
  assert.ok(car.v < 18, 'highway car loses momentum');
  assert.equal(hits.length, 1);
  assert.equal(hits[0].kind, 'vehicle');
  assert.ok(hits[0].speed > 2.2);
});

test('highway collision ignores road traffic above or below the player', () => {
  const collide = trafficFunction('collideWithHighwayCar', {
    footprintContact: physics.footprintContact,
    impactResponse: physics.impactResponse,
    collisionListener: () => { throw new Error('should not collide'); },
    collisionEvent: {}
  });
  const car = { x: 0, z: 0, y: 7.6, yaw: 0, v: 18, kind: 'sedan', lane: { speed: 24 } };
  const player = { position: { x: 0, y: 0, z: 3.9 }, heading: 0, speed: 0 };
  collide(car, player, 'car');
  assert.equal(player.position.z, 3.9);
  assert.equal(car.v, 18);
});
