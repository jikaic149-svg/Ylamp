// Offline CAD-coordinate surface collision checks. Never connects to hardware.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { resolve, dirname, relative, isAbsolute } from 'node:path';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { XMLParser, XMLValidator } from 'fast-xml-parser';
import { Box3, BoxGeometry, BufferGeometry, Float32BufferAttribute, Euler, Line3, Matrix4, Vector3 } from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ExtendedTriangle, MeshBVH, SAH } from 'three-mesh-bvh';

const DEG = Math.PI / 180;
const array = x => x == null ? [] : Array.isArray(x) ? x : [x];
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const triple = (s, fallback) => {
  const v = s == null ? fallback : String(s).trim().split(/\s+/).map(Number);
  assert(v.length === 3 && v.every(Number.isFinite), `Invalid xyz/rpy/scale: ${s}`);
  return v;
};
function origin(value = {}) {
  const [r, p, y] = triple(value.rpy, [0, 0, 0]);
  // URDF fixed-axis RPY = Rz(yaw)*Ry(pitch)*Rx(roll), not Three's default XYZ.
  return new Matrix4().makeRotationFromEuler(new Euler(r, p, y, 'ZYX'))
    .setPosition(...triple(value.xyz, [0, 0, 0]));
}

// The BVH library returns a dummy zero line for coplanar intersections.
// Clip the actual triangles instead; zero must not accidentally allow contact.
function coplanarPoints(a, b) {
  let poly = [a.a.clone(), a.b.clone(), a.c.clone()];
  const normal = b.getNormal(new Vector3());
  for (const [u, v] of [[b.a, b.b], [b.b, b.c], [b.c, b.a]]) {
    const edge = v.clone().sub(u);
    const distance = p => normal.dot(edge.clone().cross(p.clone().sub(u)));
    const clipped = [];
    for (let i = 0; i < poly.length; ++i) {
      const p = poly[i], q = poly[(i + 1) % poly.length];
      const dp = distance(p), dq = distance(q);
      const ip = dp >= -1e-14, iq = dq >= -1e-14;
      if (ip) clipped.push(p);
      if (ip !== iq) clipped.push(p.clone().lerp(q, dp / (dp - dq)));
    }
    poly = clipped;
  }
  return poly;
}
function intersectionPoints(a, b, line) {
  const n1 = a.getNormal(new Vector3()), n2 = b.getNormal(new Vector3());
  const coplanar = Math.abs(n1.dot(n2)) > 1 - 1e-9
    && Math.abs(n1.dot(b.a.clone().sub(a.a))) < 1e-9;
  return coplanar ? coplanarPoints(a, b) : [line.start, line.end];
}
function allowedMountContact(a, b, line, rule) {
  const points = intersectionPoints(a, b, line);
  return boundedPlaneContact(points, rule);
}
function boundedPlaneContact(points, rule) {
  return points.length > 0 && points.every(p =>
    [p.x, p.y, p.z].every(Number.isFinite)
    && Math.abs(p.y - rule.y) <= rule.axialTolerance
    && Math.hypot(p.x, p.z) <= rule.radius);
}
function crossingEvidence(a, b) {
  const signed = (plane, triangle) => {
    const normal = plane.getNormal(new Vector3());
    return [triangle.a, triangle.b, triangle.c].map(p => normal.dot(p.clone().sub(plane.a)) * 1000);
  };
  const aPlaneBVerticesMm = signed(a, b), bPlaneAVerticesMm = signed(b, a);
  // Plane straddling is evidence of a transverse crossing, NOT solid penetration depth.
  const transverseDepthMm = Math.min(...[aPlaneBVerticesMm, bPlaneAVerticesMm]
    .flatMap(v => [Math.max(...v), -Math.min(...v)]));
  return { aPlaneBVerticesMm, bPlaneAVerticesMm, transverseDepthMm };
}

function clipPolygon(points, signed) {
  const result = [];
  for (let i = 0; i < points.length; ++i) {
    const a = points[i], b = points[(i + 1) % points.length];
    const da = signed(a), db = signed(b);
    if (da >= 0) result.push(a);
    if ((da >= 0) !== (db >= 0)) result.push(a.clone().lerp(b, da / (da - db)));
  }
  return result;
}
function prunePermittedPlane(geometry, toChild, rule) {
  // Adjacent hinge motion is a rotation about child Y: y and xz radius are
  // invariant. Remove ORIGINAL SURFACE only inside the explicitly allowed
  // slab/disk; no caps and no whole-pair exemption. Outside points are retained.
  const position = geometry.attributes.position, index = geometry.index;
  const count = index ? index.count : position.count, result = [];
  const emit = polygon => {
    for (let k = 1; k + 1 < polygon.length; ++k)
      for (const p of [polygon[0], polygon[k], polygon[k + 1]]) result.push(p.x, p.y, p.z);
  };
  const band = rule.axialTolerance * .99; // Leave a rounding guard, not extra allowance.
  const half = rule.radius / Math.sqrt(2) * .999999;
  const signed = [
    p => half-p.x, p => half+p.x, p => half-p.z, p => half+p.z,
    p => rule.y+band-p.y, p => p.y-rule.y+band,
  ];
  if (rule.radius < .02) {
    signed.splice(0, 4, ...Array.from({length:32}, (_, k) => {
      const angle = k * Math.PI / 16, limit = rule.radius * Math.cos(Math.PI/32) * .999999;
      return p => limit - Math.cos(angle)*p.x - Math.sin(angle)*p.z;
    }));
  }
  for (let i = 0; i < count; i += 3) {
    const points = [0, 1, 2].map(k => new Vector3().fromBufferAttribute(position, index ? index.getX(i+k) : i+k));
    let remaining = points;
    // Partition the triangle outside an INSCRIBED square prism. Every removed
    // point is inside the allowed disk; triangles spanning beyond it stay checked.
    for (const inside of signed) {
      const f = p => inside(p.clone().applyMatrix4(toChild));
      emit(clipPolygon(remaining, p => -f(p)));
      remaining = clipPolygon(remaining, f);
      if (!remaining.length) break;
    }
  }
  const reduced = new BufferGeometry();
  reduced.setAttribute('position', new Float32BufferAttribute(result, 3));
  reduced.computeBoundingBox();
  return reduced;
}

function selfTest() {
  const v = new Vector3(0, 0, 1).applyMatrix4(origin({ rpy: `${Math.PI / 2} 0 ${Math.PI / 2}` }));
  assert(v.distanceTo(new Vector3(1, 0, 0)) < 1e-12);
  const axis = new Vector3(0, -1, 0);
  assert(new Vector3(1, 0, 0).applyMatrix4(new Matrix4().makeRotationAxis(axis, Math.PI / 2))
    .distanceTo(new Vector3(0, 0, 1)) < 1e-12);
  const geom = new BoxGeometry(1, 1, 1);
  const tree = new MeshBVH(geom);
  assert(tree.intersectsGeometry(geom, new Matrix4().makeTranslation(0.5, 0, 0)));
  assert(!tree.intersectsGeometry(geom, new Matrix4().makeTranslation(3, 0, 0)));
  const tri = x => new ExtendedTriangle(new Vector3(x, 0, 0),
    new Vector3(x + 0.002, 0, 0), new Vector3(x, 0, 0.002));
  const rule = { y: 0, radius: 0.016, axialTolerance: 0.00003 };
  for (const [x, expected] of [[0.002, true], [0.02, false]]) {
    const a = tri(x), b = tri(x), line = new Line3();
    assert(a.intersectsTriangle(b, line, true));
    assert.equal(allowedMountContact(a, b, line, rule), expected);
  }
  assert.equal(coplanarPoints(tri(0), tri(0.02)).length, 0);
  assert(Math.abs(crossingEvidence(tri(0), tri(0)).transverseDepthMm) < 1e-12);
  const crossing = new ExtendedTriangle(new Vector3(0.0005, -0.001, 0.0005),
    new Vector3(0.0005, 0.001, 0.0005), new Vector3(0.0005, 0, 0.0015));
  assert(crossingEvidence(tri(0), crossing).transverseDepthMm > 0.4);
  assert(!boundedPlaneContact([new Vector3(0, .0001, 0)], rule));
  assert(!boundedPlaneContact([new Vector3(.017, 0, 0)], rule));
  assert(!boundedPlaneContact([], rule));
  const cut = clipPolygon([new Vector3(0, -1, 0), new Vector3(1, 1, 0), new Vector3(-1, 1, 0)], p => p.y-.1);
  assert(cut.length === 4 && cut.every(p => p.y >= .1-1e-12));
  const surface = new BufferGeometry().setAttribute('position', new Float32BufferAttribute(
    [0, 0, 0, .002, 0, 0, 0, 0, .002, 0, .001, 0, .002, .001, 0, 0, .001, .002], 3));
  const pruned = prunePermittedPlane(surface, new Matrix4(), rule);
  assert.equal(pruned.attributes.position.count, 3);
  assert(pruned.attributes.position.getY(0) > rule.axialTolerance);
  assert.equal(cartesian([[1, 2], [3, 4]]).length, 4);
  console.log('SELF_TEST PASS: URDF RPY/axis, mesh overlap/separation, bounded coplanar mounting contact, pose combinations');
}

export function loadModel(urdfPath, installationRules = {}, pruneInstallation = true) {
  const bytes = readFileSync(urdfPath);
  assert.equal(XMLValidator.validate(bytes.toString()), true, 'Invalid URDF XML');
  const robot = new XMLParser({ ignoreAttributes: false, attributeNamePrefix: '' }).parse(bytes).robot;
  const links = array(robot.link), joints = array(robot.joint);
  assert.equal(joints.length, 5);
  assert.deepEqual(joints.map(j => j.name), ['J1', 'J2', 'J3', 'J4', 'J5']);
  assert.equal(links.length, 6);
  const childNames = new Set(joints.map(j => j.child.link));
  const roots = links.filter(l => !childNames.has(l.name));
  assert.equal(roots.length, 1);
  let parent = roots[0].name;
  const chain = [parent];
  const limits = [], axes = [], origins = [];
  for (const j of joints) {
    assert.equal(j.type, 'revolute');
    assert.equal(j.parent.link, parent, 'Only the serial Ylamp chain is supported');
    parent = j.child.link;
    chain.push(parent);
    const lim = [Number(j.limit.lower) / DEG, Number(j.limit.upper) / DEG];
    assert(lim.every(Number.isFinite) && lim[0] <= 0 && lim[1] >= 0 && lim[0] < lim[1]);
    limits.push(lim);
    const axis = new Vector3(...triple(j.axis?.xyz, [1, 0, 0]));
    assert(Math.abs(axis.length() - 1) < 1e-9);
    axes.push(axis);
    origins.push(origin(j.origin));
  }
  const root = dirname(urdfPath);
  const policyBytes = readFileSync(resolve(root, 'solidworks-collision-scan.json'));
  const policy = JSON.parse(policyBytes);
  assert.equal(policy.policy, 'main-bodies-only-current-solidworks-v1');
  assert.equal(policy.contactRuleEvidence.pair, 'part7/part9');
  const mountRule = {
    y: policy.surfaceOverrides['part7/part9'].y / 1000,
    radius: policy.contactRuleEvidence.allowedRadiusMm / 1000,
    axialTolerance: policy.contactRuleEvidence.axialToleranceMm / 1000,
  };
  const bodies = [], meshHashes = {};
  for (const link of links) {
    const linkIndex = chain.indexOf(link.name);
    assert(linkIndex >= 0);
    for (const c of array(link.collision)) {
      const mesh = c.geometry?.mesh;
      assert(mesh, 'Only triangle mesh collision geometry is supported');
      const name = String(mesh.filename).replace(/^package:\/\/[^/]+\//, '');
      const file = resolve(root, name), rel = relative(root, file);
      assert(rel && !rel.startsWith('..') && !isAbsolute(rel), 'Mesh escaped the URDF package');
      const stl = readFileSync(file);
      meshHashes[name] = hash(stl);
      const geometry = new STLLoader().parse(stl.buffer.slice(stl.byteOffset, stl.byteOffset + stl.byteLength));
      geometry.scale(...triple(mesh.scale, [1, 1, 1]));
      geometry.computeBoundingBox();
      const bvh = new MeshBVH(geometry, { strategy: SAH, targetLeafSize: 10, verbose: false });
      geometry.boundsTree = bvh;
      bodies.push({ name: c.name, linkIndex, local: origin(c.origin), geometry, bvh,
        world: new Matrix4(), inverse: new Matrix4(), box: new Box3() });
      console.log(`Loaded ${c.name}: ${geometry.attributes.position.count / 3} triangles`);
    }
  }
  assert.equal(bodies.length, 14);
  assert.equal(new Set(bodies.map(b => b.name)).size, bodies.length);
  const pairs = [];
  const zero = [new Matrix4()];
  for (const transform of origins) zero.push(zero.at(-1).clone().multiply(transform));
  for (let i = 0; i < bodies.length; ++i) {
    for (let k = i + 1; k < bodies.length; ++k) {
      let a = bodies[i], b = bodies[k];
      if (a.linkIndex === b.linkIndex) continue; // Same rigid link, not an adjacent-link exemption.
      if (a.name === 'part9' && b.name === 'part7') [a, b] = [b, a];
      const name = [a.name, b.name].sort().join('/');
      const pair = { a, b, name, cache: new Map(), aBVH: a.bvh, bBVH: b.bvh };
      const originalMount = name === 'part7/part9' && Object.keys(installationRules).length;
      if (originalMount) assert(zero[1].elements.every((v, j) => Math.abs(v-new Matrix4().elements[j]) < 1e-9));
      const rule = installationRules[name] ?? (originalMount ? mountRule : null);
      if (rule && pruneInstallation) {
        const high = Math.max(a.linkIndex, b.linkIndex);
        assert.equal(Math.abs(a.linkIndex-b.linkIndex), 1, 'Plane pruning only supports adjacent Y hinges');
        for (const [body, key] of [[a, 'aBVH'], [b, 'bBVH']]) {
          const matrix = zero[high].clone().invert().multiply(zero[body.linkIndex]).multiply(body.local);
          const geometry = prunePermittedPlane(body.geometry, matrix, rule);
          pair[key] = new MeshBVH(geometry, { strategy: SAH, targetLeafSize: 10, verbose: false });
          geometry.boundsTree = pair[key];
        }
      }
      pairs.push(pair);
    }
  }
  const relativeMatrix = new Matrix4(), line = new Line3(), toChild = new Matrix4();
  function collisions(q, details = null, timing = null) {
    assert(q.length === 5 && q.every(Number.isFinite));
    const world = [new Matrix4()];
    for (let i = 0; i < 5; ++i) {
      world.push(world[i].clone().multiply(origins[i])
        .multiply(new Matrix4().makeRotationAxis(axes[i], q[i] * DEG)));
    }
    for (const b of bodies) {
      b.world.multiplyMatrices(world[b.linkIndex], b.local);
      b.inverse.copy(b.world).invert();
      b.box.copy(b.geometry.boundingBox).applyMatrix4(b.world);
    }
    const hits = [];
    for (const pair of pairs) {
      const started = timing ? performance.now() : 0;
      const { a, b, cache } = pair;
      const low = Math.min(a.linkIndex, b.linkIndex), high = Math.max(a.linkIndex, b.linkIndex);
      // Relative geometry does not depend on the joints preceding both links.
      const key = q.slice(low, high).join(',');
      let hit = cache.get(key);
      if (hit === undefined || details) {
        hit = false;
        if (a.box.intersectsBox(b.box)) {
          relativeMatrix.multiplyMatrices(a.inverse, b.world);
          toChild.copy(world[high]).invert().multiply(a.world);
          // The library's oriented-box traversal is much tighter than bvhcast's
          // transformed AABBs for nested, dense motor/case meshes.
          const possible = !pruneInstallation || !Object.keys(installationRules).length
            || pair.aBVH.intersectsGeometry(pair.bBVH.geometry, relativeMatrix);
          hit = possible && pair.aBVH.bvhcast(pair.bBVH, relativeMatrix, {
            intersectsTriangles: (ta, tb) => {
              if (!ta.intersectsTriangle(tb, line, true)) return false;
              if (pair.name === 'part7/part9' && allowedMountContact(ta, tb, line, mountRule)) return false;
              const rule = installationRules[pair.name];
              if (rule) {
                const points = intersectionPoints(ta, tb, line).map(p => p.clone().applyMatrix4(toChild));
                if (boundedPlaneContact(points, rule)) return false;
              }
              if (details) {
                details.push({ pair: pair.name, childLink: chain[high],
                  crossing: crossingEvidence(ta, tb),
                  pointsInChildM: intersectionPoints(ta, tb, line).map(p => p.clone().applyMatrix4(toChild).toArray()),
                  triangleAInChildM: [ta.a, ta.b, ta.c].map(p => p.clone().applyMatrix4(toChild).toArray()),
                  triangleBInChildM: [tb.a, tb.b, tb.c].map(p => p.clone().applyMatrix4(toChild).toArray()) });
              }
              return true;
            },
          });
        }
        cache.set(key, hit);
      }
      if (hit) hits.push(pair.name);
      if (timing) timing.push({pair:pair.name, milliseconds:performance.now()-started, hit});
    }
    return hits;
  }
  function floorHits(urdfY = .0587451) {
    const hits = [];
    for (const b of bodies) {
      if (!b.linkIndex || b.box.max.y <= urdfY) continue;
      const above = p => p.clone().applyMatrix4(b.world).y > urdfY + 1e-8;
      if (b.bvh.shapecast({
        intersectsBounds: box => box.clone().applyMatrix4(b.world).max.y > urdfY,
        intersectsTriangle: t => [t.a, t.b, t.c].some(above),
      })) hits.push(`${b.name}/floor`);
    }
    return hits;
  }
  function exportFcl(directory) {
    mkdirSync(directory, {recursive:true});
    const meshes = new Map(), manifest = {bodies:[], pairs:[], origins:origins.map(m=>m.toArray()), axes:axes.map(a=>a.toArray())};
    const mesh = geometry => {
      if (meshes.has(geometry)) return meshes.get(geometry);
      const name = `mesh_${meshes.size}`;
      const position = geometry.attributes.position.array;
      const index = Uint32Array.from(geometry.index ? geometry.index.array : Array.from({length:position.length/3},(_,i)=>i));
      writeFileSync(resolve(directory,name+'.vertices'), Buffer.from(position.buffer,position.byteOffset,position.byteLength),{flag:'wx'});
      writeFileSync(resolve(directory,name+'.indices'), Buffer.from(index.buffer),{flag:'wx'});
      meshes.set(geometry,name); return name;
    };
    for (const b of bodies) manifest.bodies.push({name:b.name,linkIndex:b.linkIndex,local:b.local.toArray(),mesh:mesh(b.geometry)});
    for (const p of pairs) manifest.pairs.push({a:p.a.name,b:p.b.name,name:p.name,aMesh:mesh(p.aBVH.geometry),bMesh:mesh(p.bBVH.geometry)});
    writeFileSync(resolve(directory,'manifest.json'),JSON.stringify(manifest,null,2),{flag:'wx'});
    return {directory, meshes:meshes.size, originalBodies:bodies.length};
  }
  return { collisions, floorHits, exportFcl, limits, input: { urdf: urdfPath, urdfSha256: hash(bytes),
    policySha256: hash(policyBytes), meshSha256: meshHashes, mountContactRuleM: mountRule,
    conditionalInstallationRules: installationRules,
    permittedContactSurfacePruned: pruneInstallation && Object.keys(installationRules).length > 0,
    triangleCount: bodies.reduce((n, b) => n + b.geometry.attributes.position.count / 3, 0),
    crossLinkBodyPairs: pairs.length } };
}

function cartesian(values) {
  return values.reduce((rows, axis) => rows.flatMap(r => axis.map(v => [...r, v])), [[]]);
}
function grid([low, high], n) {
  return [...new Set([...Array.from({ length: n }, (_, k) => low + (high - low) * k / (n - 1)), 0])].sort((a, b) => a - b);
}
function run(model, options) {
  const { collisions, limits } = model;
  const baselineDetails = [];
  const baseline = collisions([0, 0, 0, 0, 0], baselineDetails);
  const report = {
    schema: 'ylamp-offline-combination-scan-v1', generatedUtc: new Date().toISOString(),
    input: model.input, jointLimitsCadDeg: Object.fromEntries(limits.map((v, i) => [`J${i + 1}`, v])),
    scope: 'J2-J5 combinations; J1 held at CAD zero. No motor commands or firmware changes.',
    calibration: 'CAD to encoder offsets, signs and turn branches are NOT verified.',
    guarantee: 'Finite sampling and triangle-surface intersection only, not a continuous-space, clearance, containment or hardware-safety certificate.',
    options, baseline, baselineDetails, scans: {}, pairCounts: {}, newPairCounts: {},
    witnesses: [], additionalCollisionSamples: [],
  };
  if (baseline.length) {
    report.warning = 'Baseline surface contacts differ from export metadata. They remain checked and reported at every pose; no collision-free certificate can be issued.';
  }
  if (options['baseline-only']) return report;
  if (options.pose) {
    const tokens = options.pose.split(',');
    const q = tokens.map(Number);
    assert(tokens.length === 5 && tokens.every(t => t.trim()) && q.every(Number.isFinite),
      '--pose needs five finite comma-separated CAD degrees');
    assert(Math.abs(q[0]) < 1e-12, 'This study holds J1 at CAD zero');
    assert(q.every((v, i) => v >= limits[i][0] - 1e-8 && v <= limits[i][1] + 1e-8), 'Pose outside URDF bounds');
    const details = [];
    report.poseCheck = { cadDegrees: q, pairs: collisions(q, details), details };
    return report;
  }
  let checked = 0;
  const check = (mode, q) => {
    const pairs = collisions(q);
    const stat = report.scans[mode] ??= { checked: 0, colliding: 0, withNewPairs: 0 };
    ++stat.checked;
    if (pairs.length) {
      ++stat.colliding;
      const newPairs = pairs.filter(pair => !baseline.includes(pair));
      if (newPairs.length) {
        ++stat.withNewPairs;
        report.additionalCollisionSamples.push({ mode, cadDegrees: q, pairs: newPairs });
      }
      for (const pair of pairs) {
        report.pairCounts[pair] = (report.pairCounts[pair] ?? 0) + 1;
        if (!baseline.includes(pair)) report.newPairCounts[pair] = (report.newPairCounts[pair] ?? 0) + 1;
        if (!report.witnesses.some(w => w.pair === pair && w.mode === mode)) {
          report.witnesses.push({ mode, pair, presentAtZero: baseline.includes(pair),
            cadDegrees: Object.fromEntries(q.map((v, i) => [`J${i + 1}`, v])) });
        }
      }
    }
    if (++checked % 250 === 0) console.log(`Checked ${checked} poses; mode=${mode}; rawSurfaceHits=${stat.colliding}/${stat.checked}; withNewPairs=${stat.withNewPairs}`);
    return pairs;
  };
  // Single-axis checks are a regression reference, not the claimed safe space.
  for (let axis = 1; axis < 5; ++axis) {
    const n = Math.ceil((limits[axis][1] - limits[axis][0]) / Number(options.step)) + 1;
    for (const value of grid(limits[axis], n)) {
      const q = [0, 0, 0, 0, 0]; q[axis] = value;
      check(`single-J${axis + 1}`, q);
    }
  }
  for (const values of cartesian(limits.slice(1).map(l => grid(l, Number(options.grid))))) {
    check('four-axis-grid', [0, ...values]);
  }
  for (let a = 1; a < 5; ++a) for (let b = a + 1; b < 5; ++b) {
    for (const av of grid(limits[a], Number(options['pair-grid']))) {
      for (const bv of grid(limits[b], Number(options['pair-grid']))) {
        const q = [0, 0, 0, 0, 0]; q[a] = av; q[b] = bv;
        check(`coupled-J${a + 1}-J${b + 1}`, q);
      }
    }
  }
  let seed = 20261115;
  const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 4294967296;
  };
  for (let k = 0; k < Number(options.random); ++k) {
    check('four-axis-random', [0, ...limits.slice(1).map(([a, b]) => a + (b - a) * random())]);
  }
  report.totalChecked = checked;
  return report;
}

if (process.argv[1] && fileURLToPath(import.meta.url) === resolve(process.argv[1])) {
const { values } = parseArgs({ options: {
  'self-test': { type: 'boolean' }, urdf: { type: 'string' }, out: { type: 'string' },
  'workspace-config': { type: 'string' }, 'export-fcl': { type: 'string' },
  pose: { type: 'string' },
  'baseline-only': { type: 'boolean' }, grid: { type: 'string', default: '5' },
  'pair-grid': { type: 'string', default: '21' }, random: { type: 'string', default: '2000' },
  step: { type: 'string', default: '0.25' },
} });
if (values['self-test']) {
  selfTest();
} else if (values['export-fcl']) {
  assert(values['workspace-config'], '--export-fcl needs --workspace-config');
  const config = JSON.parse(readFileSync(resolve(values['workspace-config']), 'utf8'));
  const model = loadModel(resolve(config.urdf), config.installationRules);
  const directory = resolve(values['export-fcl']);
  const result = model.exportFcl(resolve(directory, 'fcl_meshes'));
  writeFileSync(resolve(directory, 'mesh_input.json'), JSON.stringify(model.input, null, 2)+'\n', {flag:'wx'});
  console.log(JSON.stringify(result));
} else {
  assert(values.urdf, 'Pass --urdf PATH, or --self-test');
  for (const name of ['grid', 'pair-grid']) assert(Number.isInteger(Number(values[name])) && Number(values[name]) >= 2);
  assert(Number.isInteger(Number(values.random)) && Number(values.random) >= 0);
  assert(Number.isFinite(Number(values.step)) && Number(values.step) > 0);
  const started = Date.now();
  const model = loadModel(resolve(values.urdf));
  const report = run(model, values);
  report.elapsedSeconds = (Date.now() - started) / 1000;
  const out = resolve(values.out ?? `build/urdf_collision/scan_${Date.now()}.json`);
  mkdirSync(dirname(out), { recursive: true });
  writeFileSync(out, JSON.stringify(report, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify({ output: out, elapsedSeconds: report.elapsedSeconds,
    baseline: report.baseline, scans: report.scans, pairCounts: report.pairCounts }, null, 2));
}
}
