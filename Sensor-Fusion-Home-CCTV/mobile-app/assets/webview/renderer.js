import * as THREE from 'https://unpkg.com/three@0.167.1/build/three.module.js';

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 1, 0.05, 100);
camera.position.set(0, -4, 2);
camera.up.set(0, 0, 1);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
document.body.appendChild(renderer.domElement);

const geometry = new THREE.BufferGeometry();
let capacity = 0;
let positions = new Float32Array(0);
let colors = new Float32Array(0);
let pointSizes = new Float32Array(0);

const pointMaterial = new THREE.ShaderMaterial({
  uniforms: {
    baseSize: { value: 0.035 },
    viewportScale: { value: 1 },
  },
  vertexShader: `
    attribute float pointSize;
    varying vec3 pointColor;
    uniform float baseSize;
    uniform float viewportScale;

    void main() {
      vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
      pointColor = color;
      gl_PointSize = baseSize * pointSize * viewportScale / max(0.1, -mvPosition.z);
      gl_Position = projectionMatrix * mvPosition;
    }
  `,
  fragmentShader: `
    varying vec3 pointColor;

    void main() {
      vec2 centered = gl_PointCoord - vec2(0.5);
      if (dot(centered, centered) > 0.25) discard;
      gl_FragColor = vec4(pointColor, 1.0);
    }
  `,
  vertexColors: true,
});
const pointCloudObject = new THREE.Points(geometry, pointMaterial);
scene.add(pointCloudObject);

const status = document.querySelector('#status');
const alertStatus = document.querySelector('#alert-status');
const showPointCloud = document.querySelector('#show-point-cloud');
const show3d = document.querySelector('#show-3d');
const bboxGroup = new THREE.Group();
scene.add(bboxGroup);
const boxes = new Map();
const mannequins = new Map();

function applyViewOptions() {
  pointCloudObject.visible = showPointCloud.checked;
  bboxGroup.visible = show3d.checked;
  for (const mannequin of mannequins.values()) {
    mannequin.group.visible = show3d.checked;
  }
  localStorage.setItem('show-point-cloud', String(showPointCloud.checked));
  localStorage.setItem('show-3d', String(show3d.checked));
}

showPointCloud.checked = localStorage.getItem('show-point-cloud') !== 'false';
show3d.checked = localStorage.getItem('show-3d') !== 'false';
showPointCloud.addEventListener('change', applyViewOptions);
show3d.addEventListener('change', applyViewOptions);
applyViewOptions();

scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 2.2));
const mannequinGeometries = {
  head: new THREE.SphereGeometry(0.1, 10, 8),
  torso: new THREE.CapsuleGeometry(0.12, 0.26, 4, 8),
  limb: new THREE.CapsuleGeometry(0.045, 0.28, 3, 6),
};

function addMannequinPart(group, geometry, material, position, direction = null) {
  const part = new THREE.Mesh(geometry, material);
  part.position.fromArray(position);
  if (direction) {
    const normalized = new THREE.Vector3(...direction).normalize();
    part.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normalized);
  }
  group.add(part);
}

function createMannequin() {
  const group = new THREE.Group();
  const material = new THREE.MeshLambertMaterial({
    color: 0xff8c00,
    transparent: true,
    opacity: 0.78,
    depthWrite: false,
  });
  addMannequinPart(group, mannequinGeometries.head, material, [0, 0, 0.43]);
  addMannequinPart(group, mannequinGeometries.torso, material, [0, 0, 0.12], [0, 0, 1]);
  addMannequinPart(group, mannequinGeometries.limb, material, [-0.16, 0, 0.1], [-0.35, 0, -0.94]);
  addMannequinPart(group, mannequinGeometries.limb, material, [0.16, 0, 0.1], [0.35, 0, -0.94]);
  addMannequinPart(group, mannequinGeometries.limb, material, [-0.07, 0, -0.3], [-0.12, 0, -0.99]);
  addMannequinPart(group, mannequinGeometries.limb, material, [0.07, 0, -0.3], [0.12, 0, -0.99]);
  group.userData.material = material;
  group.visible = show3d.checked;
  scene.add(group);
  return { group, material, locked: false, standingHeight: 1.7 };
}

function removeMannequin(id) {
  const mannequin = mannequins.get(id);
  if (!mannequin) return;
  scene.remove(mannequin.group);
  mannequin.material.dispose();
  mannequins.delete(id);
}

function updateMannequin(id, center, size, level) {
  let mannequin = mannequins.get(id);
  if (!mannequin) {
    mannequin = createMannequin();
    mannequins.set(id, mannequin);
  }
  const measuredHeight = Math.max(0.2, Number(size[2]) || 1.7);
  if (level === 0 && measuredHeight >= 0.75) mannequin.standingHeight = measuredHeight;
  const height = level > 0 ? mannequin.standingHeight : measuredHeight;
  mannequin.group.position.fromArray(center);
  mannequin.group.scale.setScalar(height / 1.05);
  mannequin.group.rotation.set(0, level === 2 ? Math.PI / 2 : level === 1 ? 0.2 : 0, 0);
  mannequin.material.color.setHex(level === 2 ? 0xff2828 : 0xff8c00);
  mannequin.material.opacity = level === 2 ? 0.92 : 0.72;
  mannequin.locked = mannequin.locked || level === 2;
  return mannequin;
}

function resize() {
  const { clientWidth: width, clientHeight: height } = renderer.domElement;
  renderer.setSize(width, height, false);
  camera.aspect = width / Math.max(height, 1);
  camera.updateProjectionMatrix();
  pointMaterial.uniforms.viewportScale.value = height * renderer.getPixelRatio() * 0.5;
}
new ResizeObserver(resize).observe(document.body);
resize();

function ensureCapacity(pointCount) {
  if (capacity >= pointCount) return;
  capacity = 2 ** Math.ceil(Math.log2(Math.max(pointCount, 1)));
  positions = new Float32Array(capacity * 3);
  colors = new Float32Array(capacity * 3);
  pointSizes = new Float32Array(capacity);

  const positionAttribute = new THREE.BufferAttribute(positions, 3);
  const colorAttribute = new THREE.BufferAttribute(colors, 3);
  const sizeAttribute = new THREE.BufferAttribute(pointSizes, 1);
  positionAttribute.setUsage(THREE.DynamicDrawUsage);
  colorAttribute.setUsage(THREE.DynamicDrawUsage);
  sizeAttribute.setUsage(THREE.DynamicDrawUsage);
  geometry.setAttribute('position', positionAttribute);
  geometry.setAttribute('color', colorAttribute);
  geometry.setAttribute('pointSize', sizeAttribute);
}

function base64ToBytes(encoded) {
  if (encoded instanceof Uint8Array) return encoded;
  if (Array.isArray(encoded)) return Uint8Array.from(encoded);
  const binary = atob(encoded);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }
  return bytes;
}

function findField(message, name) {
  return message.fields.find((field) => field.name === name);
}

function renderSceneCloud(message) {
  const xField = findField(message, 'x');
  const yField = findField(message, 'y');
  const zField = findField(message, 'z');
  const rgbField = findField(message, 'rgb') ?? findField(message, 'rgba');
  if (!xField || !yField || !zField || !rgbField || !message.point_step) {
    status.textContent = 'PointCloud2 fields invalid';
    return;
  }

  const bytes = base64ToBytes(message.data);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !message.is_bigendian;
  const pointCount = message.width * message.height;
  ensureCapacity(pointCount);

  let renderedCount = 0;
  for (let row = 0; row < message.height; row += 1) {
    const rowOffset = row * message.row_step;
    for (let column = 0; column < message.width; column += 1) {
      const offset = rowOffset + column * message.point_step;
      if (offset + message.point_step > view.byteLength) break;
      const x = view.getFloat32(offset + xField.offset, littleEndian);
      const y = view.getFloat32(offset + yField.offset, littleEndian);
      const z = view.getFloat32(offset + zField.offset, littleEndian);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;

      const rgb = view.getUint32(offset + rgbField.offset, littleEndian);
      const positionOffset = renderedCount * 3;
      positions[positionOffset] = x;
      positions[positionOffset + 1] = y;
      positions[positionOffset + 2] = z;
      colors[positionOffset] = ((rgb >> 16) & 0xff) / 255;
      colors[positionOffset + 1] = ((rgb >> 8) & 0xff) / 255;
      colors[positionOffset + 2] = (rgb & 0xff) / 255;

      // Human (#ff8c00) and emergency (#ff2828) points are emphasized.
      const red = (rgb >> 16) & 0xff;
      const green = (rgb >> 8) & 0xff;
      const blue = rgb & 0xff;
      pointSizes[renderedCount] = red === 0xff && (
        (green === 0x8c && blue === 0x00) || (green === 0x28 && blue === 0x28)
      ) ? 3 : 1;
      renderedCount += 1;
    }
  }

  geometry.getAttribute('position').needsUpdate = true;
  geometry.getAttribute('color').needsUpdate = true;
  geometry.getAttribute('pointSize').needsUpdate = true;
  geometry.setDrawRange(0, renderedCount);
  status.textContent = `${renderedCount.toLocaleString()} points`;
}

function vector3(value, fallback = null) {
  if (Array.isArray(value) && value.length >= 3) return value.slice(0, 3).map(Number);
  if (value && ['x', 'y', 'z'].every((key) => key in value)) {
    return [Number(value.x), Number(value.y), Number(value.z)];
  }
  return fallback;
}

function fallLevel(value) {
  if (typeof value === 'number') return value === 2 ? 2 : value === 1 ? 1 : 0;
  if (value && typeof value === 'object') {
    return fallLevel(
      value.level ?? value.state ?? value.state_code ?? value.status ?? value.severity,
    );
  }
  const state = String(value ?? '').toLowerCase();
  if (state === 'resolved' || state === 'recovered' || state === 'normal' || state === 'info') return 0;
  if (
    state.includes('confirm')
    || state.includes('emergency')
    || state.includes('critical')
    || state === 'fall'
    || state === 'fallen'
  ) return 2;
  if (state.includes('candidate') || state.includes('warning') || state.includes('risk')) return 1;
  return 0;
}

function updateSceneMeta(rawMeta) {
  let meta;
  try {
    meta = typeof rawMeta === 'string' ? JSON.parse(rawMeta) : rawMeta;
  } catch (error) {
    console.error('Invalid /viz/scene_meta JSON', error);
    return;
  }

  const entries = meta.tracks ?? meta.bboxes ?? meta.boxes ?? [];
  const alert = meta.alert ?? meta.fall ?? (meta.event_type === 'fall_alert' ? meta : null);
  const rawAlertTrackId = alert?.track_id ?? alert?.id;
  const alertTrackId = rawAlertTrackId == null ? null : String(rawAlertTrackId);
  const alertLevel = fallLevel(alert);
  const alertState = String(alert?.level ?? alert?.state ?? alert?.status ?? '').toLowerCase();
  const alertResolved = alertState === 'resolved' || alertState === 'recovered';
  if (alertTrackId && alertResolved) {
    const mannequin = mannequins.get(alertTrackId);
    if (mannequin) mannequin.locked = false;
  }
  const visibleIds = new Set();
  let highestFallLevel = Math.max(
    fallLevel(meta.fall_state ?? meta.state ?? meta.status),
    alertLevel,
  );

  entries.forEach((entry, index) => {
    const bbox = entry.bbox ?? entry;
    const center = vector3(bbox.center ?? bbox.position);
    const size = vector3(bbox.size ?? bbox.dimensions ?? bbox.scale);
    if (!center || !size) return;

    const id = String(entry.track_id ?? entry.id ?? index);
    let level = Math.max(
      fallLevel(entry.fall_state ?? entry.state ?? entry.status),
      id === alertTrackId ? alertLevel : 0,
    );
    if (mannequins.get(id)?.locked && !(id === alertTrackId && alertResolved)) level = 2;
    highestFallLevel = Math.max(highestFallLevel, level);
    visibleIds.add(id);
    updateMannequin(id, center, size, level);

    let box = boxes.get(id);
    if (!box) {
      const sourceGeometry = new THREE.BoxGeometry(1, 1, 1);
      const edges = new THREE.EdgesGeometry(sourceGeometry);
      sourceGeometry.dispose();
      box = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ transparent: true, opacity: 0.8 }));
      bboxGroup.add(box);
      boxes.set(id, box);
    }
    box.position.fromArray(center);
    box.scale.fromArray(size);
    box.material.color.setHex(level === 2 ? 0xff2828 : level === 1 ? 0xff8c00 : 0xebebeb);
  });

  for (const [id, box] of boxes) {
    if (visibleIds.has(id)) continue;
    bboxGroup.remove(box);
    box.geometry.dispose();
    box.material.dispose();
    boxes.delete(id);
  }

  if (alertTrackId && alertLevel === 2 && !visibleIds.has(alertTrackId)) {
    const center = vector3(alert.position);
    if (center) {
      const previous = mannequins.get(alertTrackId);
      const size = previous ? [0.5, 0.5, previous.group.scale.z * 1.05] : [0.5, 0.5, 1.7];
      updateMannequin(alertTrackId, center, size, 2);
    }
  }

  for (const [id, mannequin] of mannequins) {
    if (!visibleIds.has(id) && !mannequin.locked) removeMannequin(id);
    else if (mannequin.locked) highestFallLevel = 2;
  }

  const fallEntries = meta.falls ?? meta.fall_events ?? meta.emergencies ?? [];
  for (const entry of fallEntries) {
    highestFallLevel = Math.max(
      highestFallLevel,
      fallLevel(entry.state ?? entry.status ?? entry.level ?? entry),
    );
  }
  alertStatus.textContent = highestFallLevel === 2
    ? '낙상 확정'
    : highestFallLevel === 1 ? '낙상 위험' : '';
  alertStatus.dataset.state = String(highestFallLevel);
}

function connectRosbridge(url) {
  const socket = new WebSocket(url);
  socket.addEventListener('open', () => {
    status.textContent = 'ROS 연결됨';
    const qos = {
      history: 'keep_last',
      depth: 1,
      reliability: 'reliable',
      durability: 'transient_local',
    };
    socket.send(JSON.stringify({
      op: 'subscribe',
      id: 'scene-cloud',
      topic: '/viz/scene_cloud',
      type: 'sensor_msgs/msg/PointCloud2',
      throttle_rate: 50,
      queue_length: 1,
      qos,
    }));
    socket.send(JSON.stringify({
      op: 'subscribe',
      id: 'scene-meta',
      topic: '/viz/scene_meta',
      type: 'std_msgs/msg/String',
      queue_length: 1,
      qos,
    }));
  });
  socket.addEventListener('message', (event) => {
    if (typeof event.data !== 'string') return;
    const envelope = JSON.parse(event.data);
    if (envelope.op !== 'publish') return;
    if (envelope.topic === '/viz/scene_cloud') renderSceneCloud(envelope.msg);
    else if (envelope.topic === '/viz/scene_meta') updateSceneMeta(envelope.msg.data);
  });
  socket.addEventListener('close', () => {
    status.textContent = 'ROS 재연결 중...';
    setTimeout(() => connectRosbridge(url), 1500);
  });
  socket.addEventListener('error', () => socket.close());
}

export function renderPointCloud(binary) {
  const source = binary instanceof Float32Array ? binary : new Float32Array(binary);
  if (!source.length || source.length !== source[0] * 3 + 1) return false;
  const count = source[0];
  ensureCapacity(count);
  positions.set(source.subarray(1), 0);
  for (let index = 0; index < count; index += 1) {
    const colorOffset = index * 3;
    colors[colorOffset] = 0x35 / 255;
    colors[colorOffset + 1] = 0xd9 / 255;
    colors[colorOffset + 2] = 1;
    pointSizes[index] = 1;
  }
  geometry.getAttribute('position').needsUpdate = true;
  geometry.getAttribute('color').needsUpdate = true;
  geometry.getAttribute('pointSize').needsUpdate = true;
  geometry.setDrawRange(0, count);
  status.textContent = `${count.toLocaleString()} points`;
  return true;
}
window.renderPointCloud = renderPointCloud;

export async function playRecordedFrames(url, framesPerSecond = 10) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Could not load recording: HTTP ${response.status}`);
  const allFloats = new Float32Array(await response.arrayBuffer());
  const frames = [];
  for (let offset = 0; offset < allFloats.length;) {
    const frameLength = 1 + allFloats[offset] * 3;
    if (!Number.isInteger(frameLength) || offset + frameLength > allFloats.length) {
      throw new Error(`Malformed frame at float offset ${offset}`);
    }
    frames.push(allFloats.subarray(offset, offset + frameLength));
    offset += frameLength;
  }
  let frameIndex = 0;
  setInterval(() => {
    renderPointCloud(frames[frameIndex]);
    frameIndex = (frameIndex + 1) % frames.length;
  }, 1000 / framesPerSecond);
}

export async function playIncidentClip(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Could not load incident clip: HTTP ${response.status}`);
  const clip = await response.json();
  if (!Array.isArray(clip.frames) || clip.frames.length === 0) {
    throw new Error('Incident clip has no frames');
  }

  const firstTime = clip.frames[0].time;
  const playbackStartedAt = performance.now();
  let frameIndex = 0;
  function play(now) {
    const elapsed = (now - playbackStartedAt) / 1000;
    while (
      frameIndex < clip.frames.length
      && clip.frames[frameIndex].time - firstTime <= elapsed
    ) {
      const frame = clip.frames[frameIndex];
      renderSceneCloud(frame.cloud);
      if (frame.meta) updateSceneMeta(frame.meta);
      frameIndex += 1;
    }
    if (frameIndex < clip.frames.length) requestAnimationFrame(play);
    else status.textContent += ' | 클립 재생 완료';
  }
  requestAnimationFrame(play);
}

const query = new URLSearchParams(location.search);
const rosbridgeUrl = query.get('rosbridge') ?? query.get('ws');
const incidentClipUrl = query.get('clip');
if (incidentClipUrl) {
  playIncidentClip(incidentClipUrl).catch((error) => {
    console.error(error);
    status.textContent = '사고 클립 로드 실패';
  });
} else if (rosbridgeUrl) {
  connectRosbridge(rosbridgeUrl);
} else {
  const recordingUrl = query.get('recording')
    ?? new URL('../../../data/pointcloud_frames.bin', import.meta.url).href;
  playRecordedFrames(recordingUrl).catch((error) => {
    console.error(error);
    status.textContent = '녹화 파일 로드 실패';
  });
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();

