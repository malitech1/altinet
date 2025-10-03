import * as THREE from 'https://unpkg.com/three@0.160.0/build/three.module.js?module';
import { OrbitControls } from 'https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js?module';

const container = document.getElementById('home-viewer');
if (!container) {
  console.warn('Altinet home viewer container not found.');
}

const objUrl = container?.dataset.objUrl;
if (!container || !objUrl) {
  console.error('Missing viewer container or model URL.');
} else {
  initialiseViewer(container, objUrl);
}

function initialiseViewer(containerEl, objUrl) {
  const loadingOverlay = containerEl.querySelector('[data-role="loading"]');
  const errorOverlay = containerEl.querySelector('[data-role="error"]');

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf4f6fb);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  const initialWidth = containerEl.clientWidth || containerEl.offsetWidth || 640;
  const initialHeight = containerEl.clientHeight || containerEl.offsetHeight || 480;
  renderer.setSize(initialWidth, initialHeight);
  containerEl.appendChild(renderer.domElement);

  const camera = new THREE.PerspectiveCamera(
    45,
    initialWidth / initialHeight,
    0.1,
    100,
  );
  camera.position.set(3.5, 2.8, 3.2);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.screenSpacePanning = false;
  controls.maxPolarAngle = Math.PI / 2.1;

  let updateSmoothZoom = () => {};
  let updateRoomAnnotations = () => {};

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
  const keyLight = new THREE.DirectionalLight(0xffffff, 0.8);
  keyLight.position.set(5, 10, 7);
  const fillLight = new THREE.DirectionalLight(0xe8f0ff, 0.4);
  fillLight.position.set(-6, 4, -4);

  scene.add(ambientLight, keyLight, fillLight);

  const wallMaterial = new THREE.MeshStandardMaterial({
    color: 0xd4dae4,
    roughness: 0.6,
    metalness: 0.05,
  });

  const roofMaterial = new THREE.MeshStandardMaterial({
    color: 0xb34747,
    roughness: 0.4,
    metalness: 0.1,
  });

  loadObjModel(objUrl, { wallMaterial, roofMaterial })
    .then((object) => {
      const { distances, bounds } = prepareModel(object, scene, controls);
      updateSmoothZoom = setupSmoothScrollZoom(
        controls,
        camera,
        renderer.domElement,
        distances,
      );
      updateRoomAnnotations = createRoomAnnotations({
        containerEl,
        camera,
        renderer,
        bounds,
      });
      animate();
    })
    .catch((error) => {
      console.error('Unable to load OBJ model', error);
      if (errorOverlay) {
        errorOverlay.classList.remove('d-none');
      }
    })
    .finally(() => {
      if (loadingOverlay) {
        loadingOverlay.classList.add('d-none');
      }
    });

  function animate() {
    requestAnimationFrame(animate);
    updateSmoothZoom();
    updateRoomAnnotations();
    controls.update();
    renderer.render(scene, camera);
  }

  window.addEventListener('resize', () => {
    const width = containerEl.clientWidth || containerEl.offsetWidth || initialWidth;
    const height = containerEl.clientHeight || containerEl.offsetHeight || initialHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  });
}

function prepareModel(model, scene, controls) {
  // Rotate the model so it lies flat instead of standing upright. The OBJ
  // geometry is authored with Z as the up axis, whereas the Three.js scene
  // uses Y for the vertical axis. Rotating it by -90° around X brings the
  // floor plane into alignment with the viewer's horizontal plane.
  model.rotation.x = -Math.PI / 2;

  const target = new THREE.Vector3();
  const box = new THREE.Box3().setFromObject(model);
  box.getCenter(target);
  const size = new THREE.Vector3();
  box.getSize(size);

  const diagonal = size.length();
  const minDistance = Math.max(diagonal * 0.35 || 0, 0.75);
  const maxDistance = Math.max(diagonal * 4 || 0, minDistance * 2);

  model.position.sub(target);
  model.position.y -= box.min.y;

  scene.add(model);

  controls.target.set(0, size.y * 0.45, 0);
  controls.minDistance = minDistance;
  controls.maxDistance = maxDistance;
  controls.update();

  const adjustedBounds = new THREE.Box3().setFromObject(model);

  return { distances: { minDistance, maxDistance }, bounds: adjustedBounds };
}

function setupSmoothScrollZoom(controls, camera, domElement, distances) {
  const target = controls.target;
  const direction = new THREE.Vector3();
  const newPosition = new THREE.Vector3();

  const minDistance = distances?.minDistance ?? controls.minDistance ?? 0.5;
  const maxDistance = distances?.maxDistance ?? controls.maxDistance ?? 50;

  let targetDistance = THREE.MathUtils.clamp(
    camera.position.distanceTo(target),
    minDistance,
    maxDistance,
  );

  controls.enableZoom = false;

  if (domElement.style.touchAction !== 'none') {
    domElement.style.touchAction = 'none';
  }

  const baseControlState = {
    enablePan: controls.enablePan,
    enableRotate: controls.enableRotate,
  };

  const handleWheel = (event) => {
    if (event.defaultPrevented) {
      return;
    }
    event.preventDefault();

    const delta = event.deltaY;
    if (delta === 0) {
      return;
    }

    const magnitude = Math.min(1, Math.abs(delta) / 120);
    const factor = 1 + Math.sign(delta) * magnitude * 0.25;
    targetDistance = THREE.MathUtils.clamp(
      targetDistance * factor,
      minDistance,
      maxDistance,
    );
  };

  domElement.addEventListener('wheel', handleWheel, { passive: false });

  // Track pinch gestures so mobile users can zoom the model.
  const touchState = {
    active: false,
    initialDistance: 0,
    initialTargetDistance: targetDistance,
  };

  const distanceBetweenTouches = (touches) => {
    if (touches.length < 2) {
      return 0;
    }
    const [first, second] = touches;
    const dx = second.pageX - first.pageX;
    const dy = second.pageY - first.pageY;
    return Math.hypot(dx, dy);
  };

  const handleTouchStart = (event) => {
    event.stopPropagation();

    if (event.touches.length !== 2) {
      return;
    }
    const distance = distanceBetweenTouches(event.touches);
    if (distance <= 0) {
      return;
    }

    event.preventDefault();

    controls.enableRotate = false;
    controls.enablePan = false;

    touchState.active = true;
    touchState.initialDistance = distance;
    touchState.initialTargetDistance = targetDistance;
  };

  const handleTouchMove = (event) => {
    event.stopPropagation();

    if (!touchState.active) {
      return;
    }
    if (event.touches.length !== 2) {
      return;
    }

    event.preventDefault();

    const distance = distanceBetweenTouches(event.touches);
    if (distance <= 0 || touchState.initialDistance <= 0) {
      return;
    }

    const scale = distance / touchState.initialDistance;
    const desiredDistance = touchState.initialTargetDistance * Math.max(scale, 1e-4);
    targetDistance = THREE.MathUtils.clamp(
      desiredDistance,
      minDistance,
      maxDistance,
    );
  };

  const endTouchInteraction = () => {
    touchState.active = false;
    touchState.initialDistance = 0;
    touchState.initialTargetDistance = targetDistance;
    controls.enableRotate = baseControlState.enableRotate;
    controls.enablePan = baseControlState.enablePan;
  };

  const handleTouchEnd = (event) => {
    event.stopPropagation();

    if (event.touches.length === 2) {
      // Another finger lifted but two touches remain; refresh the baseline.
      touchState.initialDistance = distanceBetweenTouches(event.touches);
      touchState.initialTargetDistance = targetDistance;
      return;
    }

    if (touchState.active) {
      endTouchInteraction();
    }
  };

  domElement.addEventListener('touchstart', handleTouchStart, { passive: false });
  domElement.addEventListener('touchmove', handleTouchMove, { passive: false });
  domElement.addEventListener('touchend', handleTouchEnd, { passive: true });
  domElement.addEventListener('touchcancel', handleTouchEnd, { passive: true });

  controls.addEventListener('end', () => {
    targetDistance = THREE.MathUtils.clamp(
      camera.position.distanceTo(target),
      minDistance,
      maxDistance,
    );
  });

  return () => {
    const currentDistance = camera.position.distanceTo(target);
    const difference = targetDistance - currentDistance;

    if (Math.abs(difference) < 1e-4) {
      return;
    }

    const smoothStep = difference * 0.12;
    const nextDistance = currentDistance + smoothStep;

    direction.subVectors(camera.position, target).normalize();
    newPosition.copy(direction).multiplyScalar(nextDistance).add(target);
    camera.position.copy(newPosition);
  };
}

function createRoomAnnotations({ containerEl, camera, renderer, bounds }) {
  const overlay = document.createElement('div');
  overlay.className = 'viewer-annotations';
  containerEl.appendChild(overlay);

  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.classList.add('viewer-annotations__canvas');
  svg.setAttribute('role', 'presentation');
  overlay.appendChild(svg);

  const roomDefinitions = [
    {
      name: 'Living Room',
      relativePosition: { x: 0.78, y: 0.15, z: 0.36 },
      anchor: 'right',
      metrics: {
        temperature: '21.5°C',
        occupants: '2 people',
        brightness: '340 lux',
        noise: '38 dBA',
      },
    },
    {
      name: 'Kitchen',
      relativePosition: { x: 0.28, y: 0.18, z: 0.35 },
      anchor: 'left',
      metrics: {
        temperature: '23.1°C',
        occupants: '1 person',
        brightness: '420 lux',
        noise: '41 dBA',
      },
    },
    {
      name: 'Bedroom',
      relativePosition: { x: 0.32, y: 0.18, z: 0.74 },
      anchor: 'left',
      metrics: {
        temperature: '20.2°C',
        occupants: '0 people',
        brightness: '120 lux',
        noise: '30 dBA',
      },
    },
  ];

  const lerp = THREE.MathUtils.lerp;
  const toWorldPosition = (relative) => {
    const x = lerp(bounds.min.x, bounds.max.x, relative.x);
    const y = lerp(bounds.min.y, bounds.max.y, relative.y ?? 0.5);
    const z = lerp(bounds.min.z, bounds.max.z, relative.z);
    return new THREE.Vector3(x, y, z);
  };

  const renderCard = (room) => {
    const metrics = room.metrics;
    return `
      <h3 class="room-info-card__title">${room.name}</h3>
      <dl class="room-info-card__metrics">
        <div class="room-info-card__metric">
          <dt>Temp</dt>
          <dd>${metrics.temperature}</dd>
        </div>
        <div class="room-info-card__metric">
          <dt>People</dt>
          <dd>${metrics.occupants}</dd>
        </div>
        <div class="room-info-card__metric">
          <dt>Brightness</dt>
          <dd>${metrics.brightness}</dd>
        </div>
        <div class="room-info-card__metric">
          <dt>Ambient noise</dt>
          <dd>${metrics.noise}</dd>
        </div>
      </dl>
    `;
  };

  const entries = roomDefinitions.map((room) => {
    const card = document.createElement('article');
    card.className = 'room-info-card';
    card.innerHTML = renderCard(room);
    overlay.appendChild(card);

    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.classList.add('room-info-connector');
    line.setAttribute('stroke', 'rgba(0, 0, 0, 0.5)');
    line.setAttribute('stroke-width', '1.25');
    line.setAttribute('vector-effect', 'non-scaling-stroke');
    svg.appendChild(line);

    return {
      room,
      card,
      line,
      worldPosition: toWorldPosition(room.relativePosition),
    };
  });

  const rendererSize = new THREE.Vector2();
  const projectedPosition = new THREE.Vector3();

  const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

  const preventCollisions = (items, containerHeight, margin, spacing) => {
    const groups = {
      left: [],
      right: [],
    };

    for (const item of items) {
      groups[item.anchor]?.push(item);
    }

    const enforceGroup = (group) => {
      if (group.length === 0) {
        return;
      }

      group.sort((a, b) => a.centerY - b.centerY);

      let previous = null;
      for (const item of group) {
        const minCenter = margin + item.halfHeight;
        const maxCenter = containerHeight - margin - item.halfHeight;
        let center = clamp(item.centerY, minCenter, maxCenter);
        if (previous) {
          const minAllowed =
            previous.centerY + previous.halfHeight + item.halfHeight + spacing;
          if (center < minAllowed) {
            center = minAllowed;
          }
        }
        item.centerY = clamp(center, minCenter, maxCenter);
        previous = item;
      }

      let next = null;
      for (let index = group.length - 1; index >= 0; index -= 1) {
        const item = group[index];
        const minCenter = margin + item.halfHeight;
        const maxCenter = containerHeight - margin - item.halfHeight;
        let center = clamp(item.centerY, minCenter, maxCenter);
        if (next) {
          const maxAllowed =
            next.centerY - (next.halfHeight + item.halfHeight + spacing);
          if (center > maxAllowed) {
            center = maxAllowed;
          }
        }
        item.centerY = clamp(center, minCenter, maxCenter);
        next = item;
      }

      previous = null;
      for (const item of group) {
        const minCenter = margin + item.halfHeight;
        const maxCenter = containerHeight - margin - item.halfHeight;
        let center = clamp(item.centerY, minCenter, maxCenter);
        if (previous) {
          const minAllowed =
            previous.centerY + previous.halfHeight + item.halfHeight + spacing;
          if (center < minAllowed) {
            center = minAllowed;
          }
        }
        item.centerY = clamp(center, minCenter, maxCenter);
        previous = item;
      }
    };

    enforceGroup(groups.left);
    enforceGroup(groups.right);
  };

  const update = () => {
    renderer.getSize(rendererSize);
    const width = rendererSize.x;
    const height = rendererSize.y;

    overlay.style.width = `${width}px`;
    overlay.style.height = `${height}px`;
    svg.setAttribute('width', `${width}`);
    svg.setAttribute('height', `${height}`);
    svg.setAttribute('viewBox', `0 0 ${width} ${height}`);

    const margin = 12;
    const spacing = 12;
    const defaultHorizontalOffset = 140;

    const layoutEntries = [];


    for (const entry of entries) {
      projectedPosition.copy(entry.worldPosition).project(camera);

      const visible =
        projectedPosition.z > -1 &&
        projectedPosition.z < 1 &&
        Math.abs(projectedPosition.x) <= 1.2 &&
        Math.abs(projectedPosition.y) <= 1.2;

      if (!visible) {
        entry.card.style.opacity = '0';
        entry.card.style.pointerEvents = 'none';
        entry.line.setAttribute('opacity', '0');
        continue;
      }

      entry.card.style.opacity = '';
      entry.card.style.pointerEvents = '';
      entry.line.setAttribute('opacity', '1');

      const anchor = entry.room.anchor ?? (projectedPosition.x < 0 ? 'left' : 'right');
      entry.card.classList.toggle('room-info-card--left', anchor === 'left');
      entry.card.classList.toggle('room-info-card--right', anchor === 'right');

      const x = (projectedPosition.x * 0.5 + 0.5) * width;
      const y = (-projectedPosition.y * 0.5 + 0.5) * height;

      const cardWidth = entry.card.offsetWidth || 180;
      const cardHeight = entry.card.offsetHeight || 120;
      const halfHeight = cardHeight / 2;

      const verticalOffset = entry.room.verticalOffset ?? 0;
      let connectionY = y + verticalOffset;
      let connectionX =
        anchor === 'right'
          ? x + (entry.room.horizontalOffset ?? defaultHorizontalOffset)
          : x - (entry.room.horizontalOffset ?? defaultHorizontalOffset);


      if (anchor === 'right') {
        connectionX = clamp(connectionX, margin, width - margin - cardWidth);
      } else {
        connectionX = clamp(connectionX, margin + cardWidth, width - margin);
      }

      layoutEntries.push({
        entry,
        anchor,
        connectionX,
        centerY: clamp(
          connectionY,
          margin + halfHeight,
          height - margin - halfHeight,
        ),
        halfHeight,
        lineStart: { x, y },
      });

      entry.line.setAttribute('x1', `${x}`);
      entry.line.setAttribute('y1', `${y}`);
    }

    preventCollisions(layoutEntries, height, margin, spacing);

    for (const layout of layoutEntries) {
      const { entry, connectionX, centerY, lineStart } = layout;

      entry.card.style.left = `${connectionX}px`;
      entry.card.style.top = `${centerY}px`;

      entry.line.setAttribute('x1', `${lineStart.x}`);
      entry.line.setAttribute('y1', `${lineStart.y}`);
      entry.line.setAttribute('x2', `${connectionX}`);
      entry.line.setAttribute('y2', `${centerY}`);
    }
  };

  update();
  return update;
}

async function loadObjModel(url, materials) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to fetch OBJ: ${response.status}`);
  }
  const objText = await response.text();
  const parsed = parseObj(objText);
  return buildMeshGroup(parsed, materials);
}

function parseObj(text) {
  const vertexPositions = [];
  const vertexNormals = [];
  const groups = new Map();
  let currentGroup = 'Structure';

  const ensureGroup = (name) => {
    if (!groups.has(name)) {
      groups.set(name, { positions: [], normals: [] });
    }
    return groups.get(name);
  };

  const lines = text.split(/\r?\n/);
  for (const rawLine of lines) {
    const line = rawLine.trim();
    if (line === '' || line.startsWith('#')) {
      continue;
    }
    const parts = line.split(/\s+/);
    const keyword = parts[0];

    switch (keyword) {
      case 'v': {
        vertexPositions.push(parts.slice(1).map(Number));
        break;
      }
      case 'vn': {
        vertexNormals.push(parts.slice(1).map(Number));
        break;
      }
      case 'g': {
        currentGroup = parts[1] ?? currentGroup;
        ensureGroup(currentGroup);
        break;
      }
      case 'f': {
        const faceVertices = parts.slice(1).map((chunk) => {
          const [vIdx, , nIdx] = chunk.split('/');
          return {
            vertexIndex: parseInt(vIdx, 10) - 1,
            normalIndex: nIdx ? parseInt(nIdx, 10) - 1 : null,
          };
        });

        const groupData = ensureGroup(currentGroup);

        for (let i = 1; i < faceVertices.length - 1; i += 1) {
          const triangle = [faceVertices[0], faceVertices[i], faceVertices[i + 1]];
          for (const corner of triangle) {
            const vertex = vertexPositions[corner.vertexIndex];
            if (!vertex) {
              throw new Error(`Vertex index out of range: ${corner.vertexIndex}`);
            }
            groupData.positions.push(...vertex);

            const normal =
              (corner.normalIndex !== null && vertexNormals[corner.normalIndex]) || null;
            if (normal) {
              groupData.normals.push(...normal);
            }
          }
        }
        break;
      }
      default:
        // Ignore other keywords (mtllib, usemtl, etc.).
        break;
    }
  }

  return groups;
}

function buildMeshGroup(groups, materials) {
  const root = new THREE.Group();

  for (const [name, data] of groups.entries()) {
    if (!data.positions.length) {
      continue;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(data.positions, 3),
    );

    if (data.normals.length === data.positions.length) {
      geometry.setAttribute(
        'normal',
        new THREE.Float32BufferAttribute(data.normals, 3),
      );
    } else {
      geometry.computeVertexNormals();
    }

    const material = name.toLowerCase().includes('roof')
      ? materials.roofMaterial
      : materials.wallMaterial;

    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = name;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    root.add(mesh);
  }

  return root;
}
