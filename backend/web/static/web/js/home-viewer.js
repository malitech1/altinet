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
      const distances = prepareModel(object, scene, controls);
      updateSmoothZoom = setupSmoothScrollZoom(
        controls,
        camera,
        renderer.domElement,
        distances,
      );
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

  return { minDistance, maxDistance };
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
