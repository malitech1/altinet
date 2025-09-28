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

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
  const keyLight = new THREE.DirectionalLight(0xffffff, 0.8);
  keyLight.position.set(5, 10, 7);
  const fillLight = new THREE.DirectionalLight(0xe8f0ff, 0.4);
  fillLight.position.set(-6, 4, -4);

  scene.add(ambientLight, keyLight, fillLight);

  const groundGeometry = new THREE.CircleGeometry(6, 64);
  const groundMaterial = new THREE.MeshStandardMaterial({ color: 0xdfe6f3 });
  const ground = new THREE.Mesh(groundGeometry, groundMaterial);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -0.001;
  scene.add(ground);

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
      prepareModel(object, scene, controls);
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
  const target = new THREE.Vector3();
  const box = new THREE.Box3().setFromObject(model);
  box.getCenter(target);
  const size = new THREE.Vector3();
  box.getSize(size);

  model.position.sub(target);
  model.position.y -= box.min.y;

  scene.add(model);

  controls.target.set(0, size.y * 0.45, 0);
  controls.update();
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

    if (name.toLowerCase().includes('wall')) {
      mesh.rotation.x = -Math.PI / 2;
    }

    root.add(mesh);
  }

  return root;
}
