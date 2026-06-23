const state = {
  j1: 0,
  j2: 0,
  j3: 0,
  laserActive: false,
  laserPower: 0,
  emergencyStop: false,
  executingTrajectory: false
};

const checkConveyor = document.getElementById('check-conveyor');

const conveyorState = {
  active: false,
  positions: {
    'cylinder_A1': 0.0,
    'cylinder_D': -0.375,
    'cylinder_R': -0.75,
    'cylinder_I': -1.125,
    'cylinder_A2': -1.50,
    'cylinder_N': -1.875,
    'cylinder_A3': -2.25
  }
};

let letters = {
  'cylinder_A1': 'A',
  'cylinder_D': 'D',
  'cylinder_R': 'R',
  'cylinder_I': 'I',
  'cylinder_A2': 'A',
  'cylinder_N': 'N',
  'cylinder_A3': 'A'
};

let conveyorAnimationId = null;
let lastConveyorTime = null;

function animateConveyor(now) {
  if (!conveyorState.active) {
    conveyorAnimationId = null;
    lastConveyorTime = null;
    return;
  }
  
  if (!lastConveyorTime) {
    lastConveyorTime = now;
  }
  const dt = (now - lastConveyorTime) / 1000.0;
  lastConveyorTime = now;
  
  const velocity = 0.04;
  Object.keys(conveyorState.positions).forEach(name => {
    let y = conveyorState.positions[name];
    y += velocity * dt;
    if (y > 0.75) {
      y -= 2.625;
    }
    conveyorState.positions[name] = y;
  });
  
  drawRobot();
  if (typeof drawPhysRobot === 'function') {
    drawPhysRobot();
  }
  
  conveyorAnimationId = requestAnimationFrame(animateConveyor);
}

const d1 = 60.0;
const d2 = 36.0;
const a2 = 15.0;
const L1 = 146.0;
const L2 = 161.0;
const d4 = -7.0;
const phi2 = 0.0;
const phi3 = 0.0;
const laserLength = 0.0;

const S = 0.13;
const y_base = 155;

const jointsConfig = {
  1: { min: -90.0, max: 90.0, name: 'j1' },
  2: { min: -10.0, max: 90.0, name: 'j2' },
  3: { min: -120.0, max: 10.0, name: 'j3' }
};

let ikAnimationId = null;

const laserPowerSlider = document.getElementById('input-range-laser');
const laserPowerValText = document.getElementById('laser-power-val');
const laserIndicator = document.getElementById('laser-indicator');
const btnLaserToggle = document.getElementById('btn-laser-toggle');

const btnEmergency = document.getElementById('btn-emergency');
const systemStatusDot = document.getElementById('system-status-dot');
const systemStatusText = document.getElementById('system-status-text');

const canvas = document.getElementById('robot-canvas');
const ctx = canvas.getContext('2d');
const checkShowWorkspace = document.getElementById('check-show-workspace');

let workspaceRMin = 30.0;
let workspaceRMax = 322.0;

const btnApplyIk = document.getElementById('btn-apply-ik');
const ikStatus = document.getElementById('ik-status');

let lastSentTime = 0;
const sendIntervalMs = 50;
let rosSendTimeout = null;

/*
 * Envía las posiciones angulares de las articulaciones en tiempo real al puente de comunicación 
 * de ROS 2 a través de una petición HTTP POST throttleada para no sobrecargar la red.
 */
function sendJointsToROS(j1, j2, j3) {
  const now = Date.now();
  
  if (rosSendTimeout) {
    clearTimeout(rosSendTimeout);
    rosSendTimeout = null;
  }

  const executeSend = () => {
    lastSentTime = Date.now();
    
    // Aplicamos los desfasajes exactos para que el brazo fisico en Gazebo
    // se alinee con la cinematica horizontal recta de la interfaz de usuario.
    const j2_val = j2 - 59.017;
    const j3_val = j3 + 70.0;

    const j2_ros = Math.max(-110.0, Math.min(110.0, j2_val));
    const j3_ros = Math.max(-110.0, Math.min(70.0, j3_val));

    fetch('/api/move', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ j1, j2: j2_ros, j3: j3_ros })
    }).catch(err => console.error("Error enviando articulaciones a ROS:", err));
  };

  if (now - lastSentTime >= sendIntervalMs) {
    executeSend();
  } else {
    const remaining = sendIntervalMs - (now - lastSentTime);
    rosSendTimeout = setTimeout(executeSend, remaining);
  }
}

/*
 * Calcula la cinemática directa del manipulador a partir de las variables articulares,
 * actualiza las coordenadas mostradas en la interfaz y comanda el movimiento en ROS 2.
 */
function updateKinematics() {
  const theta1 = state.j1 * Math.PI / 180;
  const theta2 = state.j2 * Math.PI / 180;
  const theta3 = state.j3 * Math.PI / 180;

  const t2_star = theta2;
  const t3_star = theta3;

  const x_plane = a2 + L1 * Math.cos(t2_star) + L2 * Math.cos(t2_star + t3_star);
  const z_plane = (d1 + d2) + L1 * Math.sin(t2_star) + L2 * Math.sin(t2_star + t3_star);

  const x = x_plane * Math.cos(theta1) + d4 * Math.sin(theta1);
  const y = x_plane * Math.sin(theta1) - d4 * Math.cos(theta1);
  const z = z_plane;

  document.getElementById('coord-x').textContent = `${x.toFixed(1)} mm`;
  document.getElementById('coord-y').textContent = `${y.toFixed(1)} mm`;
  document.getElementById('coord-z').textContent = `${z.toFixed(1)} mm`;

  if (!state.executingTrajectory) {
    sendJointsToROS(state.j1, state.j2, state.j3);
  }

  return { x_plane, z_plane, x, y, z };
}

/*
 * Resuelve el modelo cinemático inverso para determinar las posiciones de consigna
 * de los servomotores a partir del objetivo cartesiano (x, y, z) deseado.
 */
function solveIK(x, y, z, customL2) {
  const targetX = parseFloat(x);
  const targetY = parseFloat(y);
  const targetZ = parseFloat(z);
  
  const currentL2 = (customL2 !== undefined) ? customL2 : L2;

  if (isNaN(targetX) || isNaN(targetY) || isNaN(targetZ)) {
    return { error: 'VALORES_INVALIDOS', msg: 'Ingresa valores numéricos válidos.' };
  }

  let r = Math.sqrt(targetX * targetX + targetY * targetY);
  if (r < Math.abs(d4)) {
    r = Math.abs(d4);
  }
  const Rp = Math.sqrt(r * r - d4 * d4);
  const theta1 = (targetX === 0 && targetY === 0) ? -Math.PI / 2 : Math.atan2(targetY, targetX) - Math.atan2(-d4, Rp);
  const j1 = theta1 * 180 / Math.PI;

  if (j1 < jointsConfig[1].min || j1 > jointsConfig[1].max) {
    return { error: 'FUERA DE ALCANCE', msg: 'Rotación base J1 fuera de límites (-165° a 165°).' };
  }

  const xc = Rp - a2;
  const zc = targetZ - (d1 + d2);

  const psi_numerator = xc * xc + zc * zc - L1 * L1 - currentL2 * currentL2;
  const psi_denominator = 2.0 * L1 * currentL2;
  let psi = psi_numerator / psi_denominator;

  if (psi > 1.0 && psi <= 1.005) {
    psi = 1.0;
  } else if (psi < -1.0 && psi >= -1.005) {
    psi = -1.0;
  }

  if (psi < -1.0 || psi > 1.0) {
    return { error: 'FUERA DE ALCANCE', msg: 'Coordenadas fuera del alcance del brazo.' };
  }

  const solutions = [-1, 1];
  for (let s of solutions) {
    const sinTheta3_star = s * Math.sqrt(1.0 - psi * psi);
    const theta3_star = Math.atan2(sinTheta3_star, psi);

    const theta2_star = Math.atan2(zc, xc) - Math.atan2(currentL2 * sinTheta3_star, L1 + currentL2 * psi);

    const theta2 = theta2_star;
    const theta3 = theta3_star;

    const j2 = theta2 * 180 / Math.PI;
    const j3 = theta3 * 180 / Math.PI;

    if (j2 >= jointsConfig[2].min && j2 <= jointsConfig[2].max &&
        j3 >= jointsConfig[3].min && j3 <= jointsConfig[3].max) {
      return { j1, j2, j3 };
    }
  }

  return { error: 'FUERA DE ALCANCE', msg: 'Posición cinemáticamente inalcanzable por límites de articulación.' };
}

/*
 * Genera una transición interpolada suave (Cubic Ease-Out) para mover las articulaciones
 * desde su pose actual hasta la pose resuelta por la cinemática inversa.
 */
function animateToJoints(targetJ1, targetJ2, targetJ3) {
  if (ikAnimationId) {
    cancelAnimationFrame(ikAnimationId);
  }

  const startJ1 = state.j1;
  const startJ2 = state.j2;
  const startJ3 = state.j3;

  const duration = 600;
  const startTime = performance.now();

  function step(now) {
    const elapsed = now - startTime;
    const progress = Math.min(elapsed / duration, 1);
    const ease = 1 - Math.pow(1 - progress, 3);

    state.j1 = startJ1 + (targetJ1 - startJ1) * ease;
    state.j2 = startJ2 + (targetJ2 - startJ2) * ease;
    state.j3 = startJ3 + (targetJ3 - startJ3) * ease;

    for (let num = 1; num <= 3; num++) {
      const valName = jointsConfig[num].name;
      document.getElementById(`input-range-j${num}`).value = state[valName];
      document.getElementById(`input-number-j${num}`).value = state[valName].toFixed(1);
    }

    updateKinematics();
    drawRobot();

    if (progress < 1) {
      ikAnimationId = requestAnimationFrame(step);
    } else {
      ikAnimationId = null;
    }
  }

  ikAnimationId = requestAnimationFrame(step);
}

// Precalcular límites del espacio de trabajo en vista superior basados en configuraciones sin colisión
function precomputeWorkspaceLimits() {
  let rMin = Infinity;
  let rMax = -Infinity;
  for (let j2 = -50.9; j2 <= 169.0; j2 += 2) {
    for (let j3 = -150.0; j3 <= 0.0; j3 += 2) {
      if (!checkCollision(j2, j3)) {
        const theta2 = j2 * Math.PI / 180;
        const theta3 = j3 * Math.PI / 180;
        const x_elbow = a2 + L1 * Math.cos(theta2);
        const x_tip = x_elbow + L2 * Math.cos(theta2 + theta3);
        const r = Math.sqrt(x_tip * x_tip + d4 * d4);
        if (r < rMin) rMin = r;
        if (r > rMax) rMax = r;
      }
    }
  }
  if (rMin !== Infinity) workspaceRMin = rMin;
  if (rMax !== -Infinity) workspaceRMax = rMax;
}

// Dibuja el contorno del espacio de trabajo en la vista lateral (X-Z)
function drawWorkspaceLateral(customCtx) {
  const c = customCtx || ctx;
  c.save();
  c.fillStyle = 'rgba(255, 115, 0, 0.05)';
  c.strokeStyle = 'rgba(255, 115, 0, 0.2)';
  c.lineWidth = 1;
  c.setLineDash([2, 3]);

  const points = [];
  const step = 4; // grados
  const x_base_left = 60;
  const y1 = y_base - (d1 + d2) * S;

  const addPoint = (j2, j3) => {
    const t2 = j2 * Math.PI / 180;
    const t3 = j3 * Math.PI / 180;
    const x2 = x_base_left + (L1 * Math.cos(t2)) * S;
    const y2 = y1 - (L1 * Math.sin(t2)) * S;
    const x4 = x2 + (L2 * Math.cos(t2 + t3)) * S;
    const y4 = y2 - (L2 * Math.sin(t2 + t3)) * S;
    
    // Limitar al nivel de la mesa
    const y_clamped = Math.min(y4, y_base);
    points.push({ x: x4, y: y_clamped });
  };

  // 1. Barrer J2 de -50.9 a 169.0 con J3 = 0
  for (let j2 = -50.9; j2 <= 169.0; j2 += step) {
    addPoint(j2, 0);
  }
  addPoint(169.0, 0);

  // 2. Barrer J3 de 0 a -150 con J2 = 169.0
  for (let j3 = 0; j3 >= -150.0; j3 -= step) {
    addPoint(169.0, j3);
  }
  addPoint(169.0, -150.0);

  // 3. Barrer J2 de 169.0 a -50.9 con J3 = -150
  for (let j2 = 169.0; j2 >= -50.9; j2 -= step) {
    addPoint(j2, -150.0);
  }
  addPoint(-50.9, -150.0);

  // 4. Barrer J3 de -150 a 0 con J2 = -50.9
  for (let j3 = -150.0; j3 <= 0; j3 += step) {
    addPoint(-50.9, j3);
  }
  addPoint(-50.9, 0);

  if (points.length > 0) {
    c.beginPath();
    c.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) {
      c.lineTo(points[i].x, points[i].y);
    }
    c.closePath();
    c.fill();
    c.stroke();
  }

  c.restore();
}

// Dibuja el contorno del espacio de trabajo en la vista superior (X-Y)
function drawWorkspaceTop(customCtx) {
  const c = customCtx || ctx;
  c.save();
  c.fillStyle = 'rgba(255, 115, 0, 0.05)';
  c.strokeStyle = 'rgba(255, 115, 0, 0.2)';
  c.lineWidth = 1;
  c.setLineDash([2, 3]);

  const x_base_right = 330;
  const y_base_right = 90;
  const r165 = 165 * Math.PI / 180;

  c.beginPath();
  // Arco exterior
  c.arc(x_base_right, y_base_right, workspaceRMax * S, -r165, r165);
  // Línea radial
  c.lineTo(
    x_base_right + workspaceRMin * S * Math.cos(r165),
    y_base_right + workspaceRMin * S * Math.sin(r165)
  );
  // Arco interior
  c.arc(x_base_right, y_base_right, workspaceRMin * S, r165, -r165, true);
  c.closePath();
  c.fill();
  c.stroke();

  c.restore();
}

/*
 * Renderiza el esquema dinámico interactivo en el lienzo (vista lateral X-Z y vista superior X-Y),
 * proyectando también la trayectoria e interacción del haz láser de corte.
 */
function drawRobot() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
  ctx.lineWidth = 1;
  const gridSize = 20;
  for (let x = 0; x < canvas.width; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 0; y < canvas.height; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.lineWidth = 1;
  ctx.setLineDash([2, 4]);
  ctx.beginPath();
  ctx.moveTo(220, 0);
  ctx.lineTo(220, canvas.height);
  ctx.stroke();
  ctx.setLineDash([]);

  // Dibujar contornos del espacio de trabajo si el toggle está activo
  const showWorkspace = checkShowWorkspace ? checkShowWorkspace.checked : true;
  if (showWorkspace) {
    drawWorkspaceLateral();
    drawWorkspaceTop();
  }

  const t2 = state.j2 * Math.PI / 180;
  const t3 = state.j3 * Math.PI / 180;
  // Para la visualización en la UI, no aplicamos los desfasajes phi2 y phi3,
  // de modo que 0° y 0° se muestren alineados en horizontal.
  const t2_star = t2;
  const t3_star = t3;
  const alpha = t2_star + t3_star;
  const sinA = Math.sin(alpha);

  const x_base_left = 60;
  
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(10, y_base);
  ctx.lineTo(210, y_base);
  ctx.stroke();
  ctx.setLineDash([]);

  // Dibujar base de la cinta transportadora en la Vista Lateral (X-Z)
  ctx.save();
  ctx.fillStyle = '#22232b'; // Gris muy oscuro
  ctx.strokeStyle = '#4e5166'; // Borde metálico
  ctx.lineWidth = 1.5;
  ctx.fillRect(x_base_left + 440 * S, y_base - 40 * S, 120 * S, 40 * S);
  ctx.strokeRect(x_base_left + 440 * S, y_base - 40 * S, 120 * S, 40 * S);
  
  ctx.fillStyle = '#7a7d8c';
  ctx.beginPath();
  ctx.arc(x_base_left + 450 * S, y_base - 20 * S, 8 * S, 0, Math.PI * 2);
  ctx.arc(x_base_left + 550 * S, y_base - 20 * S, 8 * S, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // Dibujar cilindro de corte en la Vista Lateral (X-Z)
  ctx.save();
  ctx.fillStyle = 'rgba(0, 240, 240, 0.25)'; // Cian translúcido
  ctx.strokeStyle = '#00f0f0'; // Borde cian sólido
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(x_base_left + 500 * S, y_base - 55 * S, 15 * S, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  
  // Dibujar el sistema de referencia en la base del cilindro (Z=0 del cilindro, en la superficie superior de la cinta)
  // Eje X (Rojo)
  ctx.strokeStyle = '#ff3b30';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x_base_left + 500 * S, y_base - 40 * S);
  ctx.lineTo(x_base_left + 520 * S, y_base - 40 * S);
  ctx.stroke();
  
  // Eje Z (Azul)
  ctx.strokeStyle = '#007aff';
  ctx.beginPath();
  ctx.moveTo(x_base_left + 500 * S, y_base - 40 * S);
  ctx.lineTo(x_base_left + 500 * S, y_base - 40 * S - 20 * S);
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.font = '500 8px Outfit';
  ctx.fillText('VISTA LATERAL (X-Z)', 12, 16);
  ctx.fillText('MESA', 180, y_base - 5);

  const x0 = x_base_left;
  const y0 = y_base;

  const x1 = x_base_left;
  const y1 = y_base - (d1 + d2) * S;

  const x2 = x1 + (L1 * Math.cos(t2_star)) * S;
  const y2 = y1 - (L1 * Math.sin(t2_star)) * S;

  const x4 = x2 + (L2 * Math.cos(t2_star + t3_star)) * S;
  const y4 = y2 - (L2 * Math.sin(t2_star + t3_star)) * S;

  const forearmAngle = Math.atan2(y4 - y2, x4 - x2);
  const sinA_beam = Math.sin(forearmAngle);

  const xtip = x4;
  const ytip = y4;

  let hitsGroundSide = false;
  let x_beam_end_side = xtip;
  let y_beam_end_side = ytip;

  if (state.laserActive && !state.emergencyStop) {
    if (sinA_beam > 0.001) {
      const t = (y_base - ytip) / sinA_beam;
      if (t > 0) {
        x_beam_end_side = xtip + t * Math.cos(forearmAngle);
        y_beam_end_side = y_base;
        hitsGroundSide = true;
      }
    } else {
      const length = 150;
      x_beam_end_side = xtip + length * Math.cos(forearmAngle);
      y_beam_end_side = ytip + length * Math.sin(forearmAngle);
    }

    ctx.save();
    ctx.strokeStyle = '#ff3b30';
    ctx.lineWidth = 1.2 + (state.laserPower / 30);
    ctx.shadowColor = '#ff3b30';
    ctx.shadowBlur = 6 + (state.laserPower / 8);
    ctx.beginPath();
    ctx.moveTo(xtip, ytip);
    ctx.lineTo(x_beam_end_side, y_beam_end_side);
    ctx.stroke();
    ctx.restore();

    if (hitsGroundSide && x_beam_end_side > 0 && x_beam_end_side < 220) {
      ctx.save();
      const glowGrad = ctx.createRadialGradient(x_beam_end_side, y_beam_end_side, 0, x_beam_end_side, y_beam_end_side, 5 + (state.laserPower / 12));
      glowGrad.addColorStop(0, '#ffffff');
      glowGrad.addColorStop(0.2, '#ff3b30');
      glowGrad.addColorStop(1, 'rgba(255, 59, 48, 0)');
      ctx.fillStyle = glowGrad;
      ctx.beginPath();
      ctx.arc(x_beam_end_side, y_beam_end_side, 5 + (state.laserPower / 12), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }

  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(x_base_left - 14, y_base);
  ctx.lineTo(x_base_left + 14, y_base);
  ctx.stroke();

  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 9;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();

  ctx.strokeStyle = '#ff7300';
  ctx.lineWidth = 7.5;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x4, y4);
  ctx.stroke();

  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#0a0a14';
  ctx.lineWidth = 1.5;

  const drawJoint = (x, y, r) => {
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  };

  drawJoint(x1, y1, 4);
  drawJoint(x2, y2, 3.5);

  ctx.save();
  ctx.translate(x4, y4);
  ctx.rotate(forearmAngle);
  ctx.fillStyle = '#a0a3b5'; ctx.fillRect(0, -5, 2, 10);
  ctx.fillStyle = '#22232b'; ctx.fillRect(2, -4, 9, 8);
  ctx.fillStyle = '#7a7d8c'; ctx.fillRect(11, -2.5, 3, 5);
  ctx.fillStyle = '#bfa15c'; ctx.fillRect(14, -1, 2.5, 2);
  ctx.restore();

  const x_base_right = 330;
  const y_base_right = 90;

  ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.font = '500 8px Outfit';
  ctx.fillText('VISTA SUPERIOR (X-Y)', 232, 16);

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
  ctx.lineWidth = 1;
  for (let r = 25; r <= 85; r += 20) {
    ctx.beginPath();
    ctx.arc(x_base_right, y_base_right, r, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(255, 59, 48, 0.15)';
  ctx.lineWidth = 1;
  const r165 = 165 * Math.PI / 180;
  const rr_max = 85;
  ctx.beginPath();
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_base_right + rr_max * Math.cos(-r165), y_base_right + rr_max * Math.sin(-r165));
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_base_right + rr_max * Math.cos(r165), y_base_right + rr_max * Math.sin(r165));
  ctx.stroke();

  // Dibujar la cinta transportadora en la Vista Superior (X-Y)
  ctx.save();
  ctx.fillStyle = '#22232b'; // Gris muy oscuro
  ctx.strokeStyle = '#4e5166'; // Gris metálico
  ctx.lineWidth = 1.5;
  const beltX = x_base_right + 500 * S - 60 * S;
  const beltWidth = 120 * S;
  ctx.fillRect(beltX, 0, beltWidth, canvas.height);
  ctx.strokeRect(beltX, -2, beltWidth, canvas.height + 4);

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.setLineDash([4, 6]);
  ctx.beginPath();
  ctx.moveTo(x_base_right + 500 * S, 0);
  ctx.lineTo(x_base_right + 500 * S, canvas.height);
  ctx.stroke();
  ctx.restore();

  // Dibujar los 7 cilindros con sus letras en la Vista Superior (X-Y)

  Object.keys(conveyorState.positions).forEach(name => {
    const y_gz = conveyorState.positions[name];
    const y_cv = y_base_right - y_gz * 1000 * S;
    
    if (y_cv >= -50 && y_cv <= canvas.height + 50) {
      ctx.save();
      ctx.fillStyle = 'rgba(0, 240, 240, 0.25)'; // Cian translúcido
      ctx.strokeStyle = '#00f0f0'; // Borde cian sólido
      ctx.lineWidth = 1;
      
      const rectX = x_base_right + 500 * S - 15 * S;
      const rectY = y_cv - 30 * S;
      const rectW = 30 * S;
      const rectH = 60 * S;
      
      ctx.fillRect(rectX, rectY, rectW, rectH);
      ctx.strokeRect(rectX, rectY, rectW, rectH);
      
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 9px Outfit, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(letters[name], rectX + rectW / 2, rectY + rectH / 2);

      // Dibujar sistema de referencia de la base
      // Eje X (Rojo)
      ctx.strokeStyle = '#ff3b30';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x_base_right + 500 * S, y_cv);
      ctx.lineTo(x_base_right + 520 * S, y_cv);
      ctx.stroke();

      // Eje Y (Verde)
      ctx.strokeStyle = '#4cd964';
      ctx.beginPath();
      ctx.moveTo(x_base_right + 500 * S, y_cv);
      ctx.lineTo(x_base_right + 500 * S, y_cv - 20 * S);
      ctx.stroke();
      ctx.restore();
    }
  });

  const t1 = state.j1 * Math.PI / 180;
  const X2 = a2 + L1 * Math.cos(t2_star);
  const X4 = X2 + L2 * Math.cos(t2_star + t3_star);
  const X_tip = X4;

  const x_j3_top = x_base_right + X2 * S * Math.cos(-t1) - d4 * S * Math.sin(-t1);
  const y_j3_top = y_base_right + X2 * S * Math.sin(-t1) + d4 * S * Math.cos(-t1);

  const x_flange_top = x_base_right + X4 * S * Math.cos(-t1) - d4 * S * Math.sin(-t1);
  const y_flange_top = y_base_right + X4 * S * Math.sin(-t1) + d4 * S * Math.cos(-t1);

  const x_tip_top = x_flange_top;
  const y_tip_top = y_flange_top;

  if (state.laserActive && !state.emergencyStop) {
    let X_beam_end = X_tip;
    if (sinA_beam > 0.001) {
      const t = (y_base - ytip) / sinA_beam;
      if (t > 0) {
        X_beam_end = X_tip + (t * Math.cos(forearmAngle)) / S;
      }
    } else {
      X_beam_end = X_tip + 150 / S;
    }

    const x_beam_end_top = x_base_right + X_beam_end * S * Math.cos(-t1);
    const y_beam_end_top = y_base_right + X_beam_end * S * Math.sin(-t1);

    ctx.save();
    ctx.strokeStyle = '#ff3b30';
    ctx.lineWidth = 1.2 + (state.laserPower / 30);
    ctx.shadowColor = '#ff3b30';
    ctx.shadowBlur = 6 + (state.laserPower / 8);
    ctx.beginPath();
    ctx.moveTo(x_tip_top, y_tip_top);
    ctx.lineTo(x_beam_end_top, y_beam_end_top);
    ctx.stroke();
    ctx.restore();

    if (x_beam_end_top > 220 && x_beam_end_top < canvas.width) {
      ctx.save();
      const glowGradTop = ctx.createRadialGradient(x_beam_end_top, y_beam_end_top, 0, x_beam_end_top, y_beam_end_top, 5 + (state.laserPower / 12));
      glowGradTop.addColorStop(0, '#ffffff');
      glowGradTop.addColorStop(0.2, '#ff3b30');
      glowGradTop.addColorStop(1, 'rgba(255, 59, 48, 0)');
      ctx.fillStyle = glowGradTop;
      ctx.beginPath();
      ctx.arc(x_beam_end_top, y_beam_end_top, 5 + (state.laserPower / 12), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }

  ctx.fillStyle = '#1e202b';
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(x_base_right, y_base_right, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = '#ff7300';
  ctx.lineWidth = 6.5;
  ctx.beginPath();
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_j3_top, y_j3_top);
  ctx.stroke();

  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 4.5;
  ctx.beginPath();
  ctx.moveTo(x_j3_top, y_j3_top);
  ctx.lineTo(x_flange_top, y_flange_top);
  ctx.stroke();

  ctx.strokeStyle = '#7a7d8c';
  ctx.lineWidth = 3.5;
  ctx.beginPath();
  ctx.moveTo(x_flange_top, y_flange_top);
  ctx.lineTo(x_tip_top, y_tip_top);
  ctx.stroke();

  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#0a0a14';
  ctx.lineWidth = 1.2;

  drawJoint(x_base_right, y_base_right, 3);
  drawJoint(x_j3_top, y_j3_top, 2.5);
}

/*
 * Maneja los cambios de valores en una sola articulación desde cualquier origen (slider o input),
 * aplicando límites físicos de seguridad antes de propagar la actualización.
 */
/*
 * Valida si una configuracion de angulos del hombro (j2) y codo (j3) provoca
 * colision con el piso (Z < 0), con la columna de la base o autocolision del brazo.
 */
function checkCollision(j2, j3) {
  const theta2 = j2 * Math.PI / 180;
  const theta3 = j3 * Math.PI / 180;

  // Altura del hombro (Z de joint2)
  const z_shoulder = d1 + d2; // 96.0 mm

  // Posicion del codo (joint3)
  const x_elbow = a2 + L1 * Math.cos(theta2);
  const z_elbow = z_shoulder + L1 * Math.sin(theta2);

  // Posicion del efector final (tip)
  const x_tip = x_elbow + L2 * Math.cos(theta2 + theta3);
  const z_tip = z_elbow + L2 * Math.sin(theta2 + theta3);

  // 1. Colision con el piso: el codo o la punta no pueden traspasar Z = 0
  const floorClearance = 2.0; // Margen de seguridad de 2 mm sobre el piso
  if (z_elbow < floorClearance || z_tip < floorClearance) {
    return true;
  }

  // 2. Colision con la columna de la base (cilindro de radio 12 mm en X=0, Y=0 hasta la altura del hombro)
  const baseRadius = 12.0; // Radio de despeje seguro para la base
  const baseHeight = z_shoulder;

  // Muestreo de puntos a lo largo de los eslabones para comprobar invasion del cilindro de la base
  const numSamples = 10;
  for (let i = 0; i <= numSamples; i++) {
    const t = i / numSamples;

    // Segmento Hombro -> Codo
    const x_se = a2 + (x_elbow - a2) * t;
    const z_se = z_shoulder + (z_elbow - z_shoulder) * t;
    if (z_se < baseHeight) {
      const dist3d_se = Math.sqrt(x_se * x_se + d4 * d4);
      if (dist3d_se < baseRadius) {
        return true;
      }
    }

    // Segmento Codo -> Efector Final
    const x_et = x_elbow + (x_tip - x_elbow) * t;
    const z_et = z_elbow + (z_tip - z_elbow) * t;
    if (z_et < baseHeight) {
      const dist3d_et = Math.sqrt(x_et * x_et + d4 * d4);
      if (dist3d_et < baseRadius) {
        return true;
      }
    }
  }

  // 3. Autocolision del brazo (el codo se cierra tanto que choca el eslabon 3 con el 2)
  if (j3 < -150.0) {
    return true;
  }

  return false;
}

function updateJoint(num, value, source) {
  if (state.emergencyStop) return;

  const config = jointsConfig[num];
  let parsed = parseFloat(value);
  
  if (isNaN(parsed)) {
    if (source === 'number') return;
    parsed = 0;
  }

  const cappedValue = Math.max(config.min, Math.min(config.max, parsed));
  
  // Clonamos el estado para validar la colision de forma tentativa
  const tempState = { ...state };
  tempState[config.name] = cappedValue;

  if (checkCollision(tempState.j2, tempState.j3)) {
    // Si la consigna causa colision, forzamos al control de la interfaz a volver
    // al valor de estado anterior valido y terminamos.
    document.getElementById(`input-range-j${num}`).value = state[config.name];
    if (source !== 'number') {
      document.getElementById(`input-number-j${num}`).value = state[config.name].toFixed(1);
    }
    return;
  }

  state[config.name] = cappedValue;

  document.getElementById(`input-range-j${num}`).value = cappedValue;
  if (source !== 'number') {
    document.getElementById(`input-number-j${num}`).value = cappedValue;
  }

  if (ikAnimationId) {
    cancelAnimationFrame(ikAnimationId);
    ikAnimationId = null;
  }

  updateKinematics();
  drawRobot();
  syncIkTargetInputs();
}

/*
 * Sincroniza las cajas de texto numéricas de la cinemática inversa con las coordenadas 
 * cartesianas físicas resultantes de la pose actual.
 */
function syncIkTargetInputs() {
  const currentCoords = updateKinematics();
  document.getElementById('input-ik-x').value = Math.round(currentCoords.x);
  document.getElementById('input-ik-y').value = Math.round(currentCoords.y);
  document.getElementById('input-ik-z').value = Math.round(currentCoords.z);
  
  ikStatus.textContent = 'ALCANCE OK';
  ikStatus.className = 'ik-status-badge ik-status-ok';
}

[1, 2, 3].forEach(num => {
  const rangeInput = document.getElementById(`input-range-j${num}`);
  const numberInput = document.getElementById(`input-number-j${num}`);

  rangeInput.addEventListener('input', (e) => {
    updateJoint(num, e.target.value, 'range');
  });

  numberInput.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (!isNaN(val)) {
      const config = jointsConfig[num];
      const capped = Math.max(config.min, Math.min(config.max, val));
      state[config.name] = capped;
      rangeInput.value = capped;
      
      if (ikAnimationId) {
        cancelAnimationFrame(ikAnimationId);
        ikAnimationId = null;
      }
      
      updateKinematics();
      drawRobot();
      syncIkTargetInputs();
    }
  });

  numberInput.addEventListener('change', (e) => {
    let val = parseFloat(e.target.value);
    if (isNaN(val)) val = 0;
    const config = jointsConfig[num];
    const capped = Math.max(config.min, Math.min(config.max, val));
    
    e.target.value = capped;
    updateJoint(num, capped, 'number');
  });
});

btnApplyIk.addEventListener('click', () => {
  if (state.emergencyStop) return;

  const x = document.getElementById('input-ik-x').value;
  const y = document.getElementById('input-ik-y').value;
  const z = document.getElementById('input-ik-z').value;

  const result = solveIK(x, y, z);

  if (result === null || result.error) {
    ikStatus.textContent = 'FUERA DE ALCANCE';
    ikStatus.className = 'ik-status-badge ik-status-error';
    
    ikStatus.style.animation = 'none';
    void ikStatus.offsetWidth;
    ikStatus.style.animation = '';
    
    console.warn("IK Fallo:", result ? result.msg : "Fuera de limites");
  } else if (checkCollision(result.j2, result.j3)) {
    ikStatus.textContent = 'COLISION';
    ikStatus.className = 'ik-status-badge ik-status-error';
    
    ikStatus.style.animation = 'none';
    void ikStatus.offsetWidth;
    ikStatus.style.animation = '';
  } else {
    ikStatus.textContent = 'ALCANCE OK';
    ikStatus.className = 'ik-status-badge ik-status-ok';
    animateToJoints(result.j1, result.j2, result.j3);
  }
});

['input-ik-x', 'input-ik-y', 'input-ik-z'].forEach(id => {
  document.getElementById(id).addEventListener('input', () => {
    if (state.emergencyStop) return;

    const x = document.getElementById('input-ik-x').value;
    const y = document.getElementById('input-ik-y').value;
    const z = document.getElementById('input-ik-z').value;

    const result = solveIK(x, y, z);

    if (result === null || result.error) {
      ikStatus.textContent = 'FUERA DE ALCANCE';
      ikStatus.className = 'ik-status-badge ik-status-error';
    } else if (checkCollision(result.j2, result.j3)) {
      ikStatus.textContent = 'COLISION';
      ikStatus.className = 'ik-status-badge ik-status-error';
    } else {
      ikStatus.textContent = 'ALCANCE OK';
      ikStatus.className = 'ik-status-badge ik-status-ok';
    }
  });
});

laserPowerSlider.addEventListener('input', (e) => {
  if (state.emergencyStop) return;
  state.laserPower = parseInt(e.target.value);
  laserPowerValText.textContent = state.laserPower;
  
  if (state.laserActive) {
    drawRobot();
  }
});

function sendLaserActiveToROS(active) {
  console.log("[UI] Enviando estado del láser a ROS:", active);
  fetch('/api/toggle_laser', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ active: active })
  })
  .then(res => res.json())
  .then(data => console.log("[UI] Respuesta toggle_laser:", data))
  .catch(err => console.error("[UI] Error al togglear láser:", err));
}

btnLaserToggle.addEventListener('click', () => {
  if (state.emergencyStop) return;
  
  state.laserActive = !state.laserActive;
  
  if (state.laserActive) {
    laserIndicator.textContent = 'ENCENDIDO';
    laserIndicator.className = 'laser-status-badge laser-on';
    btnLaserToggle.textContent = 'DESACTIVAR APUNTADOR LÁSER';
    btnLaserToggle.classList.add('btn-laser-active');
  } else {
    laserIndicator.textContent = 'APAGADO';
    laserIndicator.className = 'laser-status-badge laser-off';
    btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
    btnLaserToggle.classList.remove('btn-laser-active');
  }
  
  sendLaserActiveToROS(state.laserActive);
  drawRobot();
});

function disablePhysInputs(disable) {
  const controls = document.querySelectorAll(
    '#phys-panel-controls input, #phys-btn-reset, #phys-btn-laser-toggle, #phys-btn-apply-ik'
  );
  controls.forEach(el => {
    el.disabled = disable;
  });
}

function setEmergencyState(active) {
  state.emergencyStop = active;
  if (typeof physState !== 'undefined') {
    physState.emergencyStop = active;
  }
  
  const physBtnEmergency = document.getElementById('phys-btn-emergency');

  if (active) {
    // Enviar señal de parada de emergencia física (ESTOP) al ESP32
    fetch('/api/physical/move', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ estop: true })
    }).catch(err => console.error("[UI] Error al enviar ESTOP al ESP32:", err));

    document.body.classList.add('emergency-active');
    btnEmergency.querySelector('span').textContent = 'RESTABLECER Y REINICIAR';
    if (physBtnEmergency) {
      physBtnEmergency.querySelector('span').textContent = 'RESTABLECER Y REINICIAR';
    }
    
    systemStatusText.textContent = 'EMERGENCIA: SISTEMA BLOQUEADO';
    systemStatusText.style.color = '#ff3b30';
    systemStatusDot.style.backgroundColor = '#ff3b30';
    systemStatusDot.style.boxShadow = '0 0 10px #ff3b30';
    
    if (ikAnimationId) {
      cancelAnimationFrame(ikAnimationId);
      ikAnimationId = null;
    }
    if (typeof physIkAnimationId !== 'undefined' && physIkAnimationId) {
      cancelAnimationFrame(physIkAnimationId);
      physIkAnimationId = null;
    }

    state.laserActive = false;
    sendLaserActiveToROS(false);
    laserIndicator.textContent = 'APAGADO';
    laserIndicator.className = 'laser-status-badge laser-off';
    btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
    btnLaserToggle.classList.remove('btn-laser-active');
    
    if (typeof updatePhysLaserUI === 'function') {
      updatePhysLaserUI(false);
    }
    
    // Detener la cinta transportadora por seguridad en parada de emergencia
    conveyorState.active = false;
    fetch('/api/toggle_conveyor', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ active: false })
    }).catch(err => console.error("[UI] Error al detener la cinta en emergencia:", err));

    disableInputs(true);
    disablePhysInputs(true);
    drawRobot();
    if (typeof drawPhysRobot === 'function') {
      drawPhysRobot();
    }
  } else {
    document.body.classList.remove('emergency-active');
    btnEmergency.querySelector('span').textContent = 'PARADA DE EMBALAJES';
    if (physBtnEmergency) {
      physBtnEmergency.querySelector('span').textContent = 'PARADA DE EMERGENCIA';
    }
    
    const isPhysTab = document.getElementById('tab-physical') && document.getElementById('tab-physical').classList.contains('active');
    systemStatusText.textContent = isPhysTab ? 'MODO BRAZO FÍSICO' : 'MODO SIMULACIÓN ACTIVO';
    systemStatusText.style.color = '';
    systemStatusDot.style.backgroundColor = '';
    systemStatusDot.style.boxShadow = '';
    
    disableInputs(false);
    disablePhysInputs(false);
    drawRobot();
    if (typeof drawPhysRobot === 'function') {
      drawPhysRobot();
    }
  }
}

btnEmergency.addEventListener('click', () => {
  setEmergencyState(!state.emergencyStop);
});

const btnResetHome = document.getElementById('btn-reset-home');

btnResetHome.addEventListener('click', () => {
  if (state.emergencyStop) return;
  
  if (ikAnimationId) {
    cancelAnimationFrame(ikAnimationId);
    ikAnimationId = null;
  }
  
  animateToJoints(0, 0, 0);
  
  document.getElementById('input-ik-x').value = 322;
  document.getElementById('input-ik-y').value = 7;
  document.getElementById('input-ik-z').value = 96;
  
  ikStatus.textContent = 'ALCANCE OK';
  ikStatus.className = 'ik-status-badge ik-status-ok';

  state.laserActive = false;
  sendLaserActiveToROS(false);
  state.laserPower = 0;
  laserPowerSlider.value = 0;
  laserPowerValText.textContent = '0';
  laserIndicator.textContent = 'APAGADO';
  laserIndicator.className = 'laser-status-badge laser-off';
  btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
  btnLaserToggle.classList.remove('btn-laser-active');
});

/*
 * Deshabilita o habilita todos los inputs y sliders interactivos de la interfaz web, 
 * limitando el control total del robot de simulación durante estados de emergencia.
 */
function disableInputs(disable) {
  const controls = document.querySelectorAll(
    '.joint-slider, .joint-value-input input, #input-range-laser, #input-ik-x, #input-ik-y, #input-ik-z, #btn-apply-ik, #btn-reset-home, #btn-conveyor-toggle, #btn-calculate-path, #btn-execute-calculated'
  );
  controls.forEach(el => {
    el.disabled = disable;
  });
  
  btnLaserToggle.disabled = disable;
}

function sendWorkspaceToggleToROS(visible) {
  console.log("[UI] Enviando visibilidad del espacio de trabajo a ROS:", visible);
  fetch('/api/toggle_workspace', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ visible: visible })
  })
  .then(res => res.json())
  .then(data => console.log("[UI] Respuesta del servidor:", data))
  .catch(err => console.error("[UI] Error al enviar estado a ROS:", err));
}


/*
 * Inicializa la interfaz de usuario, cargando la cinemática por defecto,
 * dibujando el canvas y sincronizando las coordenadas de entrada.
 */
function init() {
  updateKinematics();
  precomputeWorkspaceLimits();

  // Cargar dinámicamente el mapeo de letras aleatorias
  fetch('target_letters.json')
    .then(res => {
      if (!res.ok) {
        throw new Error('No se pudo obtener el archivo JSON');
      }
      return res.json();
    })
    .then(data => {
      Object.assign(letters, data);
      drawRobot();
    })
    .catch(err => {
      console.warn("No se pudo cargar target_letters.json, usando predeterminado:", err);
    });

  if (checkShowWorkspace) {
    checkShowWorkspace.addEventListener('change', () => {
      drawRobot();
      sendWorkspaceToggleToROS(checkShowWorkspace.checked);
    });
    // Sincronizar estado inicial
    sendWorkspaceToggleToROS(checkShowWorkspace.checked);
  }

  // Configuración de pestañas de visualización (Esquema 2D vs Cámara)
  const btnTab2d = document.getElementById('btn-tab-2d');
  const btnTabCamera = document.getElementById('btn-tab-camera');
  const container2d = document.getElementById('container-2d');
  const containerCamera = document.getElementById('container-camera');
  const imgCameraStream = document.getElementById('camera-stream-img');

  if (btnTab2d && btnTabCamera && container2d && containerCamera && imgCameraStream) {
    btnTab2d.addEventListener('click', () => {
      btnTab2d.classList.add('active');
      btnTabCamera.classList.remove('active');
      container2d.style.display = 'block';
      containerCamera.style.display = 'none';
      imgCameraStream.src = ''; // Detiene la conexión MJPEG para liberar recursos
    });

    btnTabCamera.addEventListener('click', () => {
      btnTabCamera.classList.add('active');
      btnTab2d.classList.remove('active');
      containerCamera.style.display = 'flex';
      container2d.style.display = 'none';
      imgCameraStream.src = '/api/camera_stream'; // Conecta al flujo MJPEG en vivo
    });
  }

  // Configuración de pestañas de visualización del Brazo Físico (Esquema 2D vs Cámara Real)
  const physBtnTab2d = document.getElementById('phys-btn-tab-2d');
  const physBtnTabCamera = document.getElementById('phys-btn-tab-camera');
  const physContainer2d = document.getElementById('phys-container-2d');
  const physContainerCamera = document.getElementById('phys-container-camera');
  const physImgCameraStream = document.getElementById('phys-camera-stream-img');
  const physCameraSelect = document.getElementById('phys-camera-select');

  function loadPhysicalCameras() {
    if (!physCameraSelect) return;
    fetch('/api/list_cameras')
      .then(res => res.json())
      .then(data => {
        const currentSel = physCameraSelect.value;
        physCameraSelect.innerHTML = '<option value="auto">Auto-detectar (Prioriza /dev/video2)</option>';
        if (data.cameras && data.cameras.length > 0) {
          data.cameras.forEach(cam => {
            const opt = document.createElement('option');
            opt.value = cam.id;
            opt.textContent = cam.name;
            physCameraSelect.appendChild(opt);
          });
        }
        // Restaurar selección anterior si sigue estando disponible
        if (Array.from(physCameraSelect.options).some(o => o.value == currentSel)) {
          physCameraSelect.value = currentSel;
        }
      })
      .catch(err => {
        console.error("Error al listar camaras:", err);
      });
  }

  if (physBtnTab2d && physBtnTabCamera && physContainer2d && physContainerCamera && physImgCameraStream) {
    physBtnTab2d.addEventListener('click', () => {
      physBtnTab2d.classList.add('active');
      physBtnTabCamera.classList.remove('active');
      physContainer2d.style.display = 'block';
      physContainerCamera.style.display = 'none';
      physImgCameraStream.src = ''; // Detiene la conexión MJPEG
    });

    physBtnTabCamera.addEventListener('click', () => {
      physBtnTabCamera.classList.add('active');
      physBtnTab2d.classList.remove('active');
      physContainerCamera.style.display = 'flex';
      physContainer2d.style.display = 'none';
      loadPhysicalCameras();
      
      const idx = physCameraSelect ? physCameraSelect.value : 'auto';
      physImgCameraStream.src = `/api/real_camera_stream?index=${idx}&t=${Date.now()}`;
    });

    if (physCameraSelect) {
      physCameraSelect.addEventListener('change', () => {
        const idx = physCameraSelect.value;
        physImgCameraStream.src = `/api/real_camera_stream?index=${idx}&t=${Date.now()}`;
      });
    }
  }

  // Sincronización de Cinta Transportadora
  const btnConveyorToggle = document.getElementById('btn-conveyor-toggle');

  function syncConveyorState() {
    fetch('/api/conveyor_state')
      .then(res => res.json())
      .then(data => {
        const isAct = !!data.active;
        
        if (btnConveyorToggle) {
          btnConveyorToggle.textContent = isAct ? 'DESACTIVAR CINTA' : 'ACTIVAR CINTA';
          if (isAct) {
            btnConveyorToggle.style.background = '#ff453a'; // Rojo premium
          } else {
            btnConveyorToggle.style.background = '#34c759'; // Verde premium
          }
        }
        
        if (isAct !== conveyorState.active) {
          conveyorState.active = isAct;
          if (isAct) {
            lastConveyorTime = performance.now();
            if (!conveyorAnimationId) {
              conveyorAnimationId = requestAnimationFrame(animateConveyor);
            }
          }
        }
        
        if (data.positions) {
          Object.keys(data.positions).forEach(name => {
            conveyorState.positions[name] = data.positions[name];
          });
          if (!isAct) {
            drawRobot();
            if (typeof drawPhysRobot === 'function') {
              drawPhysRobot();
            }
          }
        }
      })
      .catch(err => console.error("[UI] Error al sincronizar cinta:", err));
  }

  if (btnConveyorToggle) {
    btnConveyorToggle.addEventListener('click', () => {
      if (state.emergencyStop) return;
      
      const newActiveState = !conveyorState.active;
      
      // Actualizar interfaz inmediatamente para mayor respuesta táctil
      btnConveyorToggle.textContent = newActiveState ? 'DESACTIVAR CINTA' : 'ACTIVAR CINTA';
      btnConveyorToggle.style.background = newActiveState ? '#ff453a' : '#34c759';
      
      fetch('/api/toggle_conveyor', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ active: newActiveState })
      })
      .then(res => res.json())
      .then(data => {
        const isAct = !!data.active;
        conveyorState.active = isAct;
        if (isAct) {
          lastConveyorTime = performance.now();
          if (!conveyorAnimationId) {
            conveyorAnimationId = requestAnimationFrame(animateConveyor);
          }
        }
        btnConveyorToggle.textContent = isAct ? 'DESACTIVAR CINTA' : 'ACTIVAR CINTA';
        btnConveyorToggle.style.background = isAct ? '#ff453a' : '#34c759';
      })
      .catch(err => {
        console.error("[UI] Error al togglear cinta:", err);
        // Revertir UI en caso de error
        btnConveyorToggle.textContent = conveyorState.active ? 'DESACTIVAR CINTA' : 'ACTIVAR CINTA';
        btnConveyorToggle.style.background = conveyorState.active ? '#ff453a' : '#34c759';
      });
    });
  }

  let calculatedPoints = [];

  const btnCalculatePath = document.getElementById('btn-calculate-path');
  const pathResultContainer = document.getElementById('path-result-container');
  const pathResultImg = document.getElementById('path-result-img');
  const btnExecuteCalculated = document.getElementById('btn-execute-calculated');

  if (btnCalculatePath && pathResultContainer && pathResultImg && btnExecuteCalculated) {
    btnCalculatePath.addEventListener('click', () => {
      if (state.emergencyStop) return;
      btnCalculatePath.disabled = true;
      btnCalculatePath.textContent = 'PROCESANDO...';
      btnCalculatePath.style.background = '#2c2c3e';
      pathResultContainer.style.display = 'none';
      
      fetch('/api/calculate_path', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({})
      })
      .then(res => {
        if (!res.ok) {
          return res.json().then(err => { throw new Error(err.error || 'Error desconocido') });
        }
        return res.json();
      })
      .then(data => {
        btnCalculatePath.disabled = false;
        btnCalculatePath.textContent = 'CALCULAR RECORRIDO';
        btnCalculatePath.style.background = '#007aff';
        
        if (data.status === 'success' && data.points && data.points.length > 0) {
          calculatedPoints = data.points;
          pathResultImg.src = data.image;
          pathResultContainer.style.display = 'block';
          console.log(`Puntos calculados con éxito: ${calculatedPoints.length} puntos.`);
        }
      })
      .catch(err => {
        console.error("[UI] Error al calcular recorrido:", err);
        alert("Fallo al calcular recorrido: " + err.message);
        btnCalculatePath.disabled = false;
        btnCalculatePath.textContent = 'CALCULAR RECORRIDO';
        btnCalculatePath.style.background = '#007aff';
      });
    });

    btnExecuteCalculated.addEventListener('click', () => {
      if (calculatedPoints.length === 0) return;
      if (state.emergencyStop) return;
      
      state.executingTrajectory = true;
      disableInputs(true);
      btnExecuteCalculated.disabled = true;
      btnExecuteCalculated.textContent = 'EJECUTANDO...';
      btnExecuteCalculated.style.background = '#2c2c3e';
      
      executeTrajectory(calculatedPoints)
        .then(() => {
          state.executingTrajectory = false;
          if (!state.emergencyStop) {
            disableInputs(false);
          }
          btnExecuteCalculated.disabled = state.emergencyStop;
          btnExecuteCalculated.textContent = 'EJECUTAR RECORRIDO';
          btnExecuteCalculated.style.background = '#34c759';
        })
        .catch(err => {
          state.executingTrajectory = false;
          console.error("Error en ejecución de trayectoria:", err);
          alert("Error durante la ejecución del recorrido: " + err.message);
          if (!state.emergencyStop) {
            disableInputs(false);
          }
          btnExecuteCalculated.disabled = state.emergencyStop;
          btnExecuteCalculated.textContent = 'EJECUTAR RECORRIDO';
          btnExecuteCalculated.style.background = '#34c759';
        });
    });
  }

  function executeTrajectory(points) {
    return new Promise(async (resolve, reject) => {
      if (points.length === 0) {
        resolve();
        return;
      }

      // Parámetros de velocidad configurables (bajados por solicitud del usuario para hacer el recorrido más lento)
      const drawSpeedMms = 5.0;      // Velocidad de dibujo en mm/s (por defecto 10 mm/s)
      const approachSpeedMms = 15.0; // Velocidad de aproximación/transición en mm/s (por defecto 25 mm/s)
      const rateHz = 20;             // Frecuencia de comandos en Hz
      const intervalMs = 1000 / rateHz;

      const stepSizeDraw = drawSpeedMms / rateHz;
      const stepSizeApproach = approachSpeedMms / rateHz;

      // 1. Resolver la cinemática inversa para el primer punto del recorrido
      const firstTarget = points[0];
      const targetIK = solveIK(firstTarget.x, firstTarget.y, firstTarget.z, L2 + 200.0);
      if (!targetIK || targetIK.error) {
        reject(new Error("El punto de inicio del recorrido está fuera de alcance: " + (targetIK ? targetIK.msg : "")));
        return;
      }

      const jointTrajectory = [];

      // 2. Generar trayectoria de posicionamiento inicial suave en el espacio articular (Joint Space)
      const dj1 = targetIK.j1 - state.j1;
      const dj2 = targetIK.j2 - state.j2;
      const dj3 = targetIK.j3 - state.j3;
      const maxDiff = Math.max(Math.abs(dj1), Math.abs(dj2), Math.abs(dj3));

      // Duración proporcional lenta (máx 15 grados/segundo para seguridad, mínimo 1.5 segundos)
      const transitionDuration = Math.max(1.5, maxDiff / 15.0);
      const transitionSteps = Math.ceil(transitionDuration * rateHz);

      for (let k = 1; k <= transitionSteps; k++) {
        const t = k / transitionSteps;
        // Interpolación cosenoidal para aceleración y desaceleración suaves
        const smooth_t = (1.0 - Math.cos(t * Math.PI)) / 2.0;
        jointTrajectory.push({
          j1: state.j1 + dj1 * smooth_t,
          j2: state.j2 + dj2 * smooth_t,
          j3: state.j3 + dj3 * smooth_t,
          laser: false // Láser apagado durante la transición
        });
      }

      // 3. Interpolación cartesiana lineal para el trazo de la letra (recorrido)
      const tracingPoints = [];
      tracingPoints.push(points[0]);
      for (let i = 1; i < points.length; i++) {
        const pPrev = points[i - 1];
        const pCurr = points[i];
        const dx = pCurr.x - pPrev.x;
        const dy = pCurr.y - pPrev.y;
        const dz = pCurr.z - pPrev.z;
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        const stepSize = pCurr.laser ? stepSizeDraw : stepSizeApproach;

        if (dist > stepSize) {
          const steps = Math.ceil(dist / stepSize);
          for (let k = 1; k <= steps; k++) {
            const t = k / steps;
            tracingPoints.push({
              x: pPrev.x + dx * t,
              y: pPrev.y + dy * t,
              z: pPrev.z + dz * t,
              laser: pCurr.laser
            });
          }
        } else {
          tracingPoints.push(pCurr);
        }
      }

      // 4. Resolver cinemática inversa y añadir el trazo a la trayectoria
      let validTracingCount = 0;
      for (let p of tracingPoints) {
        const result = solveIK(p.x, p.y, p.z, L2 + 200.0);
        if (result && !result.error && !checkCollision(result.j2, result.j3)) {
          jointTrajectory.push({
            j1: result.j1,
            j2: result.j2,
            j3: result.j3,
            laser: p.laser
          });
          validTracingCount++;
        }
      }

      if (validTracingCount === 0) {
        reject(new Error("Todos los puntos calculados están fuera de alcance o en colisión."));
        return;
      }
      
      let index = 0;
      
      // Inicializar el láser según el primer punto del recorrido
      const firstLaser = jointTrajectory[0].laser;
      state.laserActive = firstLaser;
      sendLaserActiveToROS(firstLaser);
      if (firstLaser) {
        laserIndicator.textContent = 'ENCENDIDO';
        laserIndicator.className = 'laser-status-badge laser-on';
        btnLaserToggle.textContent = 'DESACTIVAR APUNTADOR LÁSER';
        btnLaserToggle.classList.add('btn-laser-active');
      } else {
        laserIndicator.textContent = 'APAGADO';
        laserIndicator.className = 'laser-status-badge laser-off';
        btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
        btnLaserToggle.classList.remove('btn-laser-active');
      }
      
      while (index < jointTrajectory.length && !state.emergencyStop && state.executingTrajectory) {
        const startTime = Date.now();
        const currentJoints = jointTrajectory[index];
        
        // Controlar dinámicamente el láser en cada punto (apagar en saltos de trayectoria)
        const targetLaser = currentJoints.laser;
        if (state.laserActive !== targetLaser) {
          state.laserActive = targetLaser;
          sendLaserActiveToROS(targetLaser);
          if (targetLaser) {
            laserIndicator.textContent = 'ENCENDIDO';
            laserIndicator.className = 'laser-status-badge laser-on';
            btnLaserToggle.textContent = 'DESACTIVAR APUNTADOR LÁSER';
            btnLaserToggle.classList.add('btn-laser-active');
          } else {
            laserIndicator.textContent = 'APAGADO';
            laserIndicator.className = 'laser-status-badge laser-off';
            btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
            btnLaserToggle.classList.remove('btn-laser-active');
          }
        }
        state.j1 = currentJoints.j1;
        state.j2 = currentJoints.j2;
        state.j3 = currentJoints.j3;
        
        for (let num = 1; num <= 3; num++) {
          const valName = jointsConfig[num].name;
          document.getElementById(`input-range-j${num}`).value = state[valName];
          document.getElementById(`input-number-j${num}`).value = state[valName].toFixed(1);
        }
        
        updateKinematics();
        drawRobot();
        
        // Aplicamos los desfasajes exactos para que la simulación física en Gazebo coincida con la UI
        const j2_val = state.j2 - 59.017;
        const j3_val = state.j3 + 70.0;
        const j2_ros = Math.max(-110.0, Math.min(110.0, j2_val));
        const j3_ros = Math.max(-110.0, Math.min(70.0, j3_val));

        try {
          await fetch('/api/move', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json'
            },
            body: JSON.stringify({ j1: state.j1, j2: j2_ros, j3: j3_ros })
          });
        } catch (err) {
          console.error("Error al mover articulación:", err);
        }
        
        index++;
        
        const elapsed = Date.now() - startTime;
        const waitTime = Math.max(0, intervalMs - elapsed);
        if (waitTime > 0 && index < jointTrajectory.length && !state.emergencyStop && state.executingTrajectory) {
          await new Promise(resolveWait => setTimeout(resolveWait, waitTime));
        }
      }

      // Desactivar el láser al finalizar o por parada de emergencia
      state.laserActive = false;
      sendLaserActiveToROS(false);
      laserIndicator.textContent = 'APAGADO';
      laserIndicator.className = 'laser-status-badge laser-off';
      btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
      btnLaserToggle.classList.remove('btn-laser-active');
      
      if (state.emergencyStop) {
        reject(new Error("Ejecución abortada por parada de emergencia."));
      } else if (!state.executingTrajectory) {
        reject(new Error("Ejecución cancelada."));
      } else {
        resolve();
      }
    });
  }

  // Sincronizar estado inicial y correr polling a 2Hz
  syncConveyorState();
  setInterval(syncConveyorState, 500);

  drawRobot();
  syncIkTargetInputs();
  
  window.addEventListener('load', () => {
    drawRobot();
  });
}

/* ==================== CONTROL DEL BRAZO FÍSICO (ESP32) ==================== */

const physState = {
  j1: 0.0,
  j2: 0.0,
  j3: 0.0,
  laserActive: false,
  emergencyStop: false,
  connected: false,
  executingTrajectory: false
};

const physCanvas = document.getElementById('phys-robot-canvas');
const physCtx = physCanvas ? physCanvas.getContext('2d') : null;
let physIkAnimationId = null;
let statusIntervalId = null;
let physTrajectoryIntervalId = null;
let lastPhysSentTime = 0;
const physSendIntervalMs = 50;
let physSendTimeout = null;
let physLaserKeepAliveInterval = null;

function drawPhysRobot() {
  if (!physCtx) return;
  const ctx = physCtx;
  const canvas = physCanvas;
  
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
  ctx.lineWidth = 1;
  const gridSize = 20;
  for (let x = 0; x < canvas.width; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 0; y < canvas.height; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.lineWidth = 1;
  ctx.setLineDash([2, 4]);
  ctx.beginPath();
  ctx.moveTo(220, 0);
  ctx.lineTo(220, canvas.height);
  ctx.stroke();
  ctx.setLineDash([]);

  // Dibujar espacio de trabajo en físico (siempre visible para feedback visual)
  drawWorkspaceLateral(physCtx);
  drawWorkspaceTop(physCtx);

  const t2 = physState.j2 * Math.PI / 180;
  const t3 = physState.j3 * Math.PI / 180;
  const t2_star = t2;
  const t3_star = t3;
  const alpha = t2_star + t3_star;
  const sinA = Math.sin(alpha);

  const x_base_left = 60;
  
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(10, y_base);
  ctx.lineTo(210, y_base);
  ctx.stroke();
  ctx.setLineDash([]);

  // Cinta transportadora (Vista Lateral)
  ctx.save();
  ctx.fillStyle = '#22232b';
  ctx.strokeStyle = '#4e5166';
  ctx.lineWidth = 1.5;
  ctx.fillRect(x_base_left + 440 * S, y_base - 40 * S, 120 * S, 40 * S);
  ctx.strokeRect(x_base_left + 440 * S, y_base - 40 * S, 120 * S, 40 * S);
  
  ctx.fillStyle = '#7a7d8c';
  ctx.beginPath();
  ctx.arc(x_base_left + 450 * S, y_base - 20 * S, 8 * S, 0, Math.PI * 2);
  ctx.arc(x_base_left + 550 * S, y_base - 20 * S, 8 * S, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // Cilindro de corte (Vista Lateral)
  ctx.save();
  ctx.fillStyle = 'rgba(0, 240, 240, 0.25)';
  ctx.strokeStyle = '#00f0f0';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(x_base_left + 500 * S, y_base - 55 * S, 15 * S, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  
  // Ejes base
  ctx.strokeStyle = '#ff3b30';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x_base_left + 500 * S, y_base - 40 * S);
  ctx.lineTo(x_base_left + 520 * S, y_base - 40 * S);
  ctx.stroke();
  
  ctx.strokeStyle = '#007aff';
  ctx.beginPath();
  ctx.moveTo(x_base_left + 500 * S, y_base - 40 * S);
  ctx.lineTo(x_base_left + 500 * S, y_base - 40 * S - 20 * S);
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.font = '500 8px Outfit';
  ctx.fillText('VISTA LATERAL (X-Z)', 12, 16);
  ctx.fillText('MESA', 180, y_base - 5);

  const x0 = x_base_left;
  const y0 = y_base;
  const x1 = x_base_left;
  const y1 = y_base - (d1 + d2) * S;
  const x2 = x1 + (L1 * Math.cos(t2_star)) * S;
  const y2 = y1 - (L1 * Math.sin(t2_star)) * S;
  const x4 = x2 + (L2 * Math.cos(t2_star + t3_star)) * S;
  const y4 = y2 - (L2 * Math.sin(t2_star + t3_star)) * S;

  const forearmAngle = Math.atan2(y4 - y2, x4 - x2);
  const sinA_beam = Math.sin(forearmAngle);
  const xtip = x4;
  const ytip = y4;

  let hitsGroundSide = false;
  let x_beam_end_side = xtip;
  let y_beam_end_side = ytip;

  if (physState.laserActive && !physState.emergencyStop) {
    if (sinA_beam > 0.001) {
      const t = (y_base - ytip) / sinA_beam;
      if (t > 0) {
        x_beam_end_side = xtip + t * Math.cos(forearmAngle);
        y_beam_end_side = y_base;
        hitsGroundSide = true;
      }
    } else {
      const length = 150;
      x_beam_end_side = xtip + length * Math.cos(forearmAngle);
      y_beam_end_side = ytip + length * Math.sin(forearmAngle);
    }

    ctx.save();
    ctx.strokeStyle = '#ff3b30';
    ctx.lineWidth = 2.0;
    ctx.shadowColor = '#ff3b30';
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.moveTo(xtip, ytip);
    ctx.lineTo(x_beam_end_side, y_beam_end_side);
    ctx.stroke();
    ctx.restore();

    if (hitsGroundSide && x_beam_end_side > 0 && x_beam_end_side < 220) {
      ctx.save();
      const glowGrad = ctx.createRadialGradient(x_beam_end_side, y_beam_end_side, 0, x_beam_end_side, y_beam_end_side, 8);
      glowGrad.addColorStop(0, '#ffffff');
      glowGrad.addColorStop(0.2, '#ff3b30');
      glowGrad.addColorStop(1, 'rgba(255, 59, 48, 0)');
      ctx.fillStyle = glowGrad;
      ctx.beginPath();
      ctx.arc(x_beam_end_side, y_beam_end_side, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }

  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(x_base_left - 14, y_base);
  ctx.lineTo(x_base_left + 14, y_base);
  ctx.stroke();

  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 9;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();

  ctx.strokeStyle = '#ff7300';
  ctx.lineWidth = 7.5;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x4, y4);
  ctx.stroke();

  const drawJoint = (x, y, r) => {
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  };

  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#0a0a14';
  ctx.lineWidth = 1.5;
  drawJoint(x1, y1, 4);
  drawJoint(x2, y2, 3.5);

  ctx.save();
  ctx.translate(x4, y4);
  ctx.rotate(forearmAngle);
  ctx.fillStyle = '#a0a3b5'; ctx.fillRect(0, -5, 2, 10);
  ctx.fillStyle = '#22232b'; ctx.fillRect(2, -4, 9, 8);
  ctx.fillStyle = '#7a7d8c'; ctx.fillRect(11, -2.5, 3, 5);
  ctx.fillStyle = '#bfa15c'; ctx.fillRect(14, -1, 2.5, 2);
  ctx.restore();

  const x_base_right = 330;
  const y_base_right = 90;

  ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.font = '500 8px Outfit';
  ctx.fillText('VISTA SUPERIOR (X-Y)', 232, 16);

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
  ctx.lineWidth = 1;
  for (let r = 25; r <= 85; r += 20) {
    ctx.beginPath();
    ctx.arc(x_base_right, y_base_right, r, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(255, 59, 48, 0.15)';
  ctx.lineWidth = 1;
  const r165 = 165 * Math.PI / 180;
  const rr_max = 85;
  ctx.beginPath();
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_base_right + rr_max * Math.cos(-r165), y_base_right + rr_max * Math.sin(-r165));
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_base_right + rr_max * Math.cos(r165), y_base_right + rr_max * Math.sin(r165));
  ctx.stroke();

  // Cinta Vista Superior
  ctx.save();
  ctx.fillStyle = '#22232b';
  ctx.strokeStyle = '#4e5166';
  ctx.lineWidth = 1.5;
  const beltX = x_base_right + 500 * S - 60 * S;
  const beltWidth = 120 * S;
  ctx.fillRect(beltX, 0, beltWidth, canvas.height);
  ctx.strokeRect(beltX, -2, beltWidth, canvas.height + 4);

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.setLineDash([4, 6]);
  ctx.beginPath();
  ctx.moveTo(x_base_right + 500 * S, 0);
  ctx.lineTo(x_base_right + 500 * S, canvas.height);
  ctx.stroke();
  ctx.restore();

  // Dibujar cilindros
  Object.keys(conveyorState.positions).forEach(name => {
    const y_gz = conveyorState.positions[name];
    const y_cv = y_base_right - y_gz * 1000 * S;
    
    if (y_cv >= -50 && y_cv <= canvas.height + 50) {
      ctx.save();
      ctx.fillStyle = 'rgba(0, 240, 240, 0.25)';
      ctx.strokeStyle = '#00f0f0';
      ctx.lineWidth = 1;
      
      const rectX = x_base_right + 500 * S - 15 * S;
      const rectY = y_cv - 30 * S;
      const rectW = 30 * S;
      const rectH = 60 * S;
      
      ctx.fillRect(rectX, rectY, rectW, rectH);
      ctx.strokeRect(rectX, rectY, rectW, rectH);
      
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 9px Outfit, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(letters[name] || '', rectX + rectW / 2, rectY + rectH / 2);

      // Ejes
      ctx.strokeStyle = '#ff3b30';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x_base_right + 500 * S, y_cv);
      ctx.lineTo(x_base_right + 520 * S, y_cv);
      ctx.stroke();

      ctx.strokeStyle = '#4cd964';
      ctx.beginPath();
      ctx.moveTo(x_base_right + 500 * S, y_cv);
      ctx.lineTo(x_base_right + 500 * S, y_cv - 20 * S);
      ctx.stroke();
      ctx.restore();
    }
  });

  const t1 = physState.j1 * Math.PI / 180;
  const X2 = a2 + L1 * Math.cos(t2_star);
  const X4 = X2 + L2 * Math.cos(t2_star + t3_star);
  const X_tip = X4;

  const x_j3_top = x_base_right + X2 * S * Math.cos(-t1) - d4 * S * Math.sin(-t1);
  const y_j3_top = y_base_right + X2 * S * Math.sin(-t1) + d4 * S * Math.cos(-t1);
  const x_flange_top = x_base_right + X4 * S * Math.cos(-t1) - d4 * S * Math.sin(-t1);
  const y_flange_top = y_base_right + X4 * S * Math.sin(-t1) + d4 * S * Math.cos(-t1);
  const x_tip_top = x_flange_top;
  const y_tip_top = y_flange_top;

  if (physState.laserActive && !physState.emergencyStop) {
    let X_beam_end = X_tip;
    if (sinA_beam > 0.001) {
      const t = (y_base - ytip) / sinA_beam;
      if (t > 0) {
        X_beam_end = X_tip + (t * Math.cos(forearmAngle)) / S;
      }
    } else {
      X_beam_end = X_tip + 150 / S;
    }

    const x_beam_end_top = x_base_right + X_beam_end * S * Math.cos(-t1);
    const y_beam_end_top = y_base_right + X_beam_end * S * Math.sin(-t1);

    ctx.save();
    ctx.strokeStyle = '#ff3b30';
    ctx.lineWidth = 2.0;
    ctx.shadowColor = '#ff3b30';
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.moveTo(x_tip_top, y_tip_top);
    ctx.lineTo(x_beam_end_top, y_beam_end_top);
    ctx.stroke();
    ctx.restore();

    if (x_beam_end_top > 220 && x_beam_end_top < canvas.width) {
      ctx.save();
      const glowGradTop = ctx.createRadialGradient(x_beam_end_top, y_beam_end_top, 0, x_beam_end_top, y_beam_end_top, 8);
      glowGradTop.addColorStop(0, '#ffffff');
      glowGradTop.addColorStop(0.2, '#ff3b30');
      glowGradTop.addColorStop(1, 'rgba(255, 59, 48, 0)');
      ctx.fillStyle = glowGradTop;
      ctx.beginPath();
      ctx.arc(x_beam_end_top, y_beam_end_top, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }

  ctx.fillStyle = '#1e202b';
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(x_base_right, y_base_right, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = '#ff7300';
  ctx.lineWidth = 6.5;
  ctx.beginPath();
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_j3_top, y_j3_top);
  ctx.stroke();

  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 4.5;
  ctx.beginPath();
  ctx.moveTo(x_j3_top, y_j3_top);
  ctx.lineTo(x_flange_top, y_flange_top);
  ctx.stroke();

  ctx.strokeStyle = '#7a7d8c';
  ctx.lineWidth = 3.5;
  ctx.beginPath();
  ctx.moveTo(x_flange_top, y_flange_top);
  ctx.lineTo(x_tip_top, y_tip_top);
  ctx.stroke();

  drawJoint(x_base_right, y_base_right, 3);
  drawJoint(x_j3_top, y_j3_top, 2.5);
}

function updatePhysKinematics() {
  const theta1 = physState.j1 * Math.PI / 180;
  const theta2 = physState.j2 * Math.PI / 180;
  const theta3 = physState.j3 * Math.PI / 180;

  const t2_star = theta2;
  const t3_star = theta3;

  const x_plane = a2 + L1 * Math.cos(t2_star) + L2 * Math.cos(t2_star + t3_star);
  const z_plane = (d1 + d2) + L1 * Math.sin(t2_star) + L2 * Math.sin(t2_star + t3_star);

  const x = x_plane * Math.cos(theta1) + d4 * Math.sin(theta1);
  const y = x_plane * Math.sin(theta1) - d4 * Math.cos(theta1);
  const z = z_plane;

  document.getElementById('phys-coord-x').textContent = `${x.toFixed(1)} mm`;
  document.getElementById('phys-coord-y').textContent = `${y.toFixed(1)} mm`;
  document.getElementById('phys-coord-z').textContent = `${z.toFixed(1)} mm`;

  sendPhysCoordsToESP32(x, y, z);

  return { x_plane, z_plane, x, y, z };
}

function sendPhysCoordsToESP32(x, y, z) {
  if (physState.emergencyStop) return;
  if (physState.executingTrajectory) return;
  const now = Date.now();
  
  if (physSendTimeout) {
    clearTimeout(physSendTimeout);
    physSendTimeout = null;
  }

  const executeSend = () => {
    lastPhysSentTime = Date.now();
    
    fetch('/api/physical/move', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        x,
        y,
        z,
        j1: physState.j1,
        j2: physState.j2,
        j3: physState.j3,
        laser: physState.laserActive ? 1 : 0
      })
    })
    .then(res => res.json())
    .then(data => {
      const logEl = document.getElementById('phys-esp-log');
      if (logEl && data.response) {
        logEl.textContent = data.response;
        if (data.status === 'ok') {
          logEl.style.color = '#34c759';
        } else {
          logEl.style.color = '#ff3b30';
        }
      }
    })
    .catch(err => {
      console.error("Error enviando coordenadas serial:", err);
      const logEl = document.getElementById('phys-esp-log');
      if (logEl) {
        logEl.textContent = `Error de red: ${err.message}`;
        logEl.style.color = '#ff3b30';
      }
    });
  };

  if (now - lastPhysSentTime >= physSendIntervalMs) {
    executeSend();
  } else {
    const remaining = physSendIntervalMs - (now - lastPhysSentTime);
    physSendTimeout = setTimeout(executeSend, remaining);
  }
}

function pollPhysicalStatus() {
  const checkStatus = () => {
    fetch('/api/physical/status')
      .then(res => res.json())
      .then(data => {
        updatePhysConnectionUI(data);
      })
      .catch(err => {
        console.error("Error al consultar estado serial:", err);
      });
  };
  
  checkStatus();
  
  if (!statusIntervalId) {
    statusIntervalId = setInterval(checkStatus, 3000);
  }
}

function stopPollingPhysicalStatus() {
  if (statusIntervalId) {
    clearInterval(statusIntervalId);
    statusIntervalId = null;
  }
}

function updatePhysConnectionUI(data) {
  physState.connected = !!data.connected;
  
  const statusDot = document.getElementById('phys-status-dot');
  const statusText = document.getElementById('phys-status-text');
  const tabDot = document.getElementById('physical-dot');
  const portLabel = document.getElementById('phys-port-label');
  const btnConnect = document.getElementById('phys-btn-connect');
  const errorMsg = document.getElementById('phys-error-msg');
  
  if (statusDot) {
    statusDot.style.backgroundColor = data.connected ? '#34c759' : '#636366';
    statusDot.style.boxShadow = data.connected ? '0 0 10px #34c759' : 'none';
  }
  
  if (statusText) {
    statusText.textContent = data.connected ? 'CONECTADO' : 'DESCONECTADO';
    statusText.style.color = data.connected ? '#34c759' : 'rgba(255,255,255,0.85)';
  }
  
  if (tabDot) {
    if (data.connected) {
      tabDot.classList.add('connected');
    } else {
      tabDot.classList.remove('connected');
    }
  }
  
  if (portLabel) {
    portLabel.textContent = data.connected ? data.port : '';
  }
  
  if (btnConnect) {
    btnConnect.textContent = data.connected ? 'DESCONECTAR' : 'CONECTAR';
    btnConnect.style.background = data.connected ? '#ff453a' : '#007aff';
  }
  
  if (errorMsg) {
    const errorText = data.last_error || data.error;
    if (errorText && !data.connected) {
      errorMsg.textContent = `Error: ${errorText}`;
      errorMsg.style.display = 'block';
    } else {
      errorMsg.style.display = 'none';
    }
  }
}

function updatePhysLaserUI(active) {
  physState.laserActive = active;
  const laserIndicator = document.getElementById('phys-laser-indicator');
  const btnLaserToggle = document.getElementById('phys-btn-laser-toggle');
  
  if (laserIndicator) {
    laserIndicator.textContent = active ? 'ENCENDIDO' : 'APAGADO';
    laserIndicator.className = active ? 'laser-status-badge laser-on' : 'laser-status-badge laser-off';
  }
  if (btnLaserToggle) {
    btnLaserToggle.textContent = active ? 'DESACTIVAR APUNTADOR LÁSER' : 'ACTIVAR APUNTADOR LÁSER';
    if (active) {
      btnLaserToggle.classList.add('btn-laser-active');
    } else {
      btnLaserToggle.classList.remove('btn-laser-active');
    }
  }

  // Manejar intervalo de keep-alive para alimentar el watchdog de seguridad del ESP32 (1000 ms)
  if (physLaserKeepAliveInterval) {
    clearInterval(physLaserKeepAliveInterval);
    physLaserKeepAliveInterval = null;
  }
  if (active) {
    physLaserKeepAliveInterval = setInterval(() => {
      if (!physState.emergencyStop && !physState.executingTrajectory) {
        updatePhysKinematics();
      }
    }, 250);
  }
}

function updatePhysJoint(num, value, source) {
  if (physState.emergencyStop) return;

  const min = jointsConfig[num].min;
  const max = jointsConfig[num].max;
  let parsed = parseFloat(value);
  
  if (isNaN(parsed)) {
    if (source === 'number') return;
    parsed = 0;
  }

  const cappedValue = Math.max(min, Math.min(max, parsed));
  
  const tempState = { ...physState };
  tempState['j' + num] = cappedValue;

  if (checkCollision(tempState.j2, tempState.j3)) {
    document.getElementById(`phys-input-range-j${num}`).value = physState['j' + num];
    if (source !== 'number') {
      document.getElementById(`phys-input-number-j${num}`).value = physState['j' + num].toFixed(1);
    }
    return;
  }

  physState['j' + num] = cappedValue;

  document.getElementById(`phys-input-range-j${num}`).value = cappedValue;
  if (source !== 'number') {
    document.getElementById(`phys-input-number-j${num}`).value = cappedValue;
  }

  if (physIkAnimationId) {
    cancelAnimationFrame(physIkAnimationId);
    physIkAnimationId = null;
  }

  updatePhysKinematics();
  drawPhysRobot();
  syncPhysIkTargetInputs();
}

function syncPhysIkTargetInputs() {
  const currentCoords = updatePhysKinematics();
  document.getElementById('phys-input-ik-x').value = Math.round(currentCoords.x);
  document.getElementById('phys-input-ik-y').value = Math.round(currentCoords.y);
  document.getElementById('phys-input-ik-z').value = Math.round(currentCoords.z);
  
  const ikStatus = document.getElementById('phys-ik-status');
  if (ikStatus) {
    ikStatus.textContent = 'ALCANCE OK';
    ikStatus.className = 'ik-status-badge ik-status-ok';
  }
}

function animatePhysToJoints(targetJ1, targetJ2, targetJ3) {
  if (physIkAnimationId) {
    cancelAnimationFrame(physIkAnimationId);
  }

  const startJ1 = physState.j1;
  const startJ2 = physState.j2;
  const startJ3 = physState.j3;

  const duration = 600;
  const startTime = performance.now();

  function step(now) {
    const elapsed = now - startTime;
    const progress = Math.min(elapsed / duration, 1);
    const ease = 1 - Math.pow(1 - progress, 3);

    physState.j1 = startJ1 + (targetJ1 - startJ1) * ease;
    physState.j2 = startJ2 + (targetJ2 - startJ2) * ease;
    physState.j3 = startJ3 + (targetJ3 - startJ3) * ease;

    for (let num = 1; num <= 3; num++) {
      document.getElementById(`phys-input-range-j${num}`).value = physState['j' + num];
      document.getElementById(`phys-input-number-j${num}`).value = physState['j' + num].toFixed(1);
    }

    updatePhysKinematics();
    drawPhysRobot();

    if (progress < 1) {
      physIkAnimationId = requestAnimationFrame(step);
    } else {
      physIkAnimationId = null;
    }
  }

  physIkAnimationId = requestAnimationFrame(step);
}

function disablePhysInputs(disable) {
  const controls = document.querySelectorAll(
    '#phys-input-range-j1, #phys-input-range-j2, #phys-input-range-j3, #phys-input-number-j1, #phys-input-number-j2, #phys-input-number-j3, #phys-input-ik-x, #phys-input-ik-y, #phys-input-ik-z, #phys-btn-apply-ik, #phys-btn-reset, #phys-btn-laser-toggle, #phys-btn-calculate-path, #phys-btn-execute-calculated, #phys-camera-select'
  );
  controls.forEach(el => {
    if (el) el.disabled = disable;
  });
}

function executePhysTrajectory(points) {
  return new Promise(async (resolve, reject) => {
    if (points.length === 0) {
      resolve();
      return;
    }

    // Parámetros de velocidad configurables (bajados por solicitud del usuario para hacer el recorrido más lento)
    const drawSpeedMms = 5.0;
    const approachSpeedMms = 15.0;
    const rateHz = 20;
    const intervalMs = 1000 / rateHz;

    const stepSizeDraw = drawSpeedMms / rateHz;
    const stepSizeApproach = approachSpeedMms / rateHz;

    // 1. Resolver la cinemática inversa para el primer punto del recorrido
    const firstTarget = points[0];
    const targetIK = solveIK(firstTarget.x, firstTarget.y, firstTarget.z, L2 + 200.0);
    if (!targetIK || targetIK.error) {
      reject(new Error("El punto de inicio del recorrido está fuera de alcance: " + (targetIK ? targetIK.msg : "")));
      return;
    }

    const jointTrajectory = [];

    // 2. Generar trayectoria de posicionamiento inicial suave
    const dj1 = targetIK.j1 - physState.j1;
    const dj2 = targetIK.j2 - physState.j2;
    const dj3 = targetIK.j3 - physState.j3;
    const maxDiff = Math.max(Math.abs(dj1), Math.abs(dj2), Math.abs(dj3));

    const transitionDuration = Math.max(1.5, maxDiff / 15.0);
    const transitionSteps = Math.ceil(transitionDuration * rateHz);

    for (let k = 1; k <= transitionSteps; k++) {
      const t = k / transitionSteps;
      const smooth_t = (1.0 - Math.cos(t * Math.PI)) / 2.0;
      jointTrajectory.push({
        j1: physState.j1 + dj1 * smooth_t,
        j2: physState.j2 + dj2 * smooth_t,
        j3: physState.j3 + dj3 * smooth_t,
        laser: false
      });
    }

    // 3. Interpolación cartesiana lineal
    const tracingPoints = [];
    tracingPoints.push(points[0]);
    for (let i = 1; i < points.length; i++) {
      const pPrev = points[i - 1];
      const pCurr = points[i];
      const dx = pCurr.x - pPrev.x;
      const dy = pCurr.y - pPrev.y;
      const dz = pCurr.z - pPrev.z;
      const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

      const stepSize = pCurr.laser ? stepSizeDraw : stepSizeApproach;

      if (dist > stepSize) {
        const steps = Math.ceil(dist / stepSize);
        for (let k = 1; k <= steps; k++) {
          const t = k / steps;
          tracingPoints.push({
            x: pPrev.x + dx * t,
            y: pPrev.y + dy * t,
            z: pPrev.z + dz * t,
            laser: pCurr.laser
          });
        }
      } else {
        tracingPoints.push(pCurr);
      }
    }

    // 4. Resolver cinemática inversa y añadir a la trayectoria
    let validTracingCount = 0;
    for (let p of tracingPoints) {
      const result = solveIK(p.x, p.y, p.z, L2 + 200.0);
      if (result && !result.error && !checkCollision(result.j2, result.j3)) {
        jointTrajectory.push({
          j1: result.j1,
          j2: result.j2,
          j3: result.j3,
          laser: p.laser
        });
        validTracingCount++;
      }
    }

    if (validTracingCount === 0) {
      reject(new Error("Todos los puntos calculados están fuera de alcance o en colisión."));
      return;
    }
    
    let index = 0;
    
    // Controlar el láser físico inicial
    const firstLaser = jointTrajectory[0].laser;
    updatePhysLaserUI(firstLaser);

    while (index < jointTrajectory.length && !physState.emergencyStop && physState.executingTrajectory) {
      const startTime = Date.now();
      const currentJoints = jointTrajectory[index];
      
      // Controlar el láser en cada punto
      const targetLaser = currentJoints.laser;
      if (physState.laserActive !== targetLaser) {
        updatePhysLaserUI(targetLaser);
      }

      physState.j1 = currentJoints.j1;
      physState.j2 = currentJoints.j2;
      physState.j3 = currentJoints.j3;
      
      // Actualizar la interfaz física
      for (let num = 1; num <= 3; num++) {
        const rangeEl = document.getElementById(`phys-input-range-j${num}`);
        const numberEl = document.getElementById(`phys-input-number-j${num}`);
        if (rangeEl) rangeEl.value = physState[`j${num}`];
        if (numberEl) numberEl.value = physState[`j${num}`].toFixed(1);
      }
      
      // Calcular FK física y dibujar
      updatePhysKinematics();
      drawPhysRobot();
      
      try {
        const res = await fetch('/api/physical/move', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({
            j1: physState.j1,
            j2: physState.j2,
            j3: physState.j3,
            laser: physState.laserActive ? 1 : 0
          })
        });
        const data = await res.json();
        const logEl = document.getElementById('phys-esp-log');
        if (logEl && data.response) {
          logEl.textContent = data.response;
          if (data.status === 'ok') {
            logEl.style.color = '#34c759';
          } else {
            logEl.style.color = '#ff3b30';
          }
        }
      } catch (err) {
        console.error("Error al mover brazo fisico:", err);
      }
      
      index++;

      const elapsed = Date.now() - startTime;
      const waitTime = Math.max(0, intervalMs - elapsed);
      if (waitTime > 0 && index < jointTrajectory.length && !physState.emergencyStop && physState.executingTrajectory) {
        await new Promise(resolveWait => setTimeout(resolveWait, waitTime));
      }
    }

    // Desactivar el láser físico al finalizar o abortar/cancelar
    updatePhysLaserUI(false);

    if (physState.emergencyStop) {
      reject(new Error("Ejecución abortada por parada de emergencia."));
    } else if (!physState.executingTrajectory) {
      reject(new Error("Ejecución cancelada."));
    } else {
      resolve();
    }
  });
}

function initPhysical() {
  // Pestañas
  const tabSimulation = document.getElementById('tab-simulation');
  const tabPhysical = document.getElementById('tab-physical');
  const viewSimulation = document.getElementById('view-simulation');
  const viewPhysical = document.getElementById('view-physical');
  
  if (tabSimulation && tabPhysical && viewSimulation && viewPhysical) {
    tabSimulation.addEventListener('click', () => {
      if (physState.executingTrajectory) {
        physState.executingTrajectory = false;
      }
      viewSimulation.style.display = '';
      viewPhysical.style.display = 'none';
      tabSimulation.classList.add('active');
      tabPhysical.classList.remove('active');
      
      const statusText = document.getElementById('system-status-text');
      const statusDot = document.getElementById('system-status-dot');
      if (statusText && statusDot && !state.emergencyStop) {
        statusText.textContent = 'MODO SIMULACIÓN ACTIVO';
        statusDot.className = 'status-dot simulation-mode';
      }
      stopPollingPhysicalStatus();

      // Detener flujo de cámara real al salir del modo físico
      const physImgCameraStream = document.getElementById('phys-camera-stream-img');
      if (physImgCameraStream) physImgCameraStream.src = '';
      const physBtnTab2d = document.getElementById('phys-btn-tab-2d');
      const physBtnTabCamera = document.getElementById('phys-btn-tab-camera');
      const physContainer2d = document.getElementById('phys-container-2d');
      const physContainerCamera = document.getElementById('phys-container-camera');
      if (physBtnTab2d && physBtnTabCamera && physContainer2d && physContainerCamera) {
        physBtnTab2d.classList.add('active');
        physBtnTabCamera.classList.remove('active');
        physContainer2d.style.display = 'block';
        physContainerCamera.style.display = 'none';
      }
    });

    tabPhysical.addEventListener('click', () => {
      if (state.executingTrajectory) {
        state.executingTrajectory = false;
      }
      viewSimulation.style.display = 'none';
      viewPhysical.style.display = '';
      tabPhysical.classList.add('active');
      tabSimulation.classList.remove('active');
      
      const statusText = document.getElementById('system-status-text');
      const statusDot = document.getElementById('system-status-dot');
      if (statusText && statusDot && !state.emergencyStop) {
        statusText.textContent = 'MODO BRAZO FÍSICO';
        statusDot.className = 'status-dot';
      }
      
      drawPhysRobot();
      pollPhysicalStatus();
      loadSerialPorts(); // Cargar la lista al cambiar a esta pestaña

      // Detener flujo de cámara de simulación al salir del modo simulación
      const imgCameraStream = document.getElementById('camera-stream-img');
      if (imgCameraStream) imgCameraStream.src = '';
      const btnTab2d = document.getElementById('btn-tab-2d');
      const btnTabCamera = document.getElementById('btn-tab-camera');
      const container2d = document.getElementById('container-2d');
      const containerCamera = document.getElementById('container-camera');
      if (btnTab2d && btnTabCamera && container2d && containerCamera) {
        btnTab2d.classList.add('active');
        btnTabCamera.classList.remove('active');
        container2d.style.display = 'block';
        containerCamera.style.display = 'none';
      }
    });
  }

  // Cargar lista de puertos seriales
  function loadSerialPorts() {
    const selectEl = document.getElementById('phys-port-select');
    if (!selectEl) return;

    fetch('/api/physical/list_ports')
      .then(res => res.json())
      .then(data => {
        const currentVal = selectEl.value;
        selectEl.innerHTML = '';
        
        const autoOpt = document.createElement('option');
        autoOpt.value = '';
        autoOpt.textContent = 'Auto (detección automática)';
        selectEl.appendChild(autoOpt);
        
        if (data.ports && data.ports.length > 0) {
          data.ports.forEach(port => {
            const opt = document.createElement('option');
            opt.value = port;
            opt.textContent = port;
            selectEl.appendChild(opt);
          });
        }
        
        selectEl.value = currentVal;
      })
      .catch(err => console.error("Error al cargar puertos seriales:", err));
  }

  // Botón de refrescar puertos
  const btnRefreshPorts = document.getElementById('phys-btn-refresh-ports');
  if (btnRefreshPorts) {
    btnRefreshPorts.addEventListener('click', loadSerialPorts);
  }

  // Cargar puertos al iniciar
  loadSerialPorts();

  // Conexión
  const btnConnect = document.getElementById('phys-btn-connect');
  if (btnConnect) {
    btnConnect.addEventListener('click', () => {
      const portSelect = document.getElementById('phys-port-select');
      const portVal = portSelect ? portSelect.value : '';
      
      const payload = {};
      if (physState.connected) {
        payload.disconnect = true;
      } else if (portVal) {
        payload.port = portVal;
      }
      
      btnConnect.disabled = true;
      btnConnect.textContent = physState.connected ? 'DESCONECTANDO...' : 'CONECTANDO...';
      
      fetch('/api/physical/connect', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(payload)
      })
      .then(res => res.json())
      .then(data => {
        btnConnect.disabled = false;
        updatePhysConnectionUI(data);
      })
      .catch(err => {
        console.error("Error al cambiar conexión serial:", err);
        btnConnect.disabled = false;
        btnConnect.textContent = physState.connected ? 'DESCONECTAR' : 'CONECTAR';
        const errorMsg = document.getElementById('phys-error-msg');
        if (errorMsg) {
          errorMsg.textContent = `Error de red: ${err.message}`;
          errorMsg.style.display = 'block';
        }
      });
    });
  }

  // Sliders e Inputs de Articulaciones
  [1, 2, 3].forEach(num => {
    const rangeInput = document.getElementById(`phys-input-range-j${num}`);
    const numberInput = document.getElementById(`phys-input-number-j${num}`);

    if (rangeInput && numberInput) {
      rangeInput.addEventListener('input', (e) => {
        updatePhysJoint(num, e.target.value, 'range');
      });

      numberInput.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        if (!isNaN(val)) {
          const capped = Math.max(jointsConfig[num].min, Math.min(jointsConfig[num].max, val));
          physState['j' + num] = capped;
          rangeInput.value = capped;
          
          if (physIkAnimationId) {
            cancelAnimationFrame(physIkAnimationId);
            physIkAnimationId = null;
          }
          
          updatePhysKinematics();
          drawPhysRobot();
          syncPhysIkTargetInputs();
        }
      });

      numberInput.addEventListener('change', (e) => {
        let val = parseFloat(e.target.value);
        if (isNaN(val)) val = 0;
        const capped = Math.max(jointsConfig[num].min, Math.min(jointsConfig[num].max, val));
        e.target.value = capped;
        updatePhysJoint(num, capped, 'number');
      });
    }
  });

  // Láser Toggle
  const btnLaserToggle = document.getElementById('phys-btn-laser-toggle');
  if (btnLaserToggle) {
    btnLaserToggle.addEventListener('click', () => {
      if (physState.emergencyStop) return;
      updatePhysLaserUI(!physState.laserActive);
      drawPhysRobot();
      updatePhysKinematics(); // Enviar comando inmediato al ESP32
    });
  }

  // Restablecer posición cero
  const btnReset = document.getElementById('phys-btn-reset');
  if (btnReset) {
    btnReset.addEventListener('click', () => {
      if (physState.emergencyStop) return;
      
      if (physIkAnimationId) {
        cancelAnimationFrame(physIkAnimationId);
        physIkAnimationId = null;
      }
      
      animatePhysToJoints(0, 0, 0);
      
      document.getElementById('phys-input-ik-x').value = 322;
      document.getElementById('phys-input-ik-y').value = 7;
      document.getElementById('phys-input-ik-z').value = 96;
      
      const ikStatus = document.getElementById('phys-ik-status');
      if (ikStatus) {
        ikStatus.textContent = 'ALCANCE OK';
        ikStatus.className = 'ik-status-badge ik-status-ok';
      }

      updatePhysLaserUI(false);
    });
  }

  // Parada de Emergencia Física
  const physBtnEmergency = document.getElementById('phys-btn-emergency');
  if (physBtnEmergency) {
    physBtnEmergency.addEventListener('click', () => {
      setEmergencyState(!state.emergencyStop);
    });
  }

  // Cinemática Inversa (IK) - Aplicar
  const btnApplyIk = document.getElementById('phys-btn-apply-ik');
  const ikStatus = document.getElementById('phys-ik-status');

  if (btnApplyIk && ikStatus) {
    btnApplyIk.addEventListener('click', () => {
      if (physState.emergencyStop) return;

      const x = document.getElementById('phys-input-ik-x').value;
      const y = document.getElementById('phys-input-ik-y').value;
      const z = document.getElementById('phys-input-ik-z').value;

      const result = solveIK(x, y, z);

      if (result === null || result.error) {
        ikStatus.textContent = 'FUERA DE ALCANCE';
        ikStatus.className = 'ik-status-badge ik-status-error';
        
        ikStatus.style.animation = 'none';
        void ikStatus.offsetWidth;
        ikStatus.style.animation = '';
      } else if (checkCollision(result.j2, result.j3)) {
        ikStatus.textContent = 'COLISION';
        ikStatus.className = 'ik-status-badge ik-status-error';
        
        ikStatus.style.animation = 'none';
        void ikStatus.offsetWidth;
        ikStatus.style.animation = '';
      } else {
        ikStatus.textContent = 'ALCANCE OK';
        ikStatus.className = 'ik-status-badge ik-status-ok';
        animatePhysToJoints(result.j1, result.j2, result.j3);
      }
    });
  }

  // Live IK Status
  ['phys-input-ik-x', 'phys-input-ik-y', 'phys-input-ik-z'].forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('input', () => {
        if (physState.emergencyStop) return;

        const x = document.getElementById('phys-input-ik-x').value;
        const y = document.getElementById('phys-input-ik-y').value;
        const z = document.getElementById('phys-input-ik-z').value;

        const result = solveIK(x, y, z);

        if (result === null || result.error) {
          ikStatus.textContent = 'FUERA DE ALCANCE';
          ikStatus.className = 'ik-status-badge ik-status-error';
        } else if (checkCollision(result.j2, result.j3)) {
          ikStatus.textContent = 'COLISION';
          ikStatus.className = 'ik-status-badge ik-status-error';
        } else {
          ikStatus.textContent = 'ALCANCE OK';
          ikStatus.className = 'ik-status-badge ik-status-ok';
        }
      });
    }
  });

  // --- INTEGRACIÓN DE CÁMARA REAL (CÁLCULO Y EJECUCIÓN) ---
  let physCalculatedPoints = [];
  const physBtnCalculatePath = document.getElementById('phys-btn-calculate-path');
  const physPathResultContainer = document.getElementById('phys-path-result-container');
  const physPathResultImg = document.getElementById('phys-path-result-img');
  const physBtnExecuteCalculated = document.getElementById('phys-btn-execute-calculated');
  const physCameraSelectEl = document.getElementById('phys-camera-select');

  if (physBtnCalculatePath && physPathResultContainer && physPathResultImg && physBtnExecuteCalculated) {
    physBtnCalculatePath.addEventListener('click', () => {
      if (physState.emergencyStop) return;
      physBtnCalculatePath.disabled = true;
      physBtnCalculatePath.textContent = 'PROCESANDO...';
      physBtnCalculatePath.style.background = '#2c2c3e';
      physPathResultContainer.style.display = 'none';

      const idx = physCameraSelectEl ? physCameraSelectEl.value : 'auto';

      fetch('/api/physical/calculate_path', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ index: idx })
      })
      .then(res => {
        if (!res.ok) {
          return res.json().then(err => { throw new Error(err.error || 'Error desconocido') });
        }
        return res.json();
      })
      .then(data => {
        physBtnCalculatePath.disabled = false;
        physBtnCalculatePath.textContent = 'CALCULAR RECORRIDO';
        physBtnCalculatePath.style.background = '#007aff';

        if (data.status === 'success' && data.points && data.points.length > 0) {
          physCalculatedPoints = data.points;
          physPathResultImg.src = data.image;
          physPathResultContainer.style.display = 'block';
          console.log(`[Físico] Puntos calculados con éxito: ${physCalculatedPoints.length} puntos.`);
        }
      })
      .catch(err => {
        console.error("[Físico] Error al calcular recorrido:", err);
        alert("Fallo al calcular recorrido en cámara física: " + err.message);
        physBtnCalculatePath.disabled = false;
        physBtnCalculatePath.textContent = 'CALCULAR RECORRIDO';
        physBtnCalculatePath.style.background = '#007aff';
      });
    });

    physBtnExecuteCalculated.addEventListener('click', () => {
      if (physCalculatedPoints.length === 0) return;
      if (physState.emergencyStop) return;

      physState.executingTrajectory = true;
      disablePhysInputs(true);
      physBtnExecuteCalculated.disabled = true;
      physBtnExecuteCalculated.textContent = 'EJECUTANDO...';
      physBtnExecuteCalculated.style.background = '#2c2c3e';

      executePhysTrajectory(physCalculatedPoints)
        .then(() => {
          physState.executingTrajectory = false;
          if (!physState.emergencyStop) {
            disablePhysInputs(false);
          }
          physBtnExecuteCalculated.disabled = physState.emergencyStop;
          physBtnExecuteCalculated.textContent = 'EJECUTAR RECORRIDO EN BRAZO FÍSICO';
          physBtnExecuteCalculated.style.background = '#34c759';
        })
        .catch(err => {
          physState.executingTrajectory = false;
          console.error("[Físico] Error en ejecución de trayectoria:", err);
          alert("Error durante la ejecución física del recorrido: " + err.message);
          if (!physState.emergencyStop) {
            disablePhysInputs(false);
          }
          physBtnExecuteCalculated.disabled = physState.emergencyStop;
          physBtnExecuteCalculated.textContent = 'EJECUTAR RECORRIDO EN BRAZO FÍSICO';
          physBtnExecuteCalculated.style.background = '#34c759';
        });
    });
  }

  // Dibujo inicial
  updatePhysKinematics();
  drawPhysRobot();
  syncPhysIkTargetInputs();
}

// Iniciar aplicación
init();
initPhysical();

