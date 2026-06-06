const state = {
  j1: 0,
  j2: 0,
  j3: 0,
  laserActive: false,
  laserPower: 0,
  emergencyStop: false
};

const d1 = 60.0;
const d2 = 36.173;
const a2 = 14.915;
const L1 = 146.190;
const L2 = 160.823;
const d4 = -4.0;
const phi2 = 0.0;
const phi3 = 0.0;
const laserLength = 0.0;

const S = 0.13;
const y_base = 155;

const jointsConfig = {
  1: { min: -165, max: 165, name: 'j1' },
  2: { min: -50.9, max: 169.0, name: 'j2' },
  3: { min: -180.0, max: 0.0, name: 'j3' }
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

  sendJointsToROS(state.j1, state.j2, state.j3);

  return { x_plane, z_plane, x, y, z };
}

/*
 * Resuelve el modelo cinemático inverso para determinar las posiciones de consigna
 * de los servomotores a partir del objetivo cartesiano (x, y, z) deseado.
 */
function solveIK(x, y, z) {
  const targetX = parseFloat(x);
  const targetY = parseFloat(y);
  const targetZ = parseFloat(z);

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

  const psi_numerator = xc * xc + zc * zc - L1 * L1 - L2 * L2;
  const psi_denominator = 2.0 * L1 * L2;
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

    const k1 = L1 + L2 * Math.cos(theta3_star);
    const k2 = L2 * Math.sin(theta3_star);
    const theta2_star = Math.atan2(k1 * zc - k2 * xc, k1 * xc + k2 * zc);

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
function updateJoint(num, value, source) {
  if (state.emergencyStop) return;

  const config = jointsConfig[num];
  let parsed = parseFloat(value);
  
  if (isNaN(parsed)) {
    if (source === 'number') return;
    parsed = 0;
  }

  const cappedValue = Math.max(config.min, Math.min(config.max, parsed));
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
    
    console.warn("IK Falló:", result ? result.msg : "Fuera de límites");
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
  
  drawRobot();
});

btnEmergency.addEventListener('click', () => {
  state.emergencyStop = !state.emergencyStop;
  
  if (state.emergencyStop) {
    document.body.classList.add('emergency-active');
    btnEmergency.querySelector('span').textContent = 'RESTABLECER Y REINICIAR';
    
    systemStatusText.textContent = 'EMERGENCIA: SISTEMA BLOQUEADO';
    systemStatusText.style.color = '#ff3b30';
    systemStatusDot.style.backgroundColor = '#ff3b30';
    systemStatusDot.style.boxShadow = '0 0 10px #ff3b30';
    
    if (ikAnimationId) {
      cancelAnimationFrame(ikAnimationId);
      ikAnimationId = null;
    }

    state.laserActive = false;
    laserIndicator.textContent = 'APAGADO';
    laserIndicator.className = 'laser-status-badge laser-off';
    btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
    btnLaserToggle.classList.remove('btn-laser-active');
    
    disableInputs(true);
    drawRobot();
  } else {
    document.body.classList.remove('emergency-active');
    btnEmergency.querySelector('span').textContent = 'PARADA DE EMBALAJES';
    
    systemStatusText.textContent = 'MODO SIMULACIÓN ACTIVO';
    systemStatusText.style.color = '';
    systemStatusDot.style.backgroundColor = '';
    systemStatusDot.style.boxShadow = '';
    
    disableInputs(false);
    drawRobot();
  }
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
  document.getElementById('input-ik-y').value = 4;
  document.getElementById('input-ik-z').value = 96;
  
  ikStatus.textContent = 'ALCANCE OK';
  ikStatus.className = 'ik-status-badge ik-status-ok';

  state.laserActive = false;
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
    '.joint-slider, .joint-value-input input, #input-range-laser, #input-ik-x, #input-ik-y, #input-ik-z, #btn-apply-ik, #btn-reset-home'
  );
  controls.forEach(el => {
    el.disabled = disable;
  });
  
  btnLaserToggle.disabled = disable;
}

/*
 * Inicializa la interfaz de usuario, cargando la cinemática por defecto,
 * dibujando el canvas y sincronizando las coordenadas de entrada.
 */
function init() {
  updateKinematics();
  drawRobot();
  syncIkTargetInputs();
  
  window.addEventListener('load', () => {
    drawRobot();
  });
}

init();
