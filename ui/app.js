// ==================== ESTADO GLOBAL DE LA INTERFAZ ====================
const state = {
  j1: 0,          // Rotación de base (Grados)
  j2: 0,          // Hombro (Grados)
  j3: 0,          // Codo (Grados)
  laserActive: false,
  laserPower: 0,
  emergencyStop: false
};

// ==================== CONFIGURACIÓN FÍSICA Y DIMENSIONES (ABB IRB 120) ====================
const d1 = 290.0;          // Altura base a hombro (mm)
const L2 = 270.0;          // Longitud hombro a codo (mm)
const L3 = 70.0;           // Longitud codo a Joint 4 (mm)
const L4 = 374.0;          // Longitud Joint 4 a Flange (mm)
const laserLength = 120.0; // Distancia del flange a la punta de la boquilla láser (mm)

// Configuración de dibujo en Canvas
const S = 0.13;            // Factor de escala (píxeles / mm)
const y_base = 155;        // Origen Y del robot en canvas (suelo)

// Límites de las articulaciones
const jointsConfig = {
  1: { min: -165, max: 165, name: 'j1' },
  2: { min: -110, max: 110, name: 'j2' },
  3: { min: -110, max: 70, name: 'j3' }
};

// Identificador de animación activa de IK
let ikAnimationId = null;

// ==================== ELEMENTOS DEL DOM ====================
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

// ==================== CINEMÁTICA DIRECTA (FK) ====================
function updateKinematics() {
  // Convertir ángulos a radianes
  const theta1 = state.j1 * Math.PI / 180;
  const theta2 = state.j2 * Math.PI / 180;
  const theta3 = state.j3 * Math.PI / 180;

  // Coordenadas en el plano del brazo (X_plane, Z_plane)
  const x_plane = L2 * Math.sin(theta2) + L3 * Math.sin(theta2 + theta3) + L4 * Math.cos(theta2 + theta3);
  const z_plane = d1 + L2 * Math.cos(theta2) + L3 * Math.cos(theta2 + theta3) - L4 * Math.sin(theta2 + theta3);

  // Coordenadas 3D con rotación de base (J1)
  const x = x_plane * Math.cos(theta1);
  const y = x_plane * Math.sin(theta1);
  const z = z_plane;

  // Actualizar valores en la interfaz
  document.getElementById('coord-x').textContent = `${x.toFixed(1)} mm`;
  document.getElementById('coord-y').textContent = `${y.toFixed(1)} mm`;
  document.getElementById('coord-z').textContent = `${z.toFixed(1)} mm`;

  return { x_plane, z_plane, x, y, z };
}

// ==================== CINEMÁTICA INVERSA (IK) ====================
function solveIK(x, y, z) {
  const targetX = parseFloat(x);
  const targetY = parseFloat(y);
  const targetZ = parseFloat(z);

  if (isNaN(targetX) || isNaN(targetY) || isNaN(targetZ)) {
    return { error: 'VALORES_INVALIDOS', msg: 'Ingresa valores numéricos válidos.' };
  }

  // 1. Ángulo de la Base J1 (atan2 de Y y X)
  const theta1 = Math.atan2(targetY, targetX);
  const j1 = theta1 * 180 / Math.PI;

  // Validar límites de J1
  if (j1 < jointsConfig[1].min || j1 > jointsConfig[1].max) {
    return { error: 'FUERA DE ALCANCE', msg: 'Rotación base J1 fuera de límites (-165° a 165°).' };
  }

  // 2. Proyección en el plano del brazo
  const x_plane = Math.sqrt(targetX * targetX + targetY * targetY);
  const z_plane = targetZ;

  // Desplazar origen respecto al hombro (J2) en Z (d1 = 290mm)
  const xc = x_plane;
  const zc = z_plane - d1;

  // Coeficientes para la ecuación: C1 * cos(phi) + C2 * sin(phi) = D
  // Donde phi = theta2 + theta3 (Orientación absoluta del antebrazo)
  const C1 = 2 * (xc * L4 + zc * L3);
  const C2 = 2 * (xc * L3 - zc * L4);
  const D = xc * xc + zc * zc + L3 * L3 + L4 * L4 - L2 * L2;
  const R = Math.sqrt(C1 * C1 + C2 * C2);

  // Evitar división por cero en configuraciones singulares
  if (R < 1e-5) {
    return { error: 'SINGULARIDAD', msg: 'Punto de singularidad mecánica.' };
  }

  // Verificar si la coordenada está fuera del alcance geométrico
  if (Math.abs(D) > R) {
    return { error: 'FUERA DE ALCANCE', msg: 'Coordenadas fuera del alcance del brazo.' };
  }

  const gamma = Math.atan2(C2, C1);
  const acosVal = Math.acos(D / R);

  // Probar ambas soluciones geométricas (codo arriba y codo abajo)
  const solutions = [1, -1];
  for (let s of solutions) {
    const phi = gamma + s * acosVal;

    // Calcular theta2 y theta3
    const A = xc - L4 * Math.cos(phi) - L3 * Math.sin(phi);
    const B = zc - L3 * Math.cos(phi) + L4 * Math.sin(phi);
    const theta2 = Math.atan2(A, B);
    const theta3 = phi - theta2;

    const j2 = theta2 * 180 / Math.PI;
    const j3 = theta3 * 180 / Math.PI;

    // Verificar que los ángulos obtenidos respeten los límites de diseño de J2 y J3
    if (j2 >= jointsConfig[2].min && j2 <= jointsConfig[2].max &&
        j3 >= jointsConfig[3].min && j3 <= jointsConfig[3].max) {
      return { j1, j2, j3 };
    }
  }

  return { error: 'FUERA DE ALCANCE', msg: 'Posición cinemáticamente inalcanzable por límites de articulación.' };
}

// ==================== ANIMACIÓN SUAVE HACIA OBJETIVO IK ====================
function animateToJoints(targetJ1, targetJ2, targetJ3) {
  if (ikAnimationId) {
    cancelAnimationFrame(ikAnimationId);
  }

  const startJ1 = state.j1;
  const startJ2 = state.j2;
  const startJ3 = state.j3;

  const duration = 600; // Duración de movimiento en milisegundos
  const startTime = performance.now();

  function step(now) {
    const elapsed = now - startTime;
    const progress = Math.min(elapsed / duration, 1);

    // Easing: deceleración suave al final (Cubic Ease-Out)
    const ease = 1 - Math.pow(1 - progress, 3);

    state.j1 = startJ1 + (targetJ1 - startJ1) * ease;
    state.j2 = startJ2 + (targetJ2 - startJ2) * ease;
    state.j3 = startJ3 + (targetJ3 - startJ3) * ease;

    // Sincronizar elementos interactivos del DOM con el estado interpolado
    for (let num = 1; num <= 3; num++) {
      const valName = jointsConfig[num].name;
      document.getElementById(`input-range-j${num}`).value = state[valName];
      document.getElementById(`input-number-j${num}`).value = state[valName].toFixed(1);
    }

    // Actualizar coordenadas y volver a dibujar
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

// ==================== RENDERIZADO EN CANVAS (VISTA LATERAL Y SUPERIOR) ====================
function drawRobot() {
  // Limpiar lienzo
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 1. Dibujar rejilla (Blueprint Grid) a lo largo del canvas
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

  // Dibujar línea divisoria central
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
  ctx.lineWidth = 1;
  ctx.setLineDash([2, 4]);
  ctx.beginPath();
  ctx.moveTo(220, 0);
  ctx.lineTo(220, canvas.height);
  ctx.stroke();
  ctx.setLineDash([]);

  // Ángulos y variables auxiliares
  const t2 = state.j2 * Math.PI / 180;
  const t3 = state.j3 * Math.PI / 180;
  const alpha = t2 + t3; // Ángulo absoluto del antebrazo
  const sinA = Math.sin(alpha);

  // ==================== MITAD IZQUIERDA: VISTA LATERAL (X-Z) ====================
  const x_base_left = 60;
  
  // Dibujar línea de la mesa
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(10, y_base);
  ctx.lineTo(210, y_base);
  ctx.stroke();
  ctx.setLineDash([]);

  // Etiquetas
  ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.font = '500 8px Outfit';
  ctx.fillText('VISTA LATERAL (X-Z)', 12, 16);
  ctx.fillText('MESA', 180, y_base - 5);

  // Calcular puntos de articulaciones en pixeles
  const x0 = x_base_left;
  const y0 = y_base;

  const x1 = x_base_left;
  const y1 = y_base - d1 * S;

  const x2 = x_base_left + (L2 * Math.sin(t2)) * S;
  const y2 = y1 - (L2 * Math.cos(t2)) * S;

  // Calculamos la posición de la brida de forma matemática igual
  const x3 = x2 + (L3 * Math.sin(alpha)) * S;
  const y3 = y2 - (L3 * Math.cos(alpha)) * S;
  const x4 = x3 + (L4 * Math.cos(alpha)) * S;
  const y4 = y3 + (L4 * Math.sin(alpha)) * S;

  // Ángulo visual recto del antebrazo (de codo x2,y2 a brida x4,y4) para eliminar la L
  const forearmAngle = Math.atan2(y4 - y2, x4 - x2);
  const sinA_beam = Math.sin(forearmAngle);

  const xtip = x4 + (laserLength * Math.cos(forearmAngle)) * S;
  const ytip = y4 + (laserLength * Math.sin(forearmAngle)) * S;

  // Dibujar haz láser (Vista lateral)
  let hitsGroundSide = false;
  let x_beam_end_side = xtip;
  let y_beam_end_side = ytip;

  if (state.laserActive && !state.emergencyStop) {
    if (sinA_beam > 0.001) {
      // Calcular corte con el suelo
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

    // Haz láser
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

    // Chispa
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

  // Dibujar mecánica del brazo (Vista lateral)
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  // Base física fija
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(x_base_left - 14, y_base);
  ctx.lineTo(x_base_left + 14, y_base);
  ctx.stroke();

  // Eslabón 1: Columna base
  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 9;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();

  // Eslabón 2: Hombro a Codo
  ctx.strokeStyle = '#ff7300';
  ctx.lineWidth = 7.5;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  // Eslabón 3 y 4 combinados: Antebrazo recto (elimina la L completamente)
  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x4, y4);
  ctx.stroke();

  // Articulaciones en vista lateral (SOLO J2 hombro y J3 codo para evitar confusión)
  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#0a0a14';
  ctx.lineWidth = 1.5;

  const drawJoint = (x, y, r) => {
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  };

  drawJoint(x1, y1, 4);   // Eje Hombro J2
  drawJoint(x2, y2, 3.5); // Eje Codo J3

  // Herramienta láser vista lateral
  ctx.save();
  ctx.translate(x4, y4);
  ctx.rotate(forearmAngle);
  ctx.fillStyle = '#a0a3b5'; ctx.fillRect(0, -5, 2, 10);
  ctx.fillStyle = '#22232b'; ctx.fillRect(2, -4, 9, 8);
  ctx.fillStyle = '#7a7d8c'; ctx.fillRect(11, -2.5, 3, 5);
  ctx.fillStyle = '#bfa15c'; ctx.fillRect(14, -1, 2.5, 2);
  ctx.restore();

  // ==================== MITAD DERECHA: VISTA SUPERIOR (X-Y) ====================
  const x_base_right = 330;
  const y_base_right = 90;

  // Etiquetas
  ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.font = '500 8px Outfit';
  ctx.fillText('VISTA SUPERIOR (X-Y)', 232, 16);

  // Rejilla de círculos de alcance
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
  ctx.lineWidth = 1;
  for (let r = 25; r <= 85; r += 20) {
    ctx.beginPath();
    ctx.arc(x_base_right, y_base_right, r, 0, Math.PI * 2);
    ctx.stroke();
  }

  // Límites mecánicos J1
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

  // Cálculo de distancias en el plano para proyección superior
  const t1 = state.j1 * Math.PI / 180;
  const X2 = L2 * Math.sin(t2);
  const X3 = X2 + L3 * Math.sin(alpha);
  const X4 = X3 + L4 * Math.cos(alpha);
  const X_tip = X3 + (L4 + laserLength) * Math.cos(alpha);

  // Proyecciones top-down
  const x_j3_top = x_base_right + X2 * S * Math.cos(-t1);
  const y_j3_top = y_base_right + X2 * S * Math.sin(-t1);

  const x_flange_top = x_base_right + X4 * S * Math.cos(-t1);
  const y_flange_top = y_base_right + X4 * S * Math.sin(-t1);

  const x_tip_top = x_base_right + X_tip * S * Math.cos(-t1);
  const y_tip_top = y_base_right + X_tip * S * Math.sin(-t1);

  // Haz láser en vista superior
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

    // Haz láser top down
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

    // Punto chispa top-down
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

  // Base física cilíndrica top-down
  ctx.fillStyle = '#1e202b';
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(x_base_right, y_base_right, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // Eslabón hombro a codo top-down (Hombro)
  ctx.strokeStyle = '#ff7300';
  ctx.lineWidth = 6.5;
  ctx.beginPath();
  ctx.moveTo(x_base_right, y_base_right);
  ctx.lineTo(x_j3_top, y_j3_top);
  ctx.stroke();

  // Eslabón codo a flange top-down (Antebrazo)
  ctx.strokeStyle = '#e65c00';
  ctx.lineWidth = 4.5;
  ctx.beginPath();
  ctx.moveTo(x_j3_top, y_j3_top);
  ctx.lineTo(x_flange_top, y_flange_top);
  ctx.stroke();

  // Herramienta láser top-down
  ctx.strokeStyle = '#7a7d8c';
  ctx.lineWidth = 3.5;
  ctx.beginPath();
  ctx.moveTo(x_flange_top, y_flange_top);
  ctx.lineTo(x_tip_top, y_tip_top);
  ctx.stroke();

  // Puntos articulares vista superior (J1 base y J3 codo)
  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#0a0a14';
  ctx.lineWidth = 1.2;

  drawJoint(x_base_right, y_base_right, 3); // J1
  drawJoint(x_j3_top, y_j3_top, 2.5);      // J3
}

// ==================== CONTROL Y SINCRONIZACIÓN DE ARTICULACIONES ====================
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

  if (source !== 'range') {
    document.getElementById(`input-range-j${num}`).value = cappedValue;
  }
  if (source !== 'number') {
    document.getElementById(`input-number-j${num}`).value = cappedValue;
  }

  // Al mover manualmente sliders, cancelar cualquier animación IK activa
  if (ikAnimationId) {
    cancelAnimationFrame(ikAnimationId);
    ikAnimationId = null;
  }

  updateKinematics();
  drawRobot();

  // Actualizar los inputs de destino IK con la posición actual calculada
  syncIkTargetInputs();
}

// Sincronizar inputs de IK con la posición física actual
function syncIkTargetInputs() {
  const currentCoords = updateKinematics();
  document.getElementById('input-ik-x').value = Math.round(currentCoords.x);
  document.getElementById('input-ik-y').value = Math.round(currentCoords.y);
  document.getElementById('input-ik-z').value = Math.round(currentCoords.z);
  
  // Restablecer indicador a OK por defecto
  ikStatus.textContent = 'ALCANCE OK';
  ikStatus.className = 'ik-status-badge ik-status-ok';
}

// Configurar Eventos para J1, J2 y J3
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
      
      // Cancelar cualquier animación IK activa al escribir directamente
      if (ikAnimationId) {
        cancelAnimationFrame(ikAnimationId);
        ikAnimationId = null;
      }
      
      updateKinematics();
      drawRobot();
      
      // Sincronizar inputs de destino IK
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

// ==================== EVENTOS Y ENTRADAS DE CINEMÁTICA INVERSA (IK) ====================
btnApplyIk.addEventListener('click', () => {
  if (state.emergencyStop) return;

  const x = document.getElementById('input-ik-x').value;
  const y = document.getElementById('input-ik-y').value;
  const z = document.getElementById('input-ik-z').value;

  const result = solveIK(x, y, z);

  if (result === null || result.error) {
    // Mostrar insignia de error con animación de sacudida
    ikStatus.textContent = 'FUERA DE ALCANCE';
    ikStatus.className = 'ik-status-badge ik-status-error';
    
    ikStatus.style.animation = 'none';
    void ikStatus.offsetWidth; // Forzar reflow para reiniciar la animación
    ikStatus.style.animation = '';
    
    console.warn("IK Falló:", result ? result.msg : "Fuera de límites");
  } else {
    // Mostrar insignia de OK
    ikStatus.textContent = 'ALCANCE OK';
    ikStatus.className = 'ik-status-badge ik-status-ok';
    
    // Mover el robot de forma suave
    animateToJoints(result.j1, result.j2, result.j3);
  }
});

// Verificación en tiempo real del alcance al modificar los inputs IK
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

// ==================== CONTROL DEL LÁSER ====================
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

// ==================== SISTEMA DE PARADA DE EMERGENCIA ====================
btnEmergency.addEventListener('click', () => {
  state.emergencyStop = !state.emergencyStop;
  
  if (state.emergencyStop) {
    // Activar bloqueo total
    document.body.classList.add('emergency-active');
    btnEmergency.querySelector('span').textContent = 'RESTABLECER Y REINICIAR';
    
    systemStatusText.textContent = 'EMERGENCIA: SISTEMA BLOQUEADO';
    systemStatusText.style.color = '#ff3b30';
    systemStatusDot.style.backgroundColor = '#ff3b30';
    systemStatusDot.style.boxShadow = '0 0 10px #ff3b30';
    
    // Cancelar animaciones en curso
    if (ikAnimationId) {
      cancelAnimationFrame(ikAnimationId);
      ikAnimationId = null;
    }

    // Apagar láser de inmediato por seguridad
    state.laserActive = false;
    laserIndicator.textContent = 'APAGADO';
    laserIndicator.className = 'laser-status-badge laser-off';
    btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
    btnLaserToggle.classList.remove('btn-laser-active');
    
    disableInputs(true);
    drawRobot();
  } else {
    // Liberar parada de emergencia
    document.body.classList.remove('emergency-active');
    btnEmergency.querySelector('span').textContent = 'PARADA DE EMERGENCIA';
    
    systemStatusText.textContent = 'MODO SIMULACIÓN ACTIVO';
    systemStatusText.style.color = '';
    systemStatusDot.style.backgroundColor = '';
    systemStatusDot.style.boxShadow = '';
    
    disableInputs(false);
    drawRobot();
  }
});

// ==================== EVENTO DEL BOTÓN DE RESETEO (HOME) ====================
const btnResetHome = document.getElementById('btn-reset-home');

btnResetHome.addEventListener('click', () => {
  if (state.emergencyStop) return;
  
  // Detener animación previa si la hubiera
  if (ikAnimationId) {
    cancelAnimationFrame(ikAnimationId);
    ikAnimationId = null;
  }
  
  // Animar suavemente de vuelta a la posición inicial (0, 0, 0)
  animateToJoints(0, 0, 0);
  
  // Actualizar los inputs de destino IK con la posición inicial
  document.getElementById('input-ik-x').value = 374;
  document.getElementById('input-ik-y').value = 0;
  document.getElementById('input-ik-z').value = 630;
  
  // Restablecer indicador de alcance
  ikStatus.textContent = 'ALCANCE OK';
  ikStatus.className = 'ik-status-badge ik-status-ok';

  // Desactivar láser por seguridad al volver a iniciales
  state.laserActive = false;
  state.laserPower = 0;
  laserPowerSlider.value = 0;
  laserPowerValText.textContent = '0';
  laserIndicator.textContent = 'APAGADO';
  laserIndicator.className = 'laser-status-badge laser-off';
  btnLaserToggle.textContent = 'ACTIVAR APUNTADOR LÁSER';
  btnLaserToggle.classList.remove('btn-laser-active');
});

function disableInputs(disable) {
  // Deshabilitar todos los inputs interactivos y botones de control
  const controls = document.querySelectorAll(
    '.joint-slider, .joint-value-input input, #input-range-laser, #input-ik-x, #input-ik-y, #input-ik-z, #btn-apply-ik, #btn-reset-home'
  );
  controls.forEach(el => {
    el.disabled = disable;
  });
  
  btnLaserToggle.disabled = disable;
}

// ==================== INICIALIZACIÓN ====================
function init() {
  updateKinematics();
  drawRobot();
  syncIkTargetInputs();
  
  // Dibujar nuevamente al cargar todo para asegurar fuentes personalizadas cargadas
  window.addEventListener('load', () => {
    drawRobot();
  });
}

init();
