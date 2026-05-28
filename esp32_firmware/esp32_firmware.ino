#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <math.h>

// ==========================================
// CONFIGURACIÓN DE HARDWARE & PINES
// ==========================================
#define SERVO_BASE_PIN     18
#define SERVO_SHOULDER_PIN 19 // Controla el par paralelo del hombro
#define SERVO_ELBOW_PIN    21
#define LASER_PIN          22

// Pines I2C para el sensor de corriente INA219
#define I2C_SDA            23
#define I2C_SCL            25

// ==========================================
// PARÁMETROS CINEMÁTICOS (en metros)
// ==========================================
const float L0 = 0.150f; // Altura de la base (150 mm)
const float L1 = 0.200f; // Longitud del hombro al codo (200 mm)
const float L2 = 0.200f; // Longitud del codo al efector (200 mm)

// ==========================================
// LIMITES FISICOS DE LAS ARTICULACIONES (Radianes)
// ==========================================
const float MIN_THETA1 = -1.5708f; // -90 deg
const float MAX_THETA1 =  1.5708f; //  90 deg
const float MIN_THETA2 = -0.1745f; // -10 deg
const float MAX_THETA2 =  1.5708f; //  90 deg
const float MIN_THETA3 = -2.0944f; // -120 deg
const float MAX_THETA3 =  0.1745f; //  10 deg

// ==========================================
// PARÁMETROS DE SEGURIDAD
// ==========================================
const float CURRENT_LIMIT_MA = 2500.0f; // Límite de corriente (2.5 Amperios)
const unsigned long SERIAL_TIMEOUT_MS = 1000; // Apagado del láser si no hay datos en 1s

// Instancias de librerías
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Adafruit_INA219 ina219;

// Variables de estado
unsigned long lastPacketTime = 0;
bool isEmergency = false;
bool hasIna219 = false;

// ==========================================
// FUNCIONES DE MAPEADO Y CINEMÁTICA
// ==========================================

// Convierte un ángulo en radianes a microsegundos para el control del servo.
// 0 radianes representa la posición central del servo (1500 us).
// Rango de -PI/2 a PI/2 radianes se mapea a 500 us - 2500 us.
int angleToMicroseconds(float angle_rad, float min_rad, float max_rad) {
  // Limitar el ángulo según las restricciones físicas de la articulación
  float clamped = constrain(angle_rad, min_rad, max_rad);
  
  // Mapear linealmente de radianes a microsegundos
  // 1500 us es el centro (0 rad). La ganancia es (1000 us) / (PI / 2 rad) = 2000.0 / PI
  float us = 1500.0f + clamped * (2000.0f / PI);
  
  // Limitar a los márgenes seguros estándar para servos (500 - 2500 us)
  return constrain((int)us, 500, 2500);
}

// Resuelve la cinemática inversa del brazo de 3 GDL (Codo arriba)
bool solveInverseKinematics(float x, float y, float z, float &t1, float &t2, float &t3) {
  // 1. Ángulo de rotación de la base
  t1 = atan2(y, x);

  // Proyección horizontal y altura corregida respecto al hombro
  float r = sqrt(x * x + y * y);
  float z_prime = z - L0;

  // 2. Ángulo del codo (Ley de cosenos)
  float psi_numerator = x * x + y * y + z_prime * z_prime - L1 * L1 - L2 * L2;
  float psi_denominator = 2.0f * L1 * L2;
  float psi = psi_numerator / psi_denominator;

  // Comprobar si la posición está dentro del volumen de trabajo (rango admisible del coseno)
  if (psi < -1.0f || psi > 1.0f) {
    return false; // Fuera del alcance del brazo
  }

  // Solución codo arriba (Elbow Up) -> Seno con signo negativo
  t3 = atan2(-sqrt(1.0f - psi * psi), psi);

  // 3. Ángulo del hombro
  float k1 = L1 + L2 * cos(t3);
  float k2 = L2 * sin(t3);
  t2 = atan2(k1 * z_prime - k2 * r, k1 * r + k2 * z_prime);

  return true;
}

// Detiene de inmediato el láser y libera los servos
void triggerEmergency(const char* reason) {
  isEmergency = true;
  digitalWrite(LASER_PIN, LOW); // Apagar láser inmediatamente
  
  // Desacoplar servos para remover tensión de alimentación
  servoBase.detach();
  servoShoulder.detach();
  servoElbow.detach();
  
  Serial.print("\n=== PARADA DE EMERGENCIA ACTIVA ===\nMotivo: ");
  Serial.println(reason);
  Serial.println("Reinicie el ESP32 para reestablecer el sistema.");
}

// ==========================================
// INICIALIZACIÓN
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 3-GDL Laser Arm Controller (C++) ===");

  // Configurar pin de control del láser
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW); // Láser apagado por defecto

  // Inicializar comunicación I2C para el sensor de corriente INA219
  Wire.begin(I2C_SDA, I2C_SCL);
  if (ina219.begin()) {
    hasIna219 = true;
    Serial.println("INA219: Sensor de corriente inicializado correctamente.");
  } else {
    Serial.println("WARNING: INA219 no detectado. Monitoreo de corriente desactivado.");
  }

  // Configurar temporizadores y canales de los servos usando ESP32Servo
  // Los servos se configuran en el periodo estándar de 50Hz (20ms)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  
  servoBase.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);

  // Acoplar los servos especificando los pines y los anchos de pulso en us (500us a 2500us)
  servoBase.attach(SERVO_BASE_PIN, 500, 2500);
  servoShoulder.attach(SERVO_SHOULDER_PIN, 500, 2500);
  servoElbow.attach(SERVO_ELBOW_PIN, 500, 2500);

  // Mover a la posición neutra inicial (0 rad = 1500 us)
  servoBase.writeMicroseconds(1500);
  servoShoulder.writeMicroseconds(1500);
  servoElbow.writeMicroseconds(1500);

  lastPacketTime = millis();
  Serial.println("Sistema listo. Esperando comandos seriales (ej: X:120.5,Y:45.0,Z:80.2)...");
}

// ==========================================
// BUCLE PRINCIPAL
// ==========================================
void loop() {
  // 1. Si estamos en estado de emergencia, detener todo el bucle
  if (isEmergency) {
    digitalWrite(LASER_PIN, LOW);
    delay(100);
    return;
  }

  // 2. Validar consumo de corriente mediante el sensor INA219
  if (hasIna219) {
    float current_mA = ina219.getCurrent_mA();
    if (abs(current_mA) > CURRENT_LIMIT_MA) {
      char errorMsg[64];
      sprintf(errorMsg, "Sobrecorriente detectada: %.1f mA (Max: %.1f mA)", abs(current_mA), CURRENT_LIMIT_MA);
      triggerEmergency(errorMsg);
      return;
    }
  }

  // 3. Watchdog de seguridad para comunicación serial (Apagado del láser si hay pérdidas)
  if (millis() - lastPacketTime > SERIAL_TIMEOUT_MS) {
    digitalWrite(LASER_PIN, LOW); // Apagar láser por falta de datos
  }

  // 4. Recepción y procesamiento de comandos por puerto serial
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      // Buscar las etiquetas de coordenadas X, Y, Z
      int idxX = line.indexOf("X:");
      int idxY = line.indexOf("Y:");
      int idxZ = line.indexOf("Z:");

      if (idxX != -1 && idxY != -1 && idxZ != -1) {
        // Encontrar comas de separación
        int endX = line.indexOf(',', idxX);
        int endY = line.indexOf(',', idxY);
        int endZ = line.length();

        // Extraer los sub-strings de las coordenadas
        String xStr = line.substring(idxX + 2, endX != -1 ? endX : line.length());
        String yStr = line.substring(idxY + 2, endY != -1 ? endY : line.length());
        String zStr = line.substring(idxZ + 2, endZ);

        float raw_x = xStr.toFloat();
        float raw_y = yStr.toFloat();
        float raw_z = zStr.toFloat();

        // Autodetectar unidades: Si los valores son mayores a 2.0 (o menores a -2.0),
        // asumimos que el host está enviando las coordenadas en milímetros y escalamos a metros.
        float scale = 1.0f;
        if (abs(raw_x) > 2.0f || abs(raw_y) > 2.0f || abs(raw_z) > 2.0f) {
          scale = 0.001f;
        }

        float x = raw_x * scale;
        float y = raw_y * scale;
        float z = raw_z * scale;

        // Resolver cinemática inversa
        float theta1 = 0.0f, theta2 = 0.0f, theta3 = 0.0f;
        if (solveInverseKinematics(x, y, z, theta1, theta2, theta3)) {
          // Si es válido, actualizar tiempo del Watchdog y encender láser
          lastPacketTime = millis();
          digitalWrite(LASER_PIN, HIGH);

          // Convertir ángulos articulares a señales PWM en microsegundos
          int base_us = angleToMicroseconds(theta1, MIN_THETA1, MAX_THETA1);
          int shoulder_us = angleToMicroseconds(theta2, MIN_THETA2, MAX_THETA2);
          int elbow_us = angleToMicroseconds(theta3, MIN_THETA3, MAX_THETA3);

          // Comandar servomotores
          servoBase.writeMicroseconds(base_us);
          servoShoulder.writeMicroseconds(shoulder_us);
          servoElbow.writeMicroseconds(elbow_us);

          // Feedback de depuración
          Serial.print("OK | Target: X=");
          Serial.print(x, 4);
          Serial.print(" Y=");
          Serial.print(y, 4);
          Serial.print(" Z=");
          Serial.print(z, 4);
          Serial.print(" | Joints: T1=");
          Serial.print(theta1 * 180.0f / PI, 1);
          Serial.print(" T2=");
          Serial.print(theta2 * 180.0f / PI, 1);
          Serial.print(" T3=");
          Serial.println(theta3 * 180.0f / PI, 1);
        } else {
          Serial.println("ERR: Posicion cartesiana fuera del area de alcance.");
        }
      } else {
        Serial.println("WARN: Formato invalido. Use 'X:val,Y:val,Z:val'");
      }
    }
  }
}
