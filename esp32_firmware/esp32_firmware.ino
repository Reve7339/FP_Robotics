#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <math.h>

#define SERVO_BASE_PIN     18
#define SERVO_SHOULDER_PIN 19
#define SERVO_ELBOW_PIN    21
#define LASER_PIN          22

#define I2C_SDA            23
#define I2C_SCL            25

const float L0 = 0.150f;
const float L1 = 0.200f;
const float L2 = 0.200f;

const float MIN_THETA1 = -1.5708f;
const float MAX_THETA1 =  1.5708f;
const float MIN_THETA2 = -0.1745f;
const float MAX_THETA2 =  1.5708f;
const float MIN_THETA3 = -2.0944f;
const float MAX_THETA3 =  0.1745f;

const float CURRENT_LIMIT_MA = 2500.0f;
const unsigned long SERIAL_TIMEOUT_MS = 1000;

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Adafruit_INA219 ina219;

unsigned long lastPacketTime = 0;
bool isEmergency = false;
bool hasIna219 = false;

/*
 * Convierte un ángulo articular en radianes al correspondiente ancho de pulso PWM en microsegundos,
 * limitando el resultado entre los márgenes físicos y de seguridad del servomotor.
 */
int angleToMicroseconds(float angle_rad, float min_rad, float max_rad) {
  float clamped = constrain(angle_rad, min_rad, max_rad);
  float us = 1500.0f + clamped * (2000.0f / PI);
  return constrain((int)us, 500, 2500);
}

/*
 * Resuelve analíticamente el modelo cinemático inverso para un brazo de 3 GDL bajo
 * la configuración geométrica de codo arriba a partir de una posición cartesiana (x, y, z).
 */
bool solveInverseKinematics(float x, float y, float z, float &t1, float &t2, float &t3) {
  t1 = atan2(y, x);

  float r = sqrt(x * x + y * y);
  float z_prime = z - L0;

  float psi_numerator = x * x + y * y + z_prime * z_prime - L1 * L1 - L2 * L2;
  float psi_denominator = 2.0f * L1 * L2;
  float psi = psi_numerator / psi_denominator;

  if (psi < -1.0f || psi > 1.0f) {
    return false;
  }

  t3 = atan2(-sqrt(1.0f - psi * psi), psi);

  float k1 = L1 + L2 * cos(t3);
  float k2 = L2 * sin(t3);
  t2 = atan2(k1 * z_prime - k2 * r, k1 * r + k2 * z_prime);

  return true;
}

/*
 * Activa de forma inmediata el estado de emergencia, apaga el emisor láser de corte
 * y desacopla la señal de los servomotores para liberar la tensión de la estructura.
 */
void triggerEmergency(const char* reason) {
  isEmergency = true;
  digitalWrite(LASER_PIN, LOW);
  
  servoBase.detach();
  servoShoulder.detach();
  servoElbow.detach();
  
  Serial.print("\n=== PARADA DE EMERGENCIA ACTIVA ===\nMotivo: ");
  Serial.println(reason);
  Serial.println("Reinicie el ESP32 para reestablecer el sistema.");
}

/*
 * Configura los periféricos de hardware, inicializa la interfaz serial, establece
 * la comunicación I2C para el sensor de corriente y asocia los pines PWM de los servos.
 */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 3-GDL Laser Arm Controller (C++) ===");

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (ina219.begin()) {
    hasIna219 = true;
    Serial.println("INA219: Sensor de corriente inicializado correctamente.");
  } else {
    Serial.println("WARNING: INA219 no detectado. Monitoreo de corriente desactivado.");
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  
  servoBase.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);

  servoBase.attach(SERVO_BASE_PIN, 500, 2500);
  servoShoulder.attach(SERVO_SHOULDER_PIN, 500, 2500);
  servoElbow.attach(SERVO_ELBOW_PIN, 500, 2500);

  servoBase.writeMicroseconds(1500);
  servoShoulder.writeMicroseconds(1500);
  servoElbow.writeMicroseconds(1500);

  lastPacketTime = millis();
  Serial.println("Sistema listo. Esperando comandos seriales (ej: X:120.5,Y:45.0,Z:80.2)...");
}

/*
 * Ejecuta el bucle principal de control, monitorea sobrecorrientes en tiempo real,
 * procesa comandos seriales cartesianos, actualiza la cinemática e implementa el Watchdog de seguridad.
 */
void loop() {
  if (isEmergency) {
    digitalWrite(LASER_PIN, LOW);
    delay(100);
    return;
  }

  if (hasIna219) {
    float current_mA = ina219.getCurrent_mA();
    if (abs(current_mA) > CURRENT_LIMIT_MA) {
      char errorMsg[64];
      sprintf(errorMsg, "Sobrecorriente detectada: %.1f mA (Max: %.1f mA)", abs(current_mA), CURRENT_LIMIT_MA);
      triggerEmergency(errorMsg);
      return;
    }
  }

  if (millis() - lastPacketTime > SERIAL_TIMEOUT_MS) {
    digitalWrite(LASER_PIN, LOW);
  }

  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      int idxX = line.indexOf("X:");
      int idxY = line.indexOf("Y:");
      int idxZ = line.indexOf("Z:");

      if (idxX != -1 && idxY != -1 && idxZ != -1) {
        int endX = line.indexOf(',', idxX);
        int endY = line.indexOf(',', idxY);
        int endZ = line.length();

        String xStr = line.substring(idxX + 2, endX != -1 ? endX : line.length());
        String yStr = line.substring(idxY + 2, endY != -1 ? endY : line.length());
        String zStr = line.substring(idxZ + 2, endZ);

        float raw_x = xStr.toFloat();
        float raw_y = yStr.toFloat();
        float raw_z = zStr.toFloat();

        float scale = 1.0f;
        if (abs(raw_x) > 2.0f || abs(raw_y) > 2.0f || abs(raw_z) > 2.0f) {
          scale = 0.001f;
        }

        float x = raw_x * scale;
        float y = raw_y * scale;
        float z = raw_z * scale;

        float theta1 = 0.0f, theta2 = 0.0f, theta3 = 0.0f;
        if (solveInverseKinematics(x, y, z, theta1, theta2, theta3)) {
          lastPacketTime = millis();
          digitalWrite(LASER_PIN, HIGH);

          int base_us = angleToMicroseconds(theta1, MIN_THETA1, MAX_THETA1);
          int shoulder_us = angleToMicroseconds(theta2, MIN_THETA2, MAX_THETA2);
          int elbow_us = angleToMicroseconds(theta3, MIN_THETA3, MAX_THETA3);

          servoBase.writeMicroseconds(base_us);
          servoShoulder.writeMicroseconds(shoulder_us);
          servoElbow.writeMicroseconds(elbow_us);

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
