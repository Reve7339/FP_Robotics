#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

#define SERVO_BASE_PIN     18
#define SERVO_SHOULDER_PIN 19
#define SERVO_ELBOW_PIN    21
#define LASER_PIN          22



const float L0 = 0.096f;
const float L1 = 0.146f;
const float L2 = 0.161f;

const float MIN_THETA1 = -1.5708f;
const float MAX_THETA1 =  1.5708f;
const float MIN_THETA2 = -0.1745f;
const float MAX_THETA2 =  1.5708f;
const float MIN_THETA3 = -2.0944f;
const float MAX_THETA3 =  0.1745f;


const unsigned long SERIAL_TIMEOUT_MS = 1000;

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;


unsigned long lastPacketTime = 0;
bool isEmergency = false;


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

  t2 = atan2(z_prime, r) - atan2(L2 * sin(t3), L1 + L2 * cos(t3));

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



  if (millis() - lastPacketTime > SERIAL_TIMEOUT_MS) {
    digitalWrite(LASER_PIN, LOW);
  }

  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      // Comando manual para parada de emergencia
      if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("EMERGENCY")) {
        triggerEmergency("Comando manual recibido");
        return;
      }

      int idxJ1 = line.indexOf("J1:");
      int idxJ2 = line.indexOf("J2:");
      int idxJ3 = line.indexOf("J3:");
      int idxL  = line.indexOf(",L:");

      int idxX = line.indexOf("X:");
      int idxY = line.indexOf("Y:");
      int idxZ = line.indexOf("Z:");

      if (idxJ1 != -1 && idxJ2 != -1 && idxJ3 != -1) {
        int endJ1 = line.indexOf(',', idxJ1);
        int endJ2 = line.indexOf(',', idxJ2);
        int endJ3 = (idxL != -1) ? idxL : line.length();

        String j1Str = line.substring(idxJ1 + 3, endJ1 != -1 ? endJ1 : line.length());
        String j2Str = line.substring(idxJ2 + 3, endJ2 != -1 ? endJ2 : line.length());
        String j3Str = line.substring(idxJ3 + 3, endJ3 != -1 ? endJ3 : line.length());

        float j1_deg = j1Str.toFloat();
        float j2_deg = j2Str.toFloat();
        float j3_deg = j3Str.toFloat();

        bool laserVal = false;
        if (idxL != -1) {
          String lStr = line.substring(idxL + 3);
          laserVal = (lStr.toInt() > 0);
        }

        float theta1 = j1_deg * PI / 180.0f;
        float theta2 = j2_deg * PI / 180.0f;
        float theta3 = j3_deg * PI / 180.0f;

        int base_us = angleToMicroseconds(theta1, MIN_THETA1, MAX_THETA1);
        int shoulder_us = angleToMicroseconds(theta2, MIN_THETA2, MAX_THETA2);
        int elbow_us = angleToMicroseconds(theta3, MIN_THETA3, MAX_THETA3);

        lastPacketTime = millis();
        if (laserVal) {
          digitalWrite(LASER_PIN, HIGH);
        } else {
          digitalWrite(LASER_PIN, LOW);
        }

        servoBase.writeMicroseconds(base_us);
        servoShoulder.writeMicroseconds(shoulder_us);
        servoElbow.writeMicroseconds(elbow_us);

        Serial.print("OK | Joints set: J1=");
        Serial.print(j1_deg, 1);
        Serial.print(" J2=");
        Serial.print(j2_deg, 1);
        Serial.print(" J3=");
        Serial.print(j3_deg, 1);
        Serial.print(" | Laser=");
        Serial.println(laserVal ? "ON" : "OFF");
      } else if (idxX != -1 && idxY != -1 && idxZ != -1) {
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
