#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>

// ============================================================
// AGV MARCADO DE LAYOUTS - ESP32
//
// v2:
//  - Telemetria de diagnostico del MPU6050 completa
//  - Bomba peristaltica (marcado) con pre-flujo y corte anticipado
//  - Control de rumbo en recta con giroscopio (PD sobre yaw)
//  - Yaw continuo (yawDeg = yaw IMU + offset): la deriva en recta
//    ya queda reflejada en la pose
//  - Odometria incremental (la pose real, no la teorica)
//  - Giro con yaw "desenrollado": ya no falla en giros de 180 grados
//  - Auto-correccion del bias del giroscopio cuando el AGV esta quieto
//  - Comandos SET / GET para afinar parametros sin recompilar
//  - SETPOSE listo para corregir pose con AprilTags mas adelante
// ============================================================

// ============================================================
// WIFI / TCP / OTA
// ============================================================
const char* ssid = "WIRELESS";
const char* password = "FAMILIA8630";

const uint16_t TCP_PORT = 8888;
WiFiServer server(TCP_PORT);
WiFiClient cliente;

const char* OTA_HOSTNAME = "AGV-ESP32";
// const char* OTA_PASSWORD = "agv123";

// ============================================================
// L298N
// ============================================================
const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;
const int IN3 = 12;   // OJO: GPIO12 es pin de strapping del ESP32
const int IN4 = 13;
const int ENB = 14;

// ============================================================
// BOMBA PERISTALTICA (a traves de MOSFET / driver propio)
// NO conectar la bomba directo al pin. Usar MOSFET logico (IRLZ44N,
// modulo MOSFET) con diodo flyback y GND comun con el ESP32.
// ============================================================
const int  PUMP_PIN = 32;
const bool PUMP_ACTIVE_HIGH = true;   // false si el driver es activo en bajo

// ============================================================
// I2C / TCA9548A / AS5600 / MPU6050
// ============================================================
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const uint8_t TCA_ADDR = 0x70;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_H = 0x0C;

const uint8_t CH_ENC_LEFT  = 1;
const uint8_t CH_ENC_RIGHT = 2;

const uint8_t CH_MPU6050 = 0;
const uint8_t MPU6050_ADDR = 0x68;
const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_CONFIG = 0x1A;
const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_GYRO_ZOUT_H = 0x47;

// Escala +/-250 deg/s = 131 LSB/(deg/s)
const float MPU_GYRO_SENS = 131.0f;

// Si al girar con TURN:+ el YAW_IMU baja, cambiar a -1.
int IMU_YAW_SIGN = +1;

const float COUNTS_PER_REV = 4096.0f;

// ============================================================
// PARAMETROS MECANICOS / CONTROL
// ============================================================
float DIAMETRO_RUEDA_MM = 64.5f;
float DISTANCIA_ENTRE_RUEDAS_MM = 165.0f;
float RELACION_TRANSMISION = 1.0f;

int ENC_LEFT_SIGN = +1;
int ENC_RIGHT_SIGN = +1;

int MOTOR_LEFT_SIGN = +1;
int MOTOR_RIGHT_SIGN = +1;

int PWM_AVANCE = 165;
int PWM_GIRO   = 145;
int PWM_MIN    = 90;
int PWM_MIN_GIRO = 80;
int PWM_MAX    = 220;

// Correccion por diferencia de encoders (0 = desactivada)
float KP_ENCODERS = 0.0f;

float DIST_DESACEL_MM = 80.0f;
float TOL_DIST_MM = 4.0f;
float TOL_GIRO_DEG = 2.0f;

unsigned long TELEMETRIA_MS = 120;
unsigned long TIMEOUT_MOV_MS = 30000;

// ============================================================
// CONTROL DE RUMBO EN RECTA (HEADING HOLD)
//
// correccion = KP_HDG * errorRumbo - KD_HDG * velocidadAngular
//   errorRumbo = rumboObjetivo - yaw      [grados]
//   velocidadAngular = gyro Z             [grados/s]
// Se suma a la rueda izquierda y se resta a la derecha, igual que
// en TURN:+ (que es el sentido en que sube el yaw).
// ============================================================
bool  HEADING_HOLD = true;
float KP_HDG = 2.5f;       // PWM por grado de error
float KD_HDG = 0.35f;      // PWM por grado/s (amortiguacion)
int   MAX_CORR_PWM = 35;   // limite de la correccion
int   PWM_MIN_CORR = 70;   // PWM minimo por rueda al corregir

// ============================================================
// BOMBA - PARAMETROS DE MARCADO
// ============================================================
int   PUMP_PWM_NOMINAL = 200;         // caudal "de crucero" (0..255)
int   PUMP_PWM_MIN = 90;              // PWM minimo al que la bomba realmente gira
unsigned long PUMP_PREFLOW_MS = 400;  // bomba encendida con el AGV quieto antes de arrancar
float PUMP_END_MM = 0.0f;             // apagar la bomba N mm antes del final del tramo
bool  PUMP_SCALE_WITH_SPEED = true;   // bajar caudal cuando el AGV desacelera
unsigned long PUMP_MANUAL_MAX_MS = 15000;  // maximo de bomba manual continua

// ============================================================
// AUTO-BIAS DEL GIROSCOPIO (solo con el AGV quieto)
// ============================================================
bool  IMU_AUTOBIAS = true;
float IMU_AUTOBIAS_ALPHA = 0.001f;
float IMU_AUTOBIAS_MAX_DPS = 0.6f;
unsigned long IMU_AUTOBIAS_ESPERA_MS = 500;
float GYRO_DEADBAND_DPS = 0.15f;

// Si true, al terminar un GOTO la pose se fuerza al punto objetivo
// (comportamiento viejo). Si false, se conserva la pose odometrica real
// y el siguiente GOTO corrige el error.
const bool SNAP_GOTO_AL_OBJETIVO = false;

// ============================================================
// ESTADOS
// ============================================================
enum EstadoMovimiento {
  IDLE,
  MOVING_STRAIGHT,
  TURNING
};

EstadoMovimiento estadoMovimiento = IDLE;

bool emergencyStop = false;
bool otaEnProgreso = false;

bool enc1Enabled = true;
bool enc2Enabled = true;
bool motor1Enabled = true;
bool motor2Enabled = true;

// Pose estimada
float posX = 0.0f;
float posY = 0.0f;
float yawDeg = 0.0f;          // = yaw IMU + offset (continuo)
float yawOffsetDeg = 0.0f;

// ============================================================
// MPU6050
// ============================================================
bool imuOK = false;
float gyroZBiasDps = 0.0f;
float gyroZRawDps = 0.0f;     // lectura cruda (sin bias, sin signo)
float gyroZDps = 0.0f;        // velocidad angular util (con bias, signo, zona muerta)
float dtImuMs = 0.0f;         // ultimo dt valido de integracion
float yawImuDeg = 0.0f;       // yaw IMU normalizado (-180,180]
double yawImuUnwrapDeg = 0.0; // yaw IMU acumulado sin envolver
double yawImuInicioGiro = 0.0;
unsigned long lastImuMicros = 0;
unsigned long idleDesdeMs = 0;

// ============================================================
// ENCODERS
// ============================================================
uint16_t rawLeftPrev = 0;
uint16_t rawRightPrev = 0;
bool leftInitialized = false;
bool rightInitialized = false;
long countLeft = 0;
long countRight = 0;
long startCountLeft = 0;
long startCountRight = 0;

// ============================================================
// OBJETIVOS / MOVIMIENTO
// ============================================================
float targetDistanceMm = 0.0f;
float targetTurnDeg = 0.0f;
float targetTurnWheelMm = 0.0f;
int turnSign = +1;

float recorridoPrevMm = 0.0f;
float headingHoldDeg = 0.0f;

// GOTO
bool gotoPending = false;
float gotoTargetX = 0.0f;
float gotoTargetY = 0.0f;
float gotoDistanceMm = 0.0f;
float gotoHeadingDeg = 0.0f;
bool gotoMarcar = false;

// Marcado
bool pumpEnabled = false;        // habilitacion global (OFF = recorrido "en seco")
bool marcarActual = false;       // el tramo en curso marca
bool preflowActivo = false;
unsigned long preflowInicioMs = 0;
int pumpTramoPwm = 0;            // PWM pedido por el tramo en curso
int pumpPwmActual = 0;           // PWM realmente aplicado al pin
bool pumpManual = false;
unsigned long pumpManualInicioMs = 0;

// Timers
unsigned long movementStartMs = 0;
unsigned long lastTelemetryMs = 0;

String tcpBuffer = "";

// ============================================================
// UTILIDADES
// ============================================================
float wheelCircumferenceMm() {
  return PI * DIAMETRO_RUEDA_MM;
}

float countsToMm(long counts) {
  return ((float)counts / COUNTS_PER_REV) *
         wheelCircumferenceMm() /
         RELACION_TRANSMISION;
}

float normalizeAngleDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

float errorRumboDeg() {
  return normalizeAngleDeg(headingHoldDeg - yawDeg);
}

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

int dividir(const String& s, char sep, String out[], int maxN) {
  int n = 0;
  int start = 0;

  while (n < maxN) {
    int idx = s.indexOf(sep, start);

    if (idx < 0) {
      out[n++] = s.substring(start);
      break;
    }

    out[n++] = s.substring(start, idx);
    start = idx + 1;
  }

  return n;
}

// ============================================================
// SALIDA A SERIAL + TCP
// ============================================================
void enviarLinea(const String& msg) {
  Serial.println(msg);

  if (cliente && cliente.connected()) {
    cliente.println(msg);
  }
}

// ============================================================
// MPU6050
// ============================================================
bool mpuWriteByte(uint8_t reg, uint8_t value) {
  tcaSelect(CH_MPU6050);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool mpuRead16(uint8_t reg, int16_t& value) {
  tcaSelect(CH_MPU6050);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)MPU6050_ADDR, 2) != 2) {
    return false;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  value = (int16_t)((highByte << 8) | lowByte);
  return true;
}

bool iniciarMPU6050() {
  if (!mpuWriteByte(MPU6050_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  // DLPF moderado
  if (!mpuWriteByte(MPU6050_CONFIG, 0x03)) return false;

  // Giroscopio +/-250 deg/s
  if (!mpuWriteByte(MPU6050_GYRO_CONFIG, 0x00)) return false;

  delay(50);
  return true;
}

// Calcula el bias del gyro Z. NO mover el AGV.
// resetYaw = true  -> ademas pone el yaw IMU en 0 (uso en el arranque)
// resetYaw = false -> solo recalcula el bias (comando CALIB)
void calibrarGyroZ(bool resetYaw) {
  const int muestras = 1200;
  double suma = 0.0;
  int validas = 0;

  Serial.println("MPU6050: calibrando gyro Z. NO MOVER EL AGV...");

  for (int i = 0; i < muestras; i++) {
    int16_t rawZ = 0;

    if (mpuRead16(MPU6050_GYRO_ZOUT_H, rawZ)) {
      suma += ((float)rawZ / MPU_GYRO_SENS);
      validas++;
    }

    delay(2);
  }

  if (validas > 0) {
    gyroZBiasDps = (float)(suma / validas);
    imuOK = true;
  } else {
    gyroZBiasDps = 0.0f;
    imuOK = false;
  }

  if (resetYaw) {
    yawImuDeg = 0.0f;
    yawImuUnwrapDeg = 0.0;
    yawImuInicioGiro = 0.0;
    yawOffsetDeg = 0.0f;
    yawDeg = 0.0f;
  }

  lastImuMicros = micros();
  idleDesdeMs = millis();

  Serial.print("MPU6050: bias gyro Z = ");
  Serial.print(gyroZBiasDps, 5);
  Serial.println(" deg/s");
}

void actualizarIMU() {
  if (!imuOK) return;

  int16_t rawZ = 0;

  if (!mpuRead16(MPU6050_GYRO_ZOUT_H, rawZ)) {
    return;
  }

  unsigned long ahora = micros();

  if (lastImuMicros == 0) {
    lastImuMicros = ahora;
    return;
  }

  float dt = (ahora - lastImuMicros) * 1.0e-6f;
  lastImuMicros = ahora;

  // Rechazar intervalos anormales para evitar saltos de integracion.
  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  dtImuMs = dt * 1000.0f;

  float rawDps = (float)rawZ / MPU_GYRO_SENS;
  gyroZRawDps = rawDps;

  // Auto-bias: con el AGV quieto, el gyro solo mide su propio offset.
  if (estadoMovimiento == IDLE) {
    if (
      IMU_AUTOBIAS &&
      (millis() - idleDesdeMs) > IMU_AUTOBIAS_ESPERA_MS &&
      fabsf(rawDps - gyroZBiasDps) < IMU_AUTOBIAS_MAX_DPS
    ) {
      gyroZBiasDps += IMU_AUTOBIAS_ALPHA * (rawDps - gyroZBiasDps);
    }
  } else {
    idleDesdeMs = millis();
  }

  gyroZDps = IMU_YAW_SIGN * (rawDps - gyroZBiasDps);

  if (fabsf(gyroZDps) < GYRO_DEADBAND_DPS) {
    gyroZDps = 0.0f;
  }

  yawImuUnwrapDeg += (double)gyroZDps * dt;
  yawImuDeg = normalizeAngleDeg((float)yawImuUnwrapDeg);

  // Yaw de la pose: continuo, sin saltos al terminar los giros.
  yawDeg = normalizeAngleDeg(yawImuDeg + yawOffsetDeg);
}

// ============================================================
// ENCODERS
// ============================================================
uint16_t readAS5600Raw(uint8_t channel) {
  tcaSelect(channel);

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);

  if (Wire.endTransmission(false) != 0) {
    return 0xFFFF;
  }

  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) {
    return 0xFFFF;
  }

  uint16_t highByte = Wire.read();
  uint16_t lowByte = Wire.read();

  return ((highByte & 0x0F) << 8) | lowByte;
}

long unwrapDelta(uint16_t current, uint16_t previous) {
  int delta = (int)current - (int)previous;

  if (delta > 2048) delta -= 4096;
  if (delta < -2048) delta += 4096;

  return delta;
}

void leerEncoders() {
  uint16_t rawL = readAS5600Raw(CH_ENC_LEFT);

  if (rawL != 0xFFFF) {
    if (!leftInitialized) {
      rawLeftPrev = rawL;
      leftInitialized = true;
    } else {
      long delta = unwrapDelta(rawL, rawLeftPrev);
      countLeft += ENC_LEFT_SIGN * delta;
      rawLeftPrev = rawL;
    }
  }

  uint16_t rawR = readAS5600Raw(CH_ENC_RIGHT);

  if (rawR != 0xFFFF) {
    if (!rightInitialized) {
      rawRightPrev = rawR;
      rightInitialized = true;
    } else {
      long delta = unwrapDelta(rawR, rawRightPrev);
      countRight += ENC_RIGHT_SIGN * delta;
      rawRightPrev = rawR;
    }
  }
}

void resetEncoders() {
  countLeft = 0;
  countRight = 0;

  leftInitialized = false;
  rightInitialized = false;

  delay(5);
  leerEncoders();

  startCountLeft = countLeft;
  startCountRight = countRight;
}

float distLeftFromMoveStart() {
  return countsToMm(countLeft - startCountLeft);
}

float distRightFromMoveStart() {
  return countsToMm(countRight - startCountRight);
}

float distanciaDisponibleAbs() {
  float dl = fabsf(distLeftFromMoveStart());
  float dr = fabsf(distRightFromMoveStart());

  if (enc1Enabled && enc2Enabled) return (dl + dr) * 0.5f;
  if (enc1Enabled) return dl;
  if (enc2Enabled) return dr;

  return 0.0f;
}

// ============================================================
// MOTORES
// ============================================================
void setMotor(
  int inA,
  int inB,
  int enPin,
  int pwmSigned,
  bool enabled,
  int motorSign
) {
  if (!enabled || emergencyStop || otaEnProgreso || pwmSigned == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(enPin, 0);
    return;
  }

  int command = pwmSigned * motorSign;
  int pwm = constrain(abs(command), 0, PWM_MAX);

  if (command > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }

  analogWrite(enPin, pwm);
}

void controlMotores(int pwmLeft, int pwmRight) {
  setMotor(IN1, IN2, ENA, pwmLeft, motor1Enabled, MOTOR_LEFT_SIGN);
  setMotor(IN3, IN4, ENB, pwmRight, motor2Enabled, MOTOR_RIGHT_SIGN);
}

void detenerMotores() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ============================================================
// BOMBA
// ============================================================
void escribirBomba(int pwm) {
  pwm = constrain(pwm, 0, 255);
  pumpPwmActual = pwm;

  int salida = PUMP_ACTIVE_HIGH ? pwm : (255 - pwm);
  analogWrite(PUMP_PIN, salida);
}

// Se llama en cada vuelta del loop. Centraliza TODA la logica de
// encendido: si algo falla (STOP, OTA, estado distinto de recta),
// la bomba se apaga sola.
void actualizarBomba() {
  int pwm = 0;

  if (!emergencyStop && !otaEnProgreso && pumpEnabled) {
    if (estadoMovimiento == MOVING_STRAIGHT) {
      pwm = pumpTramoPwm;
    } else if (pumpManual) {
      if (millis() - pumpManualInicioMs > PUMP_MANUAL_MAX_MS) {
        pumpManual = false;
        enviarLinea("INFO:PUMP_MANUAL_TIMEOUT");
      } else {
        pwm = PUMP_PWM_NOMINAL;
      }
    }
  }

  escribirBomba(pwm);
}

void apagarBombaYa() {
  pumpManual = false;
  pumpTramoPwm = 0;
  marcarActual = false;
  preflowActivo = false;
  escribirBomba(0);
}

// ============================================================
// MOVIMIENTO RECTO
// ============================================================
void comenzarAvance(float distanciaMm, bool marcar, float rumboDeg) {
  if (!enc1Enabled && !enc2Enabled) {
    gotoPending = false;
    enviarLinea("ERROR:NO_ENCODERS");
    return;
  }

  if (!motor1Enabled && !motor2Enabled) {
    gotoPending = false;
    enviarLinea("ERROR:NO_MOTORS");
    return;
  }

  emergencyStop = false;

  targetDistanceMm = fabsf(distanciaMm);

  startCountLeft = countLeft;
  startCountRight = countRight;
  recorridoPrevMm = 0.0f;

  headingHoldDeg = rumboDeg;

  marcarActual = marcar && pumpEnabled;
  preflowActivo = marcarActual && (PUMP_PREFLOW_MS > 0);
  preflowInicioMs = millis();
  pumpTramoPwm = 0;

  movementStartMs = millis();
  estadoMovimiento = MOVING_STRAIGHT;

  String msg = "MOVING:" + String(targetDistanceMm, 2);

  if (marcarActual) {
    msg += ",MARK";
  } else if (marcar) {
    msg += ",DRY";   // se pidio marcar pero la bomba esta deshabilitada
  }

  enviarLinea(msg);
}

void terminarTramoRecto() {
  detenerMotores();
  pumpTramoPwm = 0;
  marcarActual = false;
  preflowActivo = false;
  estadoMovimiento = IDLE;
}

void actualizarAvance() {
  // --- Odometria incremental con el yaw actual ---
  float recorrido = distanciaDisponibleAbs();
  float deltaMm = recorrido - recorridoPrevMm;
  recorridoPrevMm = recorrido;

  float yawRad = radians(yawDeg);
  posX += deltaMm * cos(yawRad);
  posY += deltaMm * sin(yawRad);

  float restante = targetDistanceMm - recorrido;

  // --- Fin del tramo ---
  if (restante <= TOL_DIST_MM) {
    terminarTramoRecto();

    if (gotoPending) {
      if (SNAP_GOTO_AL_OBJETIVO) {
        posX = gotoTargetX;
        posY = gotoTargetY;
      }
      gotoPending = false;
    }

    enviarLinea("POINT_REACHED");
    return;
  }

  if (millis() - movementStartMs > TIMEOUT_MOV_MS) {
    terminarTramoRecto();
    gotoPending = false;

    enviarLinea("ERROR:MOVE_TIMEOUT");
    return;
  }

  // --- Pre-flujo: bomba encendida, AGV quieto ---
  if (preflowActivo) {
    pumpTramoPwm = PUMP_PWM_NOMINAL;
    detenerMotores();

    if (millis() - preflowInicioMs >= PUMP_PREFLOW_MS) {
      preflowActivo = false;
      movementStartMs = millis();
    }

    return;
  }

  // --- Perfil de velocidad ---
  int basePwm = PWM_AVANCE;

  if (restante < DIST_DESACEL_MM) {
    float factor = constrain(restante / DIST_DESACEL_MM, 0.0f, 1.0f);

    basePwm = PWM_MIN + (int)((PWM_AVANCE - PWM_MIN) * factor);
  }

  // --- Correccion de rumbo con giroscopio ---
  float corrHdg = 0.0f;

  if (HEADING_HOLD && imuOK) {
    float err = errorRumboDeg();

    corrHdg = KP_HDG * err - KD_HDG * gyroZDps;
    corrHdg = constrain(corrHdg, (float)-MAX_CORR_PWM, (float)MAX_CORR_PWM);
  }

  // --- Correccion opcional por encoders ---
  float corrEnc = 0.0f;

  if (enc1Enabled && enc2Enabled) {
    float errorEnc = fabsf(distLeftFromMoveStart()) - fabsf(distRightFromMoveStart());
    corrEnc = KP_ENCODERS * errorEnc;
  }

  int pwmLeft = constrain(
    (int)(basePwm - corrEnc + corrHdg),
    PWM_MIN_CORR,
    PWM_MAX
  );

  int pwmRight = constrain(
    (int)(basePwm + corrEnc - corrHdg),
    PWM_MIN_CORR,
    PWM_MAX
  );

  controlMotores(pwmLeft, pwmRight);

  // --- Bomba: caudal proporcional a la velocidad, corte anticipado ---
  int pwmBomba = 0;

  if (marcarActual && restante > PUMP_END_MM) {
    pwmBomba = PUMP_PWM_NOMINAL;

    if (PUMP_SCALE_WITH_SPEED && PUMP_PWM_NOMINAL > PUMP_PWM_MIN) {
      float ratio = (float)basePwm / (float)PWM_AVANCE;

      pwmBomba = PUMP_PWM_MIN +
        (int)((PUMP_PWM_NOMINAL - PUMP_PWM_MIN) * constrain(ratio, 0.0f, 1.0f));
    }
  }

  pumpTramoPwm = pwmBomba;
}

// ============================================================
// GIRO (referencia: MPU6050, yaw acumulado sin envolver)
// ============================================================
void comenzarGiro(float grados, bool esParteDeGoto) {
  if (!enc1Enabled || !enc2Enabled) {
    gotoPending = false;
    enviarLinea("ERROR:TURN_REQUIRES_2_ENCODERS");
    return;
  }

  if (!motor1Enabled || !motor2Enabled) {
    gotoPending = false;
    enviarLinea("ERROR:TURN_REQUIRES_2_MOTORS");
    return;
  }

  if (!imuOK) {
    gotoPending = false;
    enviarLinea("ERROR:MPU6050_NOT_READY");
    return;
  }

  emergencyStop = false;

  targetTurnDeg = grados;
  turnSign = (grados >= 0.0f) ? +1 : -1;

  targetTurnWheelMm =
    PI * DISTANCIA_ENTRE_RUEDAS_MM * fabsf(grados) / 360.0f;

  startCountLeft = countLeft;
  startCountRight = countRight;

  yawImuInicioGiro = yawImuUnwrapDeg;

  // Durante el giro la bomba siempre esta apagada
  pumpTramoPwm = 0;
  marcarActual = false;
  preflowActivo = false;

  gotoPending = esParteDeGoto;

  movementStartMs = millis();
  estadoMovimiento = TURNING;

  enviarLinea("TURNING:" + String(grados, 2));
}

void actualizarGiro() {
  // Progreso medido con el yaw acumulado: no se rompe en giros de 180 grados.
  float deltaYawImu = (float)(yawImuUnwrapDeg - yawImuInicioGiro);

  float progresoAngular = turnSign * deltaYawImu;
  float objetivoAngular = fabsf(targetTurnDeg);
  float restanteDeg = objetivoAngular - progresoAngular;

  if (progresoAngular >= 0.0f && restanteDeg <= TOL_GIRO_DEG) {
    detenerMotores();
    estadoMovimiento = IDLE;

    // yawDeg ya es continuo (yaw IMU + offset): no hay nada que sumar.

    if (gotoPending) {
      comenzarAvance(gotoDistanceMm, gotoMarcar, gotoHeadingDeg);
    } else {
      enviarLinea("POINT_REACHED");
    }

    return;
  }

  if (millis() - movementStartMs > TIMEOUT_MOV_MS) {
    detenerMotores();
    estadoMovimiento = IDLE;
    gotoPending = false;

    enviarLinea("ERROR:TURN_TIMEOUT");
    return;
  }

  int pwm = PWM_GIRO;

  const float DESACEL_GIRO_DEG = 35.0f;

  if (restanteDeg < DESACEL_GIRO_DEG) {
    float factor = constrain(restanteDeg / DESACEL_GIRO_DEG, 0.0f, 1.0f);

    pwm = PWM_MIN_GIRO + (int)((PWM_GIRO - PWM_MIN_GIRO) * factor);
  }

  if (turnSign > 0) {
    controlMotores(+pwm, -pwm);
  } else {
    controlMotores(-pwm, +pwm);
  }
}

// ============================================================
// GOTO X,Y[,MARCAR]
// ============================================================
void irA(float xObjetivo, float yObjetivo, bool marcar) {
  float dx = xObjetivo - posX;
  float dy = yObjetivo - posY;

  float distancia = sqrt(dx * dx + dy * dy);
  float headingObjetivo = degrees(atan2(dy, dx));
  float giroNecesario = normalizeAngleDeg(headingObjetivo - yawDeg);

  gotoTargetX = xObjetivo;
  gotoTargetY = yObjetivo;
  gotoDistanceMm = distancia;
  gotoHeadingDeg = headingObjetivo;
  gotoMarcar = marcar;

  enviarLinea(
    "GOTO_ACCEPTED:" + String(xObjetivo, 2) + "," + String(yObjetivo, 2) +
    (marcar ? ",MARK" : "")
  );

  if (distancia <= TOL_DIST_MM) {
    posX = xObjetivo;
    posY = yObjetivo;

    enviarLinea("POINT_REACHED");
    return;
  }

  if (fabsf(giroNecesario) > TOL_GIRO_DEG) {
    comenzarGiro(giroNecesario, true);
  } else {
    gotoPending = true;
    comenzarAvance(distancia, marcar, headingObjetivo);
  }
}

// ============================================================
// TELEMETRIA
// ============================================================
void enviarTelemetria() {
  if (millis() - lastTelemetryMs < TELEMETRIA_MS) {
    return;
  }

  lastTelemetryMs = millis();

  float herr = (estadoMovimiento == MOVING_STRAIGHT) ? errorRumboDeg() : 0.0f;

  char buf[420];

  snprintf(
    buf,
    sizeof(buf),
    "TEL,ENC1=%ld,ENC2=%ld,DL=%.2f,DR=%.2f,X=%.2f,Y=%.2f,YAW=%.2f,YAW_IMU=%.2f,"
    "GYRO_Z_RAW=%.5f,BIAS_Z=%.5f,GYRO_Z=%.5f,DT_MS=%.3f,IMU=%d,HERR=%.2f,"
    "PUMP=%d,PEN=%d,MARK=%d,E1=%d,E2=%d,M1=%d,M2=%d",
    countLeft,
    countRight,
    countsToMm(countLeft),
    countsToMm(countRight),
    posX,
    posY,
    yawDeg,
    yawImuDeg,
    gyroZRawDps,
    gyroZBiasDps,
    gyroZDps,
    dtImuMs,
    imuOK ? 1 : 0,
    herr,
    pumpPwmActual,
    pumpEnabled ? 1 : 0,
    marcarActual ? 1 : 0,
    enc1Enabled ? 1 : 0,
    enc2Enabled ? 1 : 0,
    motor1Enabled ? 1 : 0,
    motor2Enabled ? 1 : 0
  );

  enviarLinea(String(buf));
}

// ============================================================
// STOP / RESET
// ============================================================
void stopAGV() {
  emergencyStop = true;
  gotoPending = false;
  estadoMovimiento = IDLE;

  detenerMotores();
  apagarBombaYa();

  enviarLinea("STOPPED");
}

void resetOrigen() {
  detenerMotores();
  apagarBombaYa();

  estadoMovimiento = IDLE;
  emergencyStop = false;
  gotoPending = false;

  posX = 0.0f;
  posY = 0.0f;
  yawDeg = 0.0f;
  yawOffsetDeg = 0.0f;

  yawImuDeg = 0.0f;
  yawImuUnwrapDeg = 0.0;
  yawImuInicioGiro = 0.0;
  lastImuMicros = micros();

  resetEncoders();

  enviarLinea("ACK:RESET");
}

// ============================================================
// PARAMETROS AJUSTABLES POR TCP:  SET:NOMBRE=VALOR   /   GET
// ============================================================
bool setParametro(String nombre, float v) {
  nombre.toUpperCase();

  if (nombre == "KP_HDG")            KP_HDG = v;
  else if (nombre == "KD_HDG")       KD_HDG = v;
  else if (nombre == "MAX_CORR")     MAX_CORR_PWM = constrain((int)v, 0, 100);
  else if (nombre == "HDG_HOLD")     HEADING_HOLD = (v != 0.0f);
  else if (nombre == "KP_ENC")       KP_ENCODERS = v;
  else if (nombre == "PWM_AVANCE")   PWM_AVANCE = constrain((int)v, PWM_MIN, PWM_MAX);
  else if (nombre == "PWM_GIRO")     PWM_GIRO = constrain((int)v, PWM_MIN_GIRO, PWM_MAX);
  else if (nombre == "PUMP_PWM")     PUMP_PWM_NOMINAL = constrain((int)v, 0, 255);
  else if (nombre == "PUMP_PWM_MIN") PUMP_PWM_MIN = constrain((int)v, 0, 255);
  else if (nombre == "PUMP_PRE_MS")  PUMP_PREFLOW_MS = (unsigned long)constrain(v, 0.0f, 5000.0f);
  else if (nombre == "PUMP_END_MM")  PUMP_END_MM = constrain(v, 0.0f, 200.0f);
  else if (nombre == "PUMP_SCALE")   PUMP_SCALE_WITH_SPEED = (v != 0.0f);
  else if (nombre == "AUTOBIAS")     IMU_AUTOBIAS = (v != 0.0f);
  else return false;

  return true;
}

void enviarParametros() {
  char buf[400];

  snprintf(
    buf,
    sizeof(buf),
    "PARAMS,KP_HDG=%.3f,KD_HDG=%.3f,MAX_CORR=%d,HDG_HOLD=%d,KP_ENC=%.3f,"
    "PWM_AVANCE=%d,PWM_GIRO=%d,PUMP_PWM=%d,PUMP_PWM_MIN=%d,PUMP_PRE_MS=%lu,"
    "PUMP_END_MM=%.1f,PUMP_SCALE=%d,AUTOBIAS=%d",
    KP_HDG,
    KD_HDG,
    MAX_CORR_PWM,
    HEADING_HOLD ? 1 : 0,
    KP_ENCODERS,
    PWM_AVANCE,
    PWM_GIRO,
    PUMP_PWM_NOMINAL,
    PUMP_PWM_MIN,
    PUMP_PREFLOW_MS,
    PUMP_END_MM,
    PUMP_SCALE_WITH_SPEED ? 1 : 0,
    IMU_AUTOBIAS ? 1 : 0
  );

  enviarLinea(String(buf));
}

// ============================================================
// PROTOCOLO TCP
//
//  PING | START | STOP | RESET | CALIB | GET
//  ENC1_ON/OFF  ENC2_ON/OFF  MOTOR1_ON/OFF  MOTOR2_ON/OFF
//  MOVE:mm[,marcar]          ej: MOVE:1000,1
//  TURN:grados
//  GOTO:x,y[,marcar]         ej: GOTO:500,0,1
//  PUMP_EN:1|0   PUMP_ON   PUMP_OFF   PUMP_PWM:0..255
//  SET:NOMBRE=VALOR
//  SETPOSE:x,y,yaw           (para correccion externa, ej. AprilTag)
// ============================================================
void procesarComando(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) return;

  if (cmd == "PING") {
    enviarLinea("ACK:PING");
    return;
  }

  if (cmd == "START") {
    emergencyStop = false;
    enviarLinea("ACK:START");
    return;
  }

  if (cmd == "STOP") {
    stopAGV();
    return;
  }

  if (cmd == "RESET") {
    resetOrigen();
    return;
  }

  if (cmd == "GET") {
    enviarParametros();
    return;
  }

  if (cmd == "CALIB") {
    if (estadoMovimiento != IDLE) {
      enviarLinea("ERROR:BUSY");
      return;
    }

    calibrarGyroZ(false);
    enviarLinea("ACK:CALIB,BIAS_Z=" + String(gyroZBiasDps, 5));
    return;
  }

  if (cmd == "ENC1_ON")  { enc1Enabled = true;  enviarLinea("ACK:ENC1_ON");  return; }
  if (cmd == "ENC1_OFF") { enc1Enabled = false; enviarLinea("ACK:ENC1_OFF"); return; }
  if (cmd == "ENC2_ON")  { enc2Enabled = true;  enviarLinea("ACK:ENC2_ON");  return; }
  if (cmd == "ENC2_OFF") { enc2Enabled = false; enviarLinea("ACK:ENC2_OFF"); return; }

  if (cmd == "MOTOR1_ON") { motor1Enabled = true; enviarLinea("ACK:MOTOR1_ON"); return; }

  if (cmd == "MOTOR1_OFF") {
    motor1Enabled = false;
    detenerMotores();
    enviarLinea("ACK:MOTOR1_OFF");
    return;
  }

  if (cmd == "MOTOR2_ON") { motor2Enabled = true; enviarLinea("ACK:MOTOR2_ON"); return; }

  if (cmd == "MOTOR2_OFF") {
    motor2Enabled = false;
    detenerMotores();
    enviarLinea("ACK:MOTOR2_OFF");
    return;
  }

  // ---------------- BOMBA ----------------
  if (cmd.startsWith("PUMP_EN:")) {
    pumpEnabled = (cmd.substring(8).toInt() != 0);

    if (!pumpEnabled) {
      apagarBombaYa();
    }

    enviarLinea(String("ACK:PUMP_EN:") + (pumpEnabled ? "1" : "0"));
    return;
  }

  if (cmd == "PUMP_ON") {
    if (!pumpEnabled) {
      enviarLinea("ERROR:PUMP_DISABLED");
      return;
    }

    if (estadoMovimiento != IDLE) {
      enviarLinea("ERROR:BUSY");
      return;
    }

    emergencyStop = false;
    pumpManual = true;
    pumpManualInicioMs = millis();

    enviarLinea("ACK:PUMP_ON");
    return;
  }

  if (cmd == "PUMP_OFF") {
    pumpManual = false;
    escribirBomba(0);

    enviarLinea("ACK:PUMP_OFF");
    return;
  }

  if (cmd.startsWith("PUMP_PWM:")) {
    PUMP_PWM_NOMINAL = constrain(cmd.substring(9).toInt(), 0, 255);

    enviarLinea("ACK:PUMP_PWM:" + String(PUMP_PWM_NOMINAL));
    return;
  }

  // ---------------- AJUSTES ----------------
  if (cmd.startsWith("SET:")) {
    String data = cmd.substring(4);
    int eq = data.indexOf('=');

    if (eq < 0) {
      enviarLinea("ERROR:BAD_SET");
      return;
    }

    String nombre = data.substring(0, eq);
    float valor = data.substring(eq + 1).toFloat();

    if (setParametro(nombre, valor)) {
      enviarLinea("ACK:SET:" + nombre);
      enviarParametros();
    } else {
      enviarLinea("ERROR:UNKNOWN_PARAM:" + nombre);
    }

    return;
  }

  if (cmd.startsWith("SETPOSE:")) {
    if (estadoMovimiento != IDLE) {
      enviarLinea("ERROR:BUSY");
      return;
    }

    String p[3];
    int n = dividir(cmd.substring(8), ',', p, 3);

    if (n < 3) {
      enviarLinea("ERROR:BAD_SETPOSE");
      return;
    }

    posX = p[0].toFloat();
    posY = p[1].toFloat();

    // El offset hace que yawDeg valga lo pedido sin tocar el yaw del IMU.
    yawOffsetDeg = normalizeAngleDeg(p[2].toFloat() - yawImuDeg);
    yawDeg = normalizeAngleDeg(yawImuDeg + yawOffsetDeg);

    enviarLinea("ACK:SETPOSE");
    return;
  }

  // ---------------- MOVIMIENTO ----------------
  if (cmd.startsWith("MOVE:")) {
    if (estadoMovimiento != IDLE) {
      enviarLinea("ERROR:BUSY");
      return;
    }

    String p[2];
    int n = dividir(cmd.substring(5), ',', p, 2);

    float dist = p[0].toFloat();
    bool marcar = (n >= 2) && (p[1].toInt() != 0);

    gotoPending = false;

    enviarLinea("ACK:MOVE");

    // Mantiene el rumbo que tenia el AGV al arrancar
    comenzarAvance(dist, marcar, yawDeg);

    return;
  }

  if (cmd.startsWith("TURN:")) {
    if (estadoMovimiento != IDLE) {
      enviarLinea("ERROR:BUSY");
      return;
    }

    gotoPending = false;

    enviarLinea("ACK:TURN");

    comenzarGiro(cmd.substring(5).toFloat(), false);

    return;
  }

  if (cmd.startsWith("GOTO:")) {
    if (estadoMovimiento != IDLE) {
      enviarLinea("ERROR:BUSY");
      return;
    }

    String p[3];
    int n = dividir(cmd.substring(5), ',', p, 3);

    if (n < 2) {
      enviarLinea("ERROR:BAD_GOTO");
      return;
    }

    float x = p[0].toFloat();
    float y = p[1].toFloat();
    bool marcar = (n >= 3) && (p[2].toInt() != 0);

    emergencyStop = false;

    enviarLinea("ACK:GOTO");

    irA(x, y, marcar);

    return;
  }

  enviarLinea("ERROR:UNKNOWN_COMMAND:" + cmd);
}

// ============================================================
// WIFI
// ============================================================
void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi conectado");

  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());

  server.begin();
  server.setNoDelay(true);

  Serial.print("Servidor TCP puerto ");
  Serial.println(TCP_PORT);
}

// ============================================================
// OTA
// ============================================================
void configurarOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);

  // ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    otaEnProgreso = true;

    emergencyStop = true;
    gotoPending = false;
    estadoMovimiento = IDLE;

    detenerMotores();
    apagarBombaYa();

    Serial.println("OTA:START - motores y bomba detenidos");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println();
    Serial.println("OTA:END");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress * 100U) / total;

    Serial.printf("OTA:%u%%\r", percent);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA ERROR[%u]\n", error);
  });

  ArduinoOTA.begin();

  Serial.print("OTA listo. Host: ");
  Serial.println(OTA_HOSTNAME);
}

// ============================================================
// CLIENTE TCP
// ============================================================
void mantenerCliente() {
  if (!cliente || !cliente.connected()) {
    WiFiClient nuevoCliente = server.available();

    if (nuevoCliente) {
      if (cliente) {
        cliente.stop();
      }

      cliente = nuevoCliente;
      cliente.setNoDelay(true);

      tcpBuffer = "";

      enviarLinea("READY");
      enviarLinea("ACK:TCP_CONNECTED");
    }

    return;
  }

  while (cliente.available()) {
    char c = cliente.read();

    if (c == '\n') {
      procesarComando(tcpBuffer);
      tcpBuffer = "";
    }
    else if (c != '\r') {
      tcpBuffer += c;

      if (tcpBuffer.length() > 120) {
        tcpBuffer = "";
        enviarLinea("ERROR:TCP_BUFFER");
      }
    }
  }
}

// ============================================================
// ACTUALIZACION DE MOVIMIENTO
// ============================================================
void actualizarMovimiento() {
  if (emergencyStop || otaEnProgreso) {
    detenerMotores();
    return;
  }

  if (estadoMovimiento == MOVING_STRAIGHT) {
    actualizarAvance();
  }
  else if (estadoMovimiento == TURNING) {
    actualizarGiro();
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // La bomba y los motores arrancan apagados.
  pinMode(PUMP_PIN, OUTPUT);
  escribirBomba(0);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  detenerMotores();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  delay(200);

  // IMPORTANTE: mantener el AGV quieto durante el arranque.
  if (iniciarMPU6050()) {
    calibrarGyroZ(true);
  } else {
    imuOK = false;
    Serial.println("ERROR: MPU6050 no encontrado en TCA canal 0");
  }

  resetEncoders();

  conectarWiFi();

  configurarOTA();

  Serial.println("READY");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  mantenerCliente();

  ArduinoOTA.handle();

  leerEncoders();

  actualizarIMU();

  actualizarMovimiento();

  actualizarBomba();

  enviarTelemetria();

  delay(2);
}
