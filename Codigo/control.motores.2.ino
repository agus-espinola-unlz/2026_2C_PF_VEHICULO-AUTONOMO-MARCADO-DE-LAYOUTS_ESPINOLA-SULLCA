#include <Wire.h>
#include <AS5600.h>
#include <Math.h>

// ======================================================
// BUSES I2C
// ======================================================

TwoWire I2C_Bus1 = TwoWire(0);
TwoWire I2C_Bus2 = TwoWire(1);

// ======================================================
// ENCODERS
// ======================================================

AS5600 encoderDerecho(&I2C_Bus1);
AS5600 encoderIzquierdo(&I2C_Bus2);

// ======================================================
// MOTORES
// ======================================================

const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;

const int IN3 = 12;
const int IN4 = 13;
const int ENB = 14;

// ======================================================
// POSICIÓN VIRTUAL
// ======================================================

float pos_x = 0;
float pos_y = 0;
float angulo_actual = 0;


void pararMotores() {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);


  // I2C
  I2C_Bus1.begin(21, 22, 400000);
  I2C_Bus2.begin(32, 33, 400000);

  // Encoders
  encoderDerecho.begin();
  encoderIzquierdo.begin();

  // Pines motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pararMotores();

  Serial.println("ROBOT LISTO");
}

void avanzar(float distancia_mm) {

  Serial.println("=================================");
  Serial.println("INICIO AVANCE");

  // =====================================
  // RUEDA
  // =====================================

  const float diametro = 65.0;

  const float circunferencia =
      PI * diametro;

  // =====================================
  // DISTANCIA -> GRADOS OBJETIVO
  // =====================================

  float gradosObjetivo =
      (distancia_mm / circunferencia) * 360.0;

  Serial.print("Grados objetivo: ");
  Serial.println(gradosObjetivo);

  // =====================================
  // ENCODER
  // =====================================

  long inicio =
      encoderDerecho.readAngle();

  long anterior = inicio;

  float gradosAcumulados = 0;

  Serial.print("Encoder inicial: ");
  Serial.println(inicio);

  // =====================================
  // MOTORES ADELANTE
  // =====================================

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // =====================================
  // CONTROL MOVIMIENTO
  // =====================================

  while (gradosAcumulados < gradosObjetivo - 2) { //ese -2 es para que empiece a frenar antes y asi no se pasa

    long actual =
        encoderDerecho.readAngle();

    long delta =
        actual - anterior;

    // =================================
    // WRAP
    // =================================

    if (delta > 2048) {
      delta -= 4096;
    }

    if (delta < -2048) {
      delta += 4096;
    }

    // =================================
    // TICKS -> GRADOS
    // =================================

    float gradosDelta =
        (delta * 360.0) / 4096.0;

    gradosAcumulados += abs(gradosDelta);

    anterior = actual;

        // =================================
    // ERROR RESTANTE
    // =================================

    float error =
        gradosObjetivo - gradosAcumulados;

    // =================================
    // CONTROL SIMPLE VELOCIDAD
    // =================================

    int velocidad;

    if (error > 180) {

      velocidad = 120;

    } else if (error > 90) {

      velocidad = 90;

    } else{

      velocidad = 45;

    }

    // =================================
    // APLICAR PWM
    // =================================

    analogWrite(ENA, velocidad);
    analogWrite(ENB, velocidad);

    Serial.print("Grados: ");
    Serial.println(gradosAcumulados);

    delay(5);
  }
  // =====================================
  // STOP
  // =====================================

  pararMotores();

  Serial.println("FIN AVANCE");
}

// ======================================================
// GIRAR
// ======================================================

void girar(float grados) {

  if (grados > 0) {

    // Derecha
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

  } else {

    // Izquierda
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);

  // AJUSTAR TIEMPO
  delay(abs(grados) * 8);

  pararMotores();

  angulo_actual += grados;
}

// ======================================================
// IR A PUNTO
// ======================================================

void irA(float x_obj, float y_obj) {

  // =========================================
  // DISTANCIA REAL
  // =========================================

  float distancia =
      sqrt((x_obj * x_obj) + (y_obj * y_obj));

  Serial.println("===============");

  Serial.print("X: ");
  Serial.println(x_obj);

  Serial.print("Y: ");
  Serial.println(y_obj);

  Serial.print("Distancia: ");
  Serial.println(distancia);

  avanzar(distancia);

  Serial.println("ACK");
}

// ======================================================
// LOOP
// ======================================================

void loop() {

  // =====================================
  // LEER SERIAL
  // =====================================

  if (Serial.available()) {

    String data =
        Serial.readStringUntil('\n');

    data.trim();

    Serial.print("RECIBIDO -> ");
    Serial.println(data);

    // =====================================
    // GOTO:X,Y
    // =====================================

    if (data.startsWith("GOTO:")) {

      String coords =
          data.substring(5);

      int coma =
          coords.indexOf(',');

      if (coma > 0) {

        float x =
            coords.substring(0, coma).toFloat();

        float y =
            coords.substring(coma + 1).toFloat();

        irA(x, y);

        Serial.println("ACK");
      }
    }

    // =====================================
    // GIRAR:XX
    // =====================================

    else if (data.startsWith("GIRAR:")) {

      float grados =
          data.substring(6).toFloat();

      girar(grados);

      Serial.println("ACK");
    }

    // =====================================
    // STOP
    // =====================================

    else if (data == "STOP") {

      pararMotores();

      Serial.println("STOPPED");
    }
  }
}