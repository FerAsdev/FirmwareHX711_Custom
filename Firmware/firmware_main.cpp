#include "HX711.h"
#include <Joystick.h>
#include <EEPROM.h>
#include <math.h>

// =======================
// Pines HX711
// =======================
#define DOUT 3
#define CLK 2

HX711 scale;

// =======================
// Joystick USB
// =======================
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  0, 0,
  false, false, false,   // X, Y, Z
  false, false, false,   // Rx, Ry, Rz
  true, false,           // Rudder, Throttle
  false, false, false    // Accelerator, Brake, Steering
);

// =======================
// Configuración guardada
// =======================
struct PedalConfig {
  uint32_t magic;
  long zeroOffset;
  long maxBrakeRaw;
  float curveExponent;
  int deadzoneRaw;
  bool invertSignal;
};

const uint32_t CONFIG_MAGIC = 0xBEEF7111;
const int EEPROM_ADDR = 0;

PedalConfig config;

// =======================
// Valores por defecto
// =======================
const long DEFAULT_MAX_BRAKE_RAW = 1500000;
const float DEFAULT_CURVE = 1.0;       // Más directo para simracing
const int DEFAULT_DEADZONE_RAW = 5000; // Evita ruido inicial
const bool DEFAULT_INVERT_SIGNAL = false;

// =======================
// Ajustes de respuesta
// =======================
// Más alto = más rápido, menos suave
// Para simracing queremos poca latencia.
const float ALPHA_PRESS = 0.85;   // Al presionar
const float ALPHA_RELEASE = 1.0;  // Al soltar, instantáneo

// Intervalo de envío USB.
// 2 ms = hasta 500 Hz de reporte USB, aunque el HX711 limite el dato real.
const unsigned long JOYSTICK_INTERVAL_MS = 2;

// =======================
// EEPROM
// =======================
void loadConfig() {
  EEPROM.get(EEPROM_ADDR, config);

  if (config.magic != CONFIG_MAGIC) {
    config.magic = CONFIG_MAGIC;
    config.zeroOffset = 0;
    config.maxBrakeRaw = DEFAULT_MAX_BRAKE_RAW;
    config.curveExponent = DEFAULT_CURVE;
    config.deadzoneRaw = DEFAULT_DEADZONE_RAW;
    config.invertSignal = DEFAULT_INVERT_SIGNAL;

    EEPROM.put(EEPROM_ADDR, config);
  }
}

void saveConfig() {
  EEPROM.put(EEPROM_ADDR, config);
}

// =======================
// Lectura HX711
// =======================
long readRawAverage(byte samples = 1) {
  long sum = 0;
  byte validSamples = 0;

  for (byte i = 0; i < samples; i++) {
    unsigned long startTime = millis();

    while (!scale.is_ready()) {
      if (millis() - startTime > 120) {
        break;
      }
      delay(1);
    }

    if (scale.is_ready()) {
      sum += scale.read();
      validSamples++;
    }
  }

  if (validSamples == 0) {
    return config.zeroOffset;
  }

  return sum / validSamples;
}

long getPedalRaw() {
  // Para correr usamos 1 muestra para menor delay
  long raw = readRawAverage(1);
  long value = raw - config.zeroOffset;

  if (config.invertSignal) {
    value = -value;
  }

  if (value < config.deadzoneRaw) {
    value = 0;
  }

  return value;
}

int rawToJoystick(long value) {
  if (value < 0) value = 0;
  if (value > config.maxBrakeRaw) value = config.maxBrakeRaw;

  float normalized = (float)value / (float)config.maxBrakeRaw;

  if (normalized < 0.0) normalized = 0.0;
  if (normalized > 1.0) normalized = 1.0;

  normalized = pow(normalized, config.curveExponent);

  int joystickValue = (int)(normalized * 1023.0);

  if (joystickValue < 0) joystickValue = 0;
  if (joystickValue > 1023) joystickValue = 1023;

  return joystickValue;
}

// =======================
// Calibración
// =======================
void tarePedal() {
  Serial.println("No pises el pedal. Calculando tara...");
  delay(2500);

  // Para tara sí promediamos más
  config.zeroOffset = readRawAverage(30);
  saveConfig();

  Serial.print("Tara guardada. Zero Offset = ");
  Serial.println(config.zeroOffset);
}

void calibrateMax() {
  Serial.println("Presiona el pedal fuerte y mantenlo presionado...");
  delay(2000);

  long maxValue = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 4000) {
    long value = getPedalRaw();

    if (value > maxValue) {
      maxValue = value;
    }

    Serial.print("Leyendo max: ");
    Serial.println(value);

    delay(50);
  }

  if (maxValue > 1000) {
    config.maxBrakeRaw = maxValue;
    saveConfig();

    Serial.print("Max guardado: ");
    Serial.println(config.maxBrakeRaw);
  } else {
    Serial.println("Max demasiado bajo. No se guardo.");
    Serial.println("Prueba mandar INVERT y luego MAX otra vez.");
  }
}

void resetConfig() {
  config.magic = CONFIG_MAGIC;
  config.zeroOffset = 0;
  config.maxBrakeRaw = DEFAULT_MAX_BRAKE_RAW;
  config.curveExponent = DEFAULT_CURVE;
  config.deadzoneRaw = DEFAULT_DEADZONE_RAW;
  config.invertSignal = DEFAULT_INVERT_SIGNAL;

  saveConfig();

  Serial.println("Configuracion reiniciada.");
}

// =======================
// Debug / Estado
// =======================
void printStatus() {
  Serial.println("===== PEDAL STATUS =====");

  Serial.print("Zero Offset: ");
  Serial.println(config.zeroOffset);

  Serial.print("Max Brake Raw: ");
  Serial.println(config.maxBrakeRaw);

  Serial.print("Curve Exponent: ");
  Serial.println(config.curveExponent, 3);

  Serial.print("Deadzone Raw: ");
  Serial.println(config.deadzoneRaw);

  Serial.print("Invert Signal: ");
  Serial.println(config.invertSignal ? "true" : "false");

  long raw = readRawAverage(1);
  long delta = raw - config.zeroOffset;

  long pedal = delta;
  if (config.invertSignal) {
    pedal = -pedal;
  }

  long pedalClamped = pedal;
  if (pedalClamped < config.deadzoneRaw) {
    pedalClamped = 0;
  }

  int joy = rawToJoystick(pedalClamped);

  Serial.print("Current RAW: ");
  Serial.println(raw);

  Serial.print("Current Delta: ");
  Serial.println(delta);

  Serial.print("Current Pedal sin clamp: ");
  Serial.println(pedal);

  Serial.print("Current Pedal: ");
  Serial.println(pedalClamped);

  Serial.print("Current Joystick: ");
  Serial.println(joy);

  Serial.println("========================");
}

void printRawOnce() {
  long raw = readRawAverage(1);
  long delta = raw - config.zeroOffset;

  long pedal = delta;
  if (config.invertSignal) {
    pedal = -pedal;
  }

  long pedalClamped = pedal;
  if (pedalClamped < config.deadzoneRaw) {
    pedalClamped = 0;
  }

  int joy = rawToJoystick(pedalClamped);

  Serial.print("RAW: ");
  Serial.print(raw);

  Serial.print(" | Delta: ");
  Serial.print(delta);

  Serial.print(" | Invert: ");
  Serial.print(config.invertSignal ? "true" : "false");

  Serial.print(" | Pedal sin clamp: ");
  Serial.print(pedal);

  Serial.print(" | PEDAL: ");
  Serial.print(pedalClamped);

  Serial.print(" | JOY: ");
  Serial.println(joy);
}

void monitorPedal() {
  Serial.println("Monitor activo por 10 segundos...");
  Serial.println("Pisa y suelta el pedal para ver los valores.");

  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    printRawOnce();
    delay(50);
  }

  Serial.println("Monitor terminado.");
}

// =======================
// Comandos Serial
// =======================
void processCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "HELP") {
    Serial.println("Comandos disponibles:");
    Serial.println("STATUS");
    Serial.println("RAW");
    Serial.println("MONITOR");
    Serial.println("TARE");
    Serial.println("MAX");
    Serial.println("INVERT");
    Serial.println("CURVE 1.0");
    Serial.println("DEADZONE 5000");
    Serial.println("SETMAX 1500000");
    Serial.println("RESET");
  }
  else if (command == "STATUS") {
    printStatus();
  }
  else if (command == "RAW") {
    printRawOnce();
  }
  else if (command == "MONITOR") {
    monitorPedal();
  }
  else if (command == "TARE") {
    tarePedal();
  }
  else if (command == "MAX") {
    calibrateMax();
  }
  else if (command == "INVERT") {
    config.invertSignal = !config.invertSignal;
    saveConfig();

    Serial.print("Invert Signal ahora es: ");
    Serial.println(config.invertSignal ? "true" : "false");
  }
  else if (command.startsWith("CURVE ")) {
    float value = command.substring(6).toFloat();

    if (value >= 0.5 && value <= 3.0) {
      config.curveExponent = value;
      saveConfig();

      Serial.print("Curva guardada: ");
      Serial.println(config.curveExponent, 3);
    } else {
      Serial.println("Valor invalido. Usa CURVE 0.5 a CURVE 3.0");
    }
  }
  else if (command.startsWith("DEADZONE ")) {
    int value = command.substring(9).toInt();

    if (value >= 0 && value <= 100000) {
      config.deadzoneRaw = value;
      saveConfig();

      Serial.print("Deadzone guardada: ");
      Serial.println(config.deadzoneRaw);
    } else {
      Serial.println("Valor invalido. Usa DEADZONE 0 a 100000");
    }
  }
  else if (command.startsWith("SETMAX ")) {
    long value = command.substring(7).toInt();

    if (value > 1000) {
      config.maxBrakeRaw = value;
      saveConfig();

      Serial.print("Max manual guardado: ");
      Serial.println(config.maxBrakeRaw);
    } else {
      Serial.println("Valor invalido.");
    }
  }
  else if (command == "RESET") {
    resetConfig();
  }
  else {
    Serial.println("Comando no reconocido. Usa HELP.");
  }
}

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(115200);
  delay(1500);

  loadConfig();

  scale.begin(DOUT, CLK);

  Joystick.begin(false);
  Joystick.setRudderRange(0, 1023);

  Serial.println("Pedal HX711 iniciado - version rapida.");
  Serial.println("Comandos: HELP, STATUS, RAW, MONITOR, TARE, MAX, INVERT, RESET");

  unsigned long startTime = millis();

  while (!scale.is_ready() && millis() - startTime < 5000) {
    Serial.println("Esperando HX711...");
    delay(500);
  }

  if (scale.is_ready()) {
    Serial.println("HX711 listo.");
  } else {
    Serial.println("HX711 no listo al inicio, seguira intentando en loop.");
  }
}

// =======================
// LOOP
// =======================
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  static unsigned long lastRead = 0;
  static float smoothBrake = 0;

  if (millis() - lastRead >= JOYSTICK_INTERVAL_MS) {
    lastRead = millis();

    long pedalRaw = getPedalRaw();
    int brakeValue = rawToJoystick(pedalRaw);

    float alpha;

    if (brakeValue > smoothBrake) {
      // Al presionar: rápido pero con leve suavizado
      alpha = ALPHA_PRESS;
    } else {
      // Al soltar: instantáneo
      alpha = ALPHA_RELEASE;
    }

    smoothBrake = smoothBrake + alpha * (brakeValue - smoothBrake);

    if (smoothBrake < 0) smoothBrake = 0;
    if (smoothBrake > 1023) smoothBrake = 1023;

    Joystick.setRudder((int)smoothBrake);
    Joystick.sendState();
  }
}