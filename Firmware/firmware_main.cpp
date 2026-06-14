#include "HX711.h"
#include <Joystick.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// =======================
// Pines HX711
// =======================
#define DOUT 3
#define CLK 2

// El HX711 selecciona 10/80 SPS mediante su pin fisico RATE.
// Usa -1 si el modulo ya viene configurado a 80 SPS o no expone RATE.
// Si conectas RATE a un pin de la Pro Micro, escribe aqui ese numero.
#define HX711_RATE_PIN -1

HX711 scale;

// =======================
// Joystick USB
// =======================
// ATS puede rechazar la asignacion si varios ejes cambian simultaneamente.
// Un solo eje X generico ofrece la mayor compatibilidad DirectInput.
#define HID_SINGLE_AXIS_COMPATIBILITY 1

#if HID_SINGLE_AXIS_COMPATIBILITY
  #define REPORT_X_AXIS 1
  #define REPORT_Z_AXIS 0
  #define REPORT_RUDDER_AXIS 0
  #define REPORT_BRAKE_AXIS 0
#else
  #define REPORT_X_AXIS 0
  #define REPORT_Z_AXIS 1
  #define REPORT_RUDDER_AXIS 1
  #define REPORT_BRAKE_AXIS 1
#endif

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  1, 0, // Un boton inactivo ayuda a enumerar como joystick DirectInput.
  REPORT_X_AXIS, false, REPORT_Z_AXIS, // X, Y, Z
  false, false, false,   // Rx, Ry, Rz
  REPORT_RUDDER_AXIS, false, // Rudder, Throttle
  false, REPORT_BRAKE_AXIS, false // Accelerator, Brake, Steering
);

// =======================
// Configuracion guardada
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
const float DEFAULT_CURVE = 1.0;
const int DEFAULT_DEADZONE_RAW = 5000;
const bool DEFAULT_INVERT_SIGNAL = false;

// =======================
// Ajustes de respuesta
// =======================
// A 80 SPS hay una muestra nueva aproximadamente cada 12.5 ms.
// El USB reporta mas rapido para que el simulador vea el ultimo valor estable.
const unsigned int EXPECTED_HX711_SPS = 80;
const unsigned long JOYSTICK_INTERVAL_MS = 2;
const unsigned long HX711_STALE_MS = 250;

// Mas alto = mas rapido, menos filtrado. El release instantaneo evita freno pegado.
const float ALPHA_PRESS = 0.90;
const float ALPHA_RELEASE = 1.0;
const int JOYSTICK_NOISE_BAND = 1;

// =======================
// Estado runtime
// =======================
long latestRaw = 0;
bool hasLatestRaw = false;
unsigned long lastSampleMs = 0;
unsigned long lastSampleUs = 0;
unsigned long measuredSampleIntervalUs = 0;
unsigned long totalHx711Samples = 0;

int targetBrake = 0;
float smoothBrake = 0.0;

char commandBuffer[40];
byte commandLength = 0;

// =======================
// EEPROM
// =======================
void setDefaultConfig() {
  config.magic = CONFIG_MAGIC;
  config.zeroOffset = 0;
  config.maxBrakeRaw = DEFAULT_MAX_BRAKE_RAW;
  config.curveExponent = DEFAULT_CURVE;
  config.deadzoneRaw = DEFAULT_DEADZONE_RAW;
  config.invertSignal = DEFAULT_INVERT_SIGNAL;
}

bool configIsValid() {
  if (config.magic != CONFIG_MAGIC) return false;
  if (config.maxBrakeRaw < 1000 || config.maxBrakeRaw > 10000000L) return false;
  if (config.deadzoneRaw < 0 || config.deadzoneRaw > 100000) return false;
  if (config.deadzoneRaw >= config.maxBrakeRaw) return false;
  if (isnan(config.curveExponent)) return false;
  if (config.curveExponent < 0.5 || config.curveExponent > 3.0) return false;

  return true;
}

void loadConfig() {
  EEPROM.get(EEPROM_ADDR, config);

  if (!configIsValid()) {
    setDefaultConfig();
    EEPROM.put(EEPROM_ADDR, config);
  }
}

void saveConfig() {
  EEPROM.put(EEPROM_ADDR, config);
}

// =======================
// Conversion de pedal
// =======================
long rawToDelta(long raw) {
  long value = raw - config.zeroOffset;

  if (config.invertSignal) {
    value = -value;
  }

  if (value < 0) {
    value = 0;
  }

  return value;
}

long applyDeadzone(long value) {
  if (value <= config.deadzoneRaw) {
    return 0;
  }

  return value - config.deadzoneRaw;
}

int rawToJoystick(long value) {
  if (value < 0) value = 0;

  long usableMax = config.maxBrakeRaw - config.deadzoneRaw;
  if (usableMax < 1000) usableMax = 1000;
  if (value > usableMax) value = usableMax;

  float normalized = (float)value / (float)usableMax;

  if (normalized < 0.0) normalized = 0.0;
  if (normalized > 1.0) normalized = 1.0;

  if (fabs(config.curveExponent - 1.0) > 0.001) {
    normalized = pow(normalized, config.curveExponent);
  }

  int joystickValue = (int)(normalized * 1023.0 + 0.5);

  if (joystickValue < 0) joystickValue = 0;
  if (joystickValue > 1023) joystickValue = 1023;

  return joystickValue;
}

void updateTargetFromRaw(long raw) {
  long delta = rawToDelta(raw);
  long pedal = applyDeadzone(delta);
  targetBrake = rawToJoystick(pedal);
}

void sendBrakeJoystickValue(int value) {
  if (value < 0) value = 0;
  if (value > 1023) value = 1023;

#if REPORT_X_AXIS
  Joystick.setXAxis(value);
#endif

#if REPORT_Z_AXIS
  Joystick.setZAxis(value);
#endif

#if REPORT_RUDDER_AXIS
  Joystick.setRudder(value);
#endif

#if REPORT_BRAKE_AXIS
  Joystick.setBrake(value);
#endif
}

void recordHx711Sample(long raw, bool updateJoystickTarget = true) {
  unsigned long nowUs = micros();

  if (lastSampleUs != 0) {
    unsigned long sampleIntervalUs = nowUs - lastSampleUs;

    if (measuredSampleIntervalUs == 0) {
      measuredSampleIntervalUs = sampleIntervalUs;
    } else {
      measuredSampleIntervalUs =
        (measuredSampleIntervalUs * 7UL + sampleIntervalUs) / 8UL;
    }
  }

  lastSampleUs = nowUs;
  latestRaw = raw;
  hasLatestRaw = true;
  lastSampleMs = millis();
  totalHx711Samples++;

  if (updateJoystickTarget) {
    updateTargetFromRaw(raw);
  }
}

float getMeasuredHx711Sps() {
  if (measuredSampleIntervalUs == 0) {
    return 0.0;
  }

  return 1000000.0 / (float)measuredSampleIntervalUs;
}

// =======================
// Lectura HX711
// =======================
bool readRawNonBlocking() {
  if (!scale.is_ready()) {
    return false;
  }

  recordHx711Sample(scale.read());

  return true;
}

long readRawBlocking(byte samples = 1, unsigned int timeoutPerSampleMs = 150) {
  long sum = 0;
  byte validSamples = 0;

  for (byte i = 0; i < samples; i++) {
    unsigned long startTime = millis();

    while (!scale.is_ready()) {
      if (millis() - startTime >= timeoutPerSampleMs) {
        break;
      }
      delay(1);
    }

    if (scale.is_ready()) {
      long raw = scale.read();
      sum += raw;
      validSamples++;
      recordHx711Sample(raw);
    }
  }

  if (validSamples == 0) {
    return hasLatestRaw ? latestRaw : config.zeroOffset;
  }

  return sum / validSamples;
}

long getPedalRawBlocking() {
  long raw = readRawBlocking(1);
  return applyDeadzone(rawToDelta(raw));
}

// =======================
// Calibracion
// =======================
void tarePedal() {
  Serial.println("No pises el pedal. Calculando tara...");
  delay(1200);

  config.zeroOffset = readRawBlocking(40, 180);
  saveConfig();

  targetBrake = 0;
  smoothBrake = 0;

  Serial.print("Tara guardada. Zero Offset = ");
  Serial.println(config.zeroOffset);
}

void calibrateMax() {
  Serial.println("Presiona el pedal fuerte y mantenlo presionado...");
  delay(1200);

  long maxValue = 0;
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  while (millis() - startTime < 4000) {
    if (scale.is_ready()) {
      long raw = scale.read();
      long value = rawToDelta(raw);

      recordHx711Sample(raw, false);

      if (value > maxValue) {
        maxValue = value;
      }

      if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.print("Leyendo max: ");
        Serial.println(value);
      }
    }
  }

  if (maxValue > 1000) {
    config.maxBrakeRaw = maxValue;

    if (config.deadzoneRaw >= config.maxBrakeRaw) {
      config.deadzoneRaw = DEFAULT_DEADZONE_RAW;
    }

    saveConfig();
    updateTargetFromRaw(latestRaw);

    Serial.print("Max guardado: ");
    Serial.println(config.maxBrakeRaw);
  } else {
    Serial.println("Max demasiado bajo. No se guardo.");
    Serial.println("Prueba mandar INVERT y luego MAX otra vez.");
  }
}

void resetConfig() {
  setDefaultConfig();
  saveConfig();

  targetBrake = 0;
  smoothBrake = 0;

  Serial.println("Configuracion reiniciada.");
}

// =======================
// Debug / Estado
// =======================
void printStatus() {
  long raw = readRawBlocking(1);
  long delta = rawToDelta(raw);
  long pedal = applyDeadzone(delta);
  int joy = rawToJoystick(pedal);

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

  Serial.print("Has HX711 Sample: ");
  Serial.println(hasLatestRaw ? "true" : "false");

  Serial.print("Last Sample Age ms: ");
  Serial.println(hasLatestRaw ? millis() - lastSampleMs : 0);

  Serial.print("HX711 Sample Rate: ");
  Serial.print(getMeasuredHx711Sps(), 1);
  Serial.println(" SPS");

  Serial.print("Expected HX711 Rate: ");
  Serial.print(EXPECTED_HX711_SPS);
  Serial.println(" SPS");

  Serial.print("Total HX711 Samples: ");
  Serial.println(totalHx711Samples);

  Serial.print("Current RAW: ");
  Serial.println(raw);

  Serial.print("Current Delta: ");
  Serial.println(delta);

  Serial.print("Current Pedal: ");
  Serial.println(pedal);

  Serial.print("Target Joystick: ");
  Serial.println(targetBrake);

  Serial.print("Current Joystick: ");
  Serial.println(joy);

  Serial.println("========================");
}

void printRawOnce() {
  long raw = readRawBlocking(1);
  long delta = rawToDelta(raw);
  long pedal = applyDeadzone(delta);
  int joy = rawToJoystick(pedal);

  Serial.print("RAW: ");
  Serial.print(raw);

  Serial.print(" | Delta: ");
  Serial.print(delta);

  Serial.print(" | Invert: ");
  Serial.print(config.invertSignal ? "true" : "false");

  Serial.print(" | PEDAL: ");
  Serial.print(pedal);

  Serial.print(" | JOY: ");
  Serial.println(joy);
}

void printHx711Rate() {
  float measuredSps = getMeasuredHx711Sps();

  Serial.print("HX711 RATE: ");
  Serial.print(measuredSps, 1);
  Serial.println(" SPS");

  if (measuredSps >= 60.0) {
    Serial.println("Modo detectado: aproximadamente 80 SPS.");
  } else if (measuredSps >= 7.0 && measuredSps <= 15.0) {
    Serial.println("Modo detectado: aproximadamente 10 SPS.");
    Serial.println("El pin fisico RATE del HX711 debe estar en HIGH/VCC para 80 SPS.");
  } else {
    Serial.println("Frecuencia aun inestable. Espera dos segundos y envia RATE otra vez.");
  }
}

void monitorPedal() {
  Serial.println("Monitor activo por 10 segundos...");
  Serial.println("Pisa y suelta el pedal para ver los valores.");

  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  while (millis() - startTime < 10000) {
    readRawNonBlocking();

    if (millis() - lastPrint >= 50) {
      lastPrint = millis();
      printRawOnce();
    }
  }

  Serial.println("Monitor terminado.");
}

// =======================
// Comandos Serial
// =======================
void printHelp() {
  Serial.println("Comandos disponibles:");
  Serial.println("STATUS");
  Serial.println("RATE");
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

char *trimCommand(char *command) {
  while (*command == ' ' || *command == '\t') {
    command++;
  }

  int len = strlen(command);
  while (len > 0 && (command[len - 1] == ' ' || command[len - 1] == '\t')) {
    command[len - 1] = '\0';
    len--;
  }

  return command;
}

void processCommand(char *input) {
  char *command = trimCommand(input);

  if (strcmp(command, "HELP") == 0) {
    printHelp();
  }
  else if (strcmp(command, "STATUS") == 0) {
    printStatus();
  }
  else if (strcmp(command, "RATE") == 0) {
    printHx711Rate();
  }
  else if (strcmp(command, "RAW") == 0) {
    printRawOnce();
  }
  else if (strcmp(command, "MONITOR") == 0) {
    monitorPedal();
  }
  else if (strcmp(command, "TARE") == 0) {
    tarePedal();
  }
  else if (strcmp(command, "MAX") == 0) {
    calibrateMax();
  }
  else if (strcmp(command, "INVERT") == 0) {
    config.invertSignal = !config.invertSignal;
    saveConfig();

    Serial.print("Invert Signal ahora es: ");
    Serial.println(config.invertSignal ? "true" : "false");
  }
  else if (strncmp(command, "CURVE ", 6) == 0) {
    float value = atof(command + 6);

    if (value >= 0.5 && value <= 3.0) {
      config.curveExponent = value;
      saveConfig();

      Serial.print("Curva guardada: ");
      Serial.println(config.curveExponent, 3);
    } else {
      Serial.println("Valor invalido. Usa CURVE 0.5 a CURVE 3.0");
    }
  }
  else if (strncmp(command, "DEADZONE ", 9) == 0) {
    long value = atol(command + 9);

    if (value >= 0 && value <= 100000 && value < config.maxBrakeRaw) {
      config.deadzoneRaw = (int)value;
      saveConfig();

      Serial.print("Deadzone guardada: ");
      Serial.println(config.deadzoneRaw);
    } else {
      Serial.println("Valor invalido. Usa DEADZONE 0 a 100000 y menor que SETMAX.");
    }
  }
  else if (strncmp(command, "SETMAX ", 7) == 0) {
    long value = atol(command + 7);

    if (value > 1000 && value > config.deadzoneRaw) {
      config.maxBrakeRaw = value;
      saveConfig();

      Serial.print("Max manual guardado: ");
      Serial.println(config.maxBrakeRaw);
    } else {
      Serial.println("Valor invalido. SETMAX debe ser mayor que 1000 y mayor que DEADZONE.");
    }
  }
  else if (strcmp(command, "RESET") == 0) {
    resetConfig();
  }
  else if (command[0] != '\0') {
    Serial.println("Comando no reconocido. Usa HELP.");
  }
}

void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength >= sizeof(commandBuffer) - 1) {
      commandLength = 0;
      Serial.println("Comando demasiado largo. Usa HELP.");
      continue;
    }

    if (c >= 'a' && c <= 'z') {
      c -= 32;
    }

    commandBuffer[commandLength++] = c;
  }
}

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(115200);
  delay(500);

  loadConfig();

#if HX711_RATE_PIN >= 0
  pinMode(HX711_RATE_PIN, OUTPUT);
  digitalWrite(HX711_RATE_PIN, HIGH);
#endif

  scale.begin(DOUT, CLK);

  Joystick.begin(false);
#if REPORT_X_AXIS
  Joystick.setXAxisRange(0, 1023);
#endif
#if REPORT_Z_AXIS
  Joystick.setZAxisRange(0, 1023);
#endif
#if REPORT_RUDDER_AXIS
  Joystick.setRudderRange(0, 1023);
#endif
#if REPORT_BRAKE_AXIS
  Joystick.setBrakeRange(0, 1023);
#endif
  sendBrakeJoystickValue(0);
  Joystick.sendState();

  Serial.println("Pedal HX711 iniciado - simracing rapido.");
  Serial.println("Objetivo HX711: 80 SPS (12.5 ms por muestra).");
  Serial.println("Comandos: HELP, STATUS, RATE, RAW, MONITOR, TARE, MAX, INVERT, RESET");

  if (scale.is_ready()) {
    readRawNonBlocking();
    Serial.println("HX711 listo.");
  } else {
    Serial.println("HX711 no listo al inicio, se leera sin bloquear en loop.");
  }
}

// =======================
// LOOP
// =======================
void loop() {
  serviceSerial();
  readRawNonBlocking();

  if (hasLatestRaw && millis() - lastSampleMs > HX711_STALE_MS) {
    targetBrake = 0;
  }

  static unsigned long lastReport = 0;

  if (millis() - lastReport >= JOYSTICK_INTERVAL_MS) {
    lastReport = millis();

    if (abs(targetBrake - (int)smoothBrake) <= JOYSTICK_NOISE_BAND) {
      smoothBrake = targetBrake;
    } else if (targetBrake > smoothBrake) {
      smoothBrake = smoothBrake + ALPHA_PRESS * (targetBrake - smoothBrake);
    } else {
      smoothBrake = smoothBrake + ALPHA_RELEASE * (targetBrake - smoothBrake);
    }

    if (smoothBrake < 0) smoothBrake = 0;
    if (smoothBrake > 1023) smoothBrake = 1023;

    sendBrakeJoystickValue((int)(smoothBrake + 0.5));
    Joystick.sendState();
  }
}
