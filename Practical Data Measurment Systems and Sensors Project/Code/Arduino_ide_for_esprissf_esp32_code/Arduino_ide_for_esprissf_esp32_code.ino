// Full Sensor Suite — ESP32 DevKit V1 (38-pin)
//
// Pressure:   Top(34), MiddleRight(35), MiddleLeft(32), Bottom(33)
// NTC:        Top(25), MiddleRight(26), MiddleLeft(27), Bottom(14)
// Coins:      Coin1(13), Coin2(15), Coin3(23)
// TEC:        GPIO19
// DHT22:      GPIO4
// Buzzer:     GPIO5
// MAX30102:   SDA(21), SCL(22)
// MPU6050:    SDA(21), SCL(22) — same I2C bus

#include <math.h>
#include <DHT.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050.h>

// ── Pressure & NTC pins ───────────────────────────────────────────────────────
const int NUM_SENSORS = 4;
const int NUM_SAMPLES = 20;
const int BAUD_RATE   = 115200;

const int PRESSURE_PINS[NUM_SENSORS]  = {34, 35, 32, 33};
const int NTC_PINS[NUM_SENSORS]       = {25, 26, 27, 14};
const char* ZONE_NAMES[NUM_SENSORS]   = {"Top", "MiddleRight", "MiddleLeft", "Bottom"};

// ── Output pins ───────────────────────────────────────────────────────────────
const int MOTOR_TOP   = 13;
const int MOTOR_MID   = 15;
const int MOTOR_BOT   = 23;
const int TEC_PIN     = 19;
const int BUZZER_PIN  = 5;
const int DHT_PIN     = 4;

// ── DHT22 ─────────────────────────────────────────────────────────────────────
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// ── MAX30102 ──────────────────────────────────────────────────────────────────
MAX30105 particleSensor;
const byte  HR_RATE_SIZE = 4;
byte        hrRates[HR_RATE_SIZE];
byte        hrRateSpot  = 0;
long        lastBeatTime = 0;
float       beatsPerMinute = 0;
float       beatAvg        = 0;

// HR normal range for adults
const float HR_MIN = 60.0;
const float HR_MAX = 100.0;

// ── MPU6050 ───────────────────────────────────────────────────────────────────
MPU6050 mpu;

// Walking detection — based on accelerometer magnitude variance
const float WALK_THRESHOLD  = 1.2;   // g — tune if needed
const int   WALK_WINDOW     = 20;    // samples to analyse
float       accelBuffer[WALK_WINDOW];
int         accelIdx        = 0;
bool        isWalking       = false;

// ── Pressure thresholds ───────────────────────────────────────────────────────
const int NO_PRESS_MAX  = 400;
const int LIGHT_PRESS   = 1200;
const int MEDIUM_PRESS  = 2400;
const int HEAVY_PRESS   = 3200;
const unsigned long TRIGGER_DELAY = 10000;

// ── NTC config ────────────────────────────────────────────────────────────────
const float SERIES_RESISTOR    = 10000.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMP       = 25.0;
const float BETA_COEFFICIENT   = 3950.0;
const float ADC_MAX            = 4095.0;

// ── Thresholds ────────────────────────────────────────────────────────────────
const float TEMP_THRESHOLD     = 35.0;
const float HUMIDITY_THRESHOLD = 65.0;

// ── State ─────────────────────────────────────────────────────────────────────
unsigned long heavySince[NUM_SENSORS];
bool          isHeavy[NUM_SENSORS];
unsigned long lastDHTRead  = 0;
const unsigned long DHT_INTERVAL = 2000;
float currentHumidity = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin(21, 22);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Output pins
  pinMode(MOTOR_TOP,  OUTPUT);  digitalWrite(MOTOR_TOP,  LOW);
  pinMode(MOTOR_MID,  OUTPUT);  digitalWrite(MOTOR_MID,  LOW);
  pinMode(MOTOR_BOT,  OUTPUT);  digitalWrite(MOTOR_BOT,  LOW);
  pinMode(TEC_PIN,    OUTPUT);  digitalWrite(TEC_PIN,    LOW);
  pinMode(BUZZER_PIN, OUTPUT);  digitalWrite(BUZZER_PIN, LOW);

  // Pressure state
  for (int i = 0; i < NUM_SENSORS; i++) {
    heavySince[i] = 0;
    isHeavy[i]    = false;
  }

  // DHT22
  dht.begin();

  // MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found — check wiring!");
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 ready");
  }

  // MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not found — check wiring!");
  } else {
    Serial.println("MPU6050 ready");
  }

  // Accel buffer
  for (int i = 0; i < WALK_WINDOW; i++) accelBuffer[i] = 0;

  Serial.println("ESP32 — Full Sensor Suite Ready");
  Serial.println("=================================");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  int   rawPressure[NUM_SENSORS];
  float temperature[NUM_SENSORS];
  bool  tecShouldRun = false;

  // ── DHT22 humidity ────────────────────────────────────────────────────────
  if (now - lastDHTRead >= DHT_INTERVAL) {
    float h = dht.readHumidity();
    if (!isnan(h)) currentHumidity = h;
    lastDHTRead = now;
  }

  // ── MAX30102 heart rate ───────────────────────────────────────────────────
  long irValue = particleSensor.getIR();
  bool fingerDetected = (irValue > 50000);

  if (fingerDetected && checkForBeat(irValue)) {
    long delta        = now - lastBeatTime;
    lastBeatTime      = now;
    beatsPerMinute    = 60.0 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      hrRates[hrRateSpot++ % HR_RATE_SIZE] = (byte)beatsPerMinute;
      float sum = 0;
      for (byte x = 0; x < HR_RATE_SIZE; x++) sum += hrRates[x];
      beatAvg = sum / HR_RATE_SIZE;
    }
  }

  bool hrAlert = fingerDetected && (beatAvg > 0) &&
                 (beatAvg < HR_MIN || beatAvg > HR_MAX);

  // ── MPU6050 walking detection ─────────────────────────────────────────────
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw to g (±2g range default, 16384 LSB/g)
  float axG = ax / 16384.0;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;
  float magnitude = sqrt(axG * axG + ayG * ayG + azG * azG);

  accelBuffer[accelIdx % WALK_WINDOW] = magnitude;
  accelIdx++;

  // Calculate variance over window
  if (accelIdx >= WALK_WINDOW) {
    float mean = 0;
    for (int i = 0; i < WALK_WINDOW; i++) mean += accelBuffer[i];
    mean /= WALK_WINDOW;
    float variance = 0;
    for (int i = 0; i < WALK_WINDOW; i++)
      variance += (accelBuffer[i] - mean) * (accelBuffer[i] - mean);
    variance /= WALK_WINDOW;
    isWalking = (variance > 0.01);  // tune this value if needed
  }

  // ── Pressure + NTC ────────────────────────────────────────────────────────
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawPressure[i] = averageReading(PRESSURE_PINS[i], NUM_SAMPLES);
    bool currentlyHeavy = (rawPressure[i] >= HEAVY_PRESS);
    if (currentlyHeavy && !isHeavy[i]) {
      heavySince[i] = now;
      isHeavy[i]    = true;
    } else if (!currentlyHeavy) {
      isHeavy[i]    = false;
      heavySince[i] = 0;
    }
    temperature[i] = readNTC(NTC_PINS[i]);
    if (temperature[i] > TEMP_THRESHOLD) tecShouldRun = true;
  }

  // ── Motor logic ───────────────────────────────────────────────────────────
  bool coin1On       = isHeavy[0] && (now - heavySince[0] >= TRIGGER_DELAY);
  bool midRightReady = isHeavy[1] && (now - heavySince[1] >= TRIGGER_DELAY);
  bool midLeftReady  = isHeavy[2] && (now - heavySince[2] >= TRIGGER_DELAY);
  bool coin2On       = midRightReady || midLeftReady;
  bool coin3On       = isHeavy[3] && (now - heavySince[3] >= TRIGGER_DELAY);

  digitalWrite(MOTOR_TOP, coin1On      ? HIGH : LOW);
  digitalWrite(MOTOR_MID, coin2On      ? HIGH : LOW);
  digitalWrite(MOTOR_BOT, coin3On      ? HIGH : LOW);
  digitalWrite(TEC_PIN,   tecShouldRun ? HIGH : LOW);

  // ── Buzzer logic (humidity OR HR alert) ───────────────────────────────────
  bool humidityAlert = (currentHumidity > HUMIDITY_THRESHOLD);
  bool buzzerOn      = humidityAlert || hrAlert;
  digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);

  // ── Serial output ─────────────────────────────────────────────────────────
  Serial.println("---- Pressure & Temperature ----");
  for (int i = 0; i < NUM_SENSORS; i++) {
    String force       = classifyForce(rawPressure[i]);
    unsigned long held = isHeavy[i] ? (now - heavySince[i]) / 1000 : 0;
    Serial.print("[");
    Serial.print(ZONE_NAMES[i]);
    Serial.print("] Pressure: ");
    Serial.print(force);
    Serial.print(" (");
    Serial.print(rawPressure[i]);
    Serial.print(") | Temp: ");
    Serial.print(temperature[i], 1);
    Serial.print("°C");
    if (temperature[i] > TEMP_THRESHOLD) Serial.print(" ⚠ HOT");
    if (isHeavy[i]) {
      Serial.print(" | Heavy: ");
      Serial.print(held);
      Serial.print("s");
    }
    Serial.println();
  }

  Serial.println("---- Environment ----");
  Serial.print("Humidity: ");
  Serial.print(currentHumidity, 1);
  Serial.print("%");
  if (humidityAlert) Serial.print(" ⚠ HIGH HUMIDITY");
  Serial.println();

  Serial.println("---- Heart Rate ----");
  if (!fingerDetected) {
    Serial.println("HR: No finger detected");
  } else {
    Serial.print("HR: ");
    Serial.print(beatsPerMinute, 0);
    Serial.print(" bpm | Avg: ");
    Serial.print(beatAvg, 0);
    Serial.print(" bpm");
    if (hrAlert) {
      if (beatAvg < HR_MIN) Serial.print(" ⚠ TOO LOW");
      if (beatAvg > HR_MAX) Serial.print(" ⚠ TOO HIGH");
    }
    Serial.println();
  }

  Serial.println("---- Movement ----");
  Serial.print("Walking: ");
  Serial.print(isWalking ? "YES 🚶" : "NO — stationary");
  Serial.print(" | Accel magnitude: ");
  Serial.print(magnitude, 3);
  Serial.println(" g");

  Serial.println("---- Outputs ----");
  Serial.print("C1:");  Serial.print(coin1On      ? "ON " : "off ");
  Serial.print("C2:");  Serial.print(coin2On      ? "ON " : "off ");
  Serial.print("C3:");  Serial.print(coin3On      ? "ON " : "off ");
  Serial.print("TEC:"); Serial.print(tecShouldRun ? "ON ❄ " : "off ");
  Serial.print("Buzzer:"); Serial.println(buzzerOn ? "ON 🔔" : "off");
  Serial.println("=================================");

  delay(200);
}

// ── NTC ───────────────────────────────────────────────────────────────────────
float readNTC(int pin) {
  int raw = averageReading(pin, NUM_SAMPLES);
  if (raw <= 0 || raw >= 4095) return -999.0;
  float resistance