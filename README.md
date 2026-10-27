# Mineresq
Underground mine safety node with surface alert station. 
// MineResQ underground node (ESP32): gas, dust, fire, flood, collapse, SOS
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <DHT.h>

#define NODE_ID 1
const int PIN_GAS = 34, PIN_DUST = 35, PIN_WATER = 32, PIN_FLAME = 33;
const int PIN_DHT = 4, PIN_BUZZ = 25, PIN_LED = 26, PIN_SOS = 27;

// ---- Thresholds (placeholders, calibrate on real sensors) ----
const unsigned long WARMUP_MS = 20000;   // MQ-4 warm-up + clean-air baseline
const int GAS_WARN = 300, GAS_DANGER = 700;      // rise above clean-air baseline
const int DUST_WARN = 1500, DUST_DANGER = 2500;
const int WATER_WARN = 1200, WATER_DANGER = 2200;
const float TEMP_WARN = 50.0f, TEMP_DANGER = 60.0f;
const float SHAKE_G = 0.5f;              // sudden acceleration change = possible collapse
const float TILT_WARN = 5.0f, TILT_DANGER = 10.0f;  // degrees from baseline

enum Flag { F_GAS = 1, F_DUST = 2, F_FIRE = 4, F_FLOOD = 8, F_COLLAPSE = 16, F_HEAT = 32, F_SOS = 64 };
enum Level { SAFE = 0, WARNING = 1, DANGER = 2 };

typedef struct __attribute__((packed)) {
  uint8_t node, level;
  uint16_t flags;
  float temp, shake;
  int16_t gas, dust, water;
} Packet;

DHT dht(PIN_DHT, DHT22);
uint8_t bcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
float gasBase = 0, gasS = 0, dustS = 0, waterS = 0, temp = 25, baseTilt = 0, lastG = 1, shake = 0;
unsigned long tStart, tSample = 0, tDht = 0, tSend = 0, collapseUntil = 0;
uint8_t lastLevel = 0;
bool baselined = false;

int16_t rd() { int16_t h = Wire.read(); int16_t l = Wire.read(); return (h << 8) | l; }

void readImu(float &g, float &tilt) {
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);
  float x = rd() / 4096.0f, y = rd() / 4096.0f, z = rd() / 4096.0f;   // +-8g
  g = sqrtf(x * x + y * y + z * z);
  tilt = acosf(constrain(z / max(g, 0.01f), -1.0f, 1.0f)) * 57.2958f;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZ, OUTPUT); pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SOS, INPUT_PULLUP); pinMode(PIN_FLAME, INPUT);
  analogReadResolution(12);
  dht.begin();
  Wire.begin();
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) Serial.println("ESP-NOW failed");
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, bcast, 6);
  esp_now_add_peer(&peer);
  gasS = analogRead(PIN_GAS); dustS = analogRead(PIN_DUST); waterS = analogRead(PIN_WATER);
  tStart = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - tSample >= 50) {
    tSample = now;
    gasS += 0.05f * (analogRead(PIN_GAS) - gasS);
    dustS += 0.05f * (analogRead(PIN_DUST) - dustS);
    waterS += 0.05f * (analogRead(PIN_WATER) - waterS);
    float g, tilt; readImu(g, tilt);
    shake = fabsf(g - lastG); lastG = g;
    if (now - tStart < WARMUP_MS) { gasBase = gasS; baseTilt = tilt; baselined = true; }
    else if (shake > SHAKE_G) collapseUntil = now + 30000;           // latch for 30 s
    static float tiltNow; tiltNow = fabsf(tilt - baseTilt);
    // keep tilt result for level logic below
    lastG = g;
    extern float gTilt; gTilt = tiltNow;
  }
  if (now - tDht >= 2000) {
    tDht = now;
    float t = dht.readTemperature();
    if (!isnan(t)) temp = t;
  }
  if (!baselined || now - tStart < WARMUP_MS) return;   // no alarms while warming up

  // ---- Evaluate ----
  extern float gTilt;
  uint16_t flags = 0; uint8_t level = SAFE;
  float gasRise = gasS - gasBase;

  if (gasRise > GAS_WARN) { flags |= F_GAS; level = max<int>(level, gasRise > GAS_DANGER ? DANGER : WARNING); }
  if (dustS > DUST_WARN) { flags |= F_DUST; level = max<int>(level, dustS > DUST_DANGER ? DANGER : WARNING); }
  if (waterS > WATER_WARN) { flags |= F_FLOOD; level = max<int>(level, waterS > WATER_DANGER ? DANGER : WARNING); }
  if (temp > TEMP_WARN) { flags |= F_HEAT; level = max<int>(level, temp > TEMP_DANGER ? DANGER : WARNING); }
  if (digitalRead(PIN_FLAME) == LOW) { flags |= F_FIRE; level = DANGER; }          // most flame modules go LOW on flame
  if (now < collapseUntil || gTilt > TILT_DANGER) { flags |= F_COLLAPSE; level = DANGER; }
  else if (gTilt > TILT_WARN) { flags |= F_COLLAPSE; level = max<int>(level, WARNING); }
  if (digitalRead(PIN_SOS) == LOW) { flags |= F_SOS; level = DANGER; }

  // ---- Local alarm ----
  if (level == DANGER) { bool on = (now % 300) < 150; digitalWrite(PIN_BUZZ, on); digitalWrite(PIN_LED, on); }
  else if (level == WARNING) { bool on = (now % 1500) < 200; digitalWrite(PIN_BUZZ, on); digitalWrite(PIN_LED, HIGH); }
  else { digitalWrite(PIN_BUZZ, LOW); digitalWrite(PIN_LED, LOW); }

  // ---- Report to surface: every 1 s, immediately on level change ----
  if (level != lastLevel || now - tSend >= 1000) {
    tSend = now; lastLevel = level;
    Packet p = {NODE_ID, level, flags, temp, shake, (int16_t)gasRise, (int16_t)dustS, (int16_t)waterS};
    esp_now_send(bcast, (uint8_t*)&p, sizeof(p));
    Serial.printf("L%d flags=0x%02X T=%.1f gas+%d dust=%d water=%d tilt=%.1f\n",
                  level, flags, temp, (int)gasRise, (int)dustS, (int)waterS, gTilt);
  }
}
float gTilt = 0;
