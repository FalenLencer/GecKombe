#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// ─── WiFi ────────────────────────────────────────────────────
const char* SSID     = "A51 de Cle";
const char* PASSWORD = "sgru3285";

// ─── MQ-9 ────────────────────────────────────────────────────
#define MQ9_ANALOG_PIN  34
#define MQ9_DIGITAL_PIN 32
#define RL_VALUE        10.0
#define RO              10.0      // à calibrer dans l'air pur
#define ADC_MAX         4095.0
#define VCC             3.3

// ─── PCA9685 — mapping servos ────────────────────────────────
#define sda 21
#define scl 22
// Postérieur droit  : ch 0 (LED01) + ch 1 (LED02)
// Postérieur gauche : ch 2 (LED03) + ch 3 (LED04)
// Antérieur droit   : ch 4 (LED05) + ch 5 (LED06)
// Antérieur gauche  : ch 6 (LED07) + ch 7 (LED08)
#define SERVO_BR_HIP    0
#define SERVO_BR_KNEE   1
#define SERVO_BL_HIP    2
#define SERVO_BL_KNEE   3
#define SERVO_FR_HIP    4
#define SERVO_FR_KNEE   5
#define SERVO_FL_HIP    6
#define SERVO_FL_KNEE   7

// Limites servo en ticks PCA9685 (50 Hz → 4096 ticks)
// Ajuste SERVO_MIN / SERVO_MAX selon tes servos
#define SERVO_MIN   150    // ~0°
#define SERVO_MID   375    // ~90°
#define SERVO_MAX   600    // ~180°
#define SERVO_FREQ  50

// ─── Positions angulaires de base (en ticks) ─────────────────
struct LegPos { uint16_t hip; uint16_t knee; };

// Posture neutre debout
const LegPos STAND = { SERVO_MID, SERVO_MID };

// ─── Séquences de marche ────────────────────────────────────
// Foulée diagonale (trot) :
//   Phase 1 : FR + BL avancent   /  FL + BR en appui
//   Phase 2 : FL + BR avancent   /  FR + BL en appui
//
// Valeurs grossières — à affiner selon ta mécanique
struct GaitFrame {
  LegPos br, bl, fr, fl;  // postérieur droit, pg, antérieur droit, ag
};

// Forward trot — 4 frames
const GaitFrame GAIT_FORWARD[4] = {
  // Phase lever FR+BL
  { {SERVO_MID,      SERVO_MID},      // BR appui
    {SERVO_MID+50,   SERVO_MID-60},   // BL lever
    {SERVO_MID-50,   SERVO_MID-60},   // FR lever
    {SERVO_MID,      SERVO_MID} },    // FL appui
  // Phase poser FR+BL
  { {SERVO_MID,      SERVO_MID},
    {SERVO_MID-30,   SERVO_MID},
    {SERVO_MID+30,   SERVO_MID},
    {SERVO_MID,      SERVO_MID} },
  // Phase lever FL+BR
  { {SERVO_MID+50,   SERVO_MID-60},   // BR lever
    {SERVO_MID,      SERVO_MID},      // BL appui
    {SERVO_MID,      SERVO_MID},      // FR appui
    {SERVO_MID-50,   SERVO_MID-60} }, // FL lever
  // Phase poser FL+BR
  { {SERVO_MID-30,   SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID+30,   SERVO_MID} }
};

// Backward — inverse hips
const GaitFrame GAIT_BACKWARD[4] = {
  { {SERVO_MID,      SERVO_MID},
    {SERVO_MID-50,   SERVO_MID-60},
    {SERVO_MID+50,   SERVO_MID-60},
    {SERVO_MID,      SERVO_MID} },
  { {SERVO_MID,      SERVO_MID},
    {SERVO_MID+30,   SERVO_MID},
    {SERVO_MID-30,   SERVO_MID},
    {SERVO_MID,      SERVO_MID} },
  { {SERVO_MID-50,   SERVO_MID-60},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID+50,   SERVO_MID-60} },
  { {SERVO_MID+30,   SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID-30,   SERVO_MID} }
};

// Left — FR+FL avancent, hip vers gauche
const GaitFrame GAIT_LEFT[2] = {
  { {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID-40,   SERVO_MID-50},
    {SERVO_MID-40,   SERVO_MID-50} },
  { {SERVO_MID+30,   SERVO_MID},
    {SERVO_MID+30,   SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID} }
};

// Right — symétrique
const GaitFrame GAIT_RIGHT[2] = {
  { {SERVO_MID+40,   SERVO_MID-50},
    {SERVO_MID+40,   SERVO_MID-50},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID} },
  { {SERVO_MID,      SERVO_MID},
    {SERVO_MID,      SERVO_MID},
    {SERVO_MID-30,   SERVO_MID},
    {SERVO_MID-30,   SERVO_MID} }
};

// ─── Objets globaux ──────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
WebServer               httpServer(80);
WebSocketsServer        wsServer(81);

// ─── État ────────────────────────────────────────────────────
enum Command { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };
volatile Command currentCmd = CMD_STOP;
volatile uint8_t speed      = 50;      // 0-100
uint8_t  gaitFrame          = 0;
uint32_t lastGaitStep       = 0;
uint32_t lastSensorSend     = 0;

// ─── Servo helpers ───────────────────────────────────────────
uint16_t mapSpeed(uint16_t base, uint16_t mid, uint8_t spd) {
  // Applique la vitesse en interpolant entre mid et base
  int delta = (int)base - (int)mid;
  return (uint16_t)(mid + delta * spd / 100);
}

void setServo(uint8_t ch, uint16_t ticks) {
  ticks = constrain(ticks, SERVO_MIN, SERVO_MAX);
  pca.setPWM(ch, 0, ticks);
}

void applyFrame(const GaitFrame& f) {
  setServo(SERVO_BR_HIP,  f.br.hip);
  setServo(SERVO_BR_KNEE, f.br.knee);
  setServo(SERVO_BL_HIP,  f.bl.hip);
  setServo(SERVO_BL_KNEE, f.bl.knee);
  setServo(SERVO_FR_HIP,  f.fr.hip);
  setServo(SERVO_FR_KNEE, f.fr.knee);
  setServo(SERVO_FL_HIP,  f.fl.hip);
  setServo(SERVO_FL_KNEE, f.fl.knee);
}

void standStill() {
  applyFrame({ STAND, STAND, STAND, STAND });
  gaitFrame = 0;
}

// ─── Gait loop ───────────────────────────────────────────────
// Délai entre frames : map vitesse 0-100 → 400-80 ms
uint32_t gaitDelay() {
  return (uint32_t)map(speed, 0, 100, 400, 80);
}

void tickGait() {
  if (millis() - lastGaitStep < gaitDelay()) return;
  lastGaitStep = millis();

  switch (currentCmd) {
    case CMD_FORWARD:
      applyFrame(GAIT_FORWARD[gaitFrame % 4]);
      gaitFrame++;
      break;
    case CMD_BACKWARD:
      applyFrame(GAIT_BACKWARD[gaitFrame % 4]);
      gaitFrame++;
      break;
    case CMD_LEFT:
      applyFrame(GAIT_LEFT[gaitFrame % 2]);
      gaitFrame++;
      break;
    case CMD_RIGHT:
      applyFrame(GAIT_RIGHT[gaitFrame % 2]);
      gaitFrame++;
      break;
    case CMD_STOP:
    default:
      standStill();
      break;
  }
}

// ─── MQ-9 ────────────────────────────────────────────────────
struct SensorData { int raw; float voltage, rs, ratio, ppm; bool alert; };

SensorData readMQ9() {
  SensorData d;
  d.raw     = analogRead(MQ9_ANALOG_PIN);
  d.alert   = digitalRead(MQ9_DIGITAL_PIN) == LOW;
  d.voltage = (d.raw / ADC_MAX) * VCC;
  d.rs      = (d.voltage > 0.01f) ? RL_VALUE * (VCC - d.voltage) / d.voltage : 0;
  d.ratio   = d.rs / RO;
  d.ppm     = (d.ratio > 0) ? 599.65f * pow(d.ratio, -2.244f) : 0;
  return d;
}

// ─── WebSocket handler ───────────────────────────────────────
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload, length)) return;

    const char* t = doc["type"];
    if (!t) return;

    if (strcmp(t, "ping") == 0) {
      wsServer.sendTXT(num, "{\"type\":\"pong\"}");
      return;
    }

    if (strcmp(t, "cmd") == 0) {
      const char* cmd = doc["cmd"];
      speed = constrain((int)doc["speed"] | 50, 0, 100);
      if      (strcmp(cmd, "forward")  == 0) currentCmd = CMD_FORWARD;
      else if (strcmp(cmd, "backward") == 0) currentCmd = CMD_BACKWARD;
      else if (strcmp(cmd, "left")     == 0) currentCmd = CMD_LEFT;
      else if (strcmp(cmd, "right")    == 0) currentCmd = CMD_RIGHT;
      else                                    currentCmd = CMD_STOP;
      gaitFrame = 0;
    }
  }

  if (type == WStype_DISCONNECTED) {
    currentCmd = CMD_STOP;
    standStill();
  }
}

// ─── HTTP page de redirection ────────────────────────────────
void handleRoot() {
  httpServer.send(200, "text/plain",
    "ESP32 QuadBot OK — WebSocket sur le port 81\n"
    "Donnees capteur sur ws://IP:81/ws\n"
    "Commandes: {\"type\":\"cmd\",\"cmd\":\"forward|backward|left|right|stop\",\"speed\":0-100}");
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(MQ9_ANALOG_PIN,  INPUT);
  pinMode(MQ9_DIGITAL_PIN, INPUT);

  Wire.begin();
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);
  standStill();
  Serial.println("PCA9685 initialisé");

  Serial.printf("Connexion WiFi à %s", SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnecté — IP : http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("WebSocket  : ws://%s:81/ws\n", WiFi.localIP().toString().c_str());

  httpServer.on("/", handleRoot);
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  Serial.println("Serveurs démarrés");
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  httpServer.handleClient();
  wsServer.loop();

  // Gait
  tickGait();

  // Envoi capteur toutes les 2 s à tous les clients connectés
  if (millis() - lastSensorSend > 2000) {
    lastSensorSend = millis();
    SensorData d = readMQ9();

    StaticJsonDocument<160> doc;
    doc["type"]    = "sensor";
    doc["raw"]     = d.raw;
    doc["voltage"] = serialized(String(d.voltage, 3));
    doc["rs"]      = serialized(String(d.rs,      3));
    doc["ratio"]   = serialized(String(d.ratio,   3));
    doc["ppm"]     = serialized(String(d.ppm,     1));
    doc["alert"]   = d.alert;

    String out;
    serializeJson(doc, out);
    wsServer.broadcastTXT(out);

    Serial.printf("[MQ-9] %.1f ppm | alert=%s | CMD=%d speed=%d\n",
      d.ppm, d.alert ? "OUI" : "non", (int)currentCmd, speed);
  }
}
