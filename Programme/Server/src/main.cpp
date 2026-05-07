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
#define RO              10.0
#define ADC_MAX         4095.0
#define VCC             3.3

// ─── PCA9685 — mapping servos ────────────────────────────────
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

#define SERVO_FREQ  50

// ─── Limites mécaniques ──────────────────────────────────────
// 1° = 2.5 ticks   |   centre = 90° = 375 ticks
//
// Hanche : ±90°  → 0°–180°   → ticks 150–600
#define HIP_MIN   150
#define HIP_MID   375
#define HIP_MAX   600
//
// Genou  : ±45°  → 45°–135°  → ticks 263–487
#define KNEE_MIN  263
#define KNEE_MID  375
#define KNEE_MAX  487

// ─── Timing marche ───────────────────────────────────────────
#define STEP_DELAY  300   // ms entre chaque étape de marche

// ─── Objets globaux ──────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
WebServer               httpServer(80);
WebSocketsServer        wsServer(81);

// ─── État ────────────────────────────────────────────────────
enum Command { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };
volatile Command currentCmd = CMD_STOP;
volatile uint8_t speed      = 50;   // 0-100
uint8_t          gaitStep   = 0;
unsigned long    lastStep   = 0;
uint32_t         lastSensorSend = 0;

// ─── Servo helper ────────────────────────────────────────────
// Envoie une position en bornant selon le type d'articulation
void setHip(uint8_t ch, uint16_t ticks) {
  pca.setPWM(ch, 0, constrain(ticks, HIP_MIN, HIP_MAX));
}

void setKnee(uint8_t ch, uint16_t ticks) {
  pca.setPWM(ch, 0, constrain(ticks, KNEE_MIN, KNEE_MAX));
}

// ─── Posture neutre ──────────────────────────────────────────
void standStill() {
  setHip  (SERVO_BR_HIP,  HIP_MID);
  setKnee (SERVO_BR_KNEE, KNEE_MID);
  setHip  (SERVO_BL_HIP,  HIP_MID);
  setKnee (SERVO_BL_KNEE, KNEE_MID);
  setHip  (SERVO_FR_HIP,  HIP_MID);
  setKnee (SERVO_FR_KNEE, KNEE_MID);
  setHip  (SERVO_FL_HIP,  HIP_MID);
  setKnee (SERVO_FL_KNEE, KNEE_MID);
  gaitStep = 0;
}

// ─── Délai adapté à la vitesse ────────────────────────────────
// speed=0  → 500 ms   |   speed=100 → 150 ms
unsigned long stepDelay() {
  return map(speed, 0, 100, 500, 150);
}

// ─── Marche AVANT ────────────────────────────────────────────
// Trot diagonal : FR+BL puis FL+BR, hanches + genoux
// Étape 0 : lever FR+BL (genou monte, hanche avance)
// Étape 1 : poser FR+BL  (genou descend)
// Étape 2 : lever FL+BR
// Étape 3 : poser FL+BR
void walkForward() {
  if (millis() - lastStep < stepDelay()) return;
  lastStep = millis();

  switch (gaitStep) {
    case 0:
      setKnee(SERVO_FR_KNEE, KNEE_MIN);   // FR genou lève
      setHip (SERVO_FR_HIP,  HIP_MID - 50);
      setKnee(SERVO_BL_KNEE, KNEE_MIN);   // BL genou lève
      setHip (SERVO_BL_HIP,  HIP_MID + 50);
      break;
    case 1:
      setKnee(SERVO_FR_KNEE, KNEE_MID);   // FR genou pose
      setHip (SERVO_FR_HIP,  HIP_MID + 30);
      setKnee(SERVO_BL_KNEE, KNEE_MID);   // BL genou pose
      setHip (SERVO_BL_HIP,  HIP_MID - 30);
      break;
    case 2:
      setKnee(SERVO_FL_KNEE, KNEE_MIN);   // FL genou lève
      setHip (SERVO_FL_HIP,  HIP_MID - 50);
      setKnee(SERVO_BR_KNEE, KNEE_MIN);   // BR genou lève
      setHip (SERVO_BR_HIP,  HIP_MID + 50);
      break;
    case 3:
      setKnee(SERVO_FL_KNEE, KNEE_MID);   // FL genou pose
      setHip (SERVO_FL_HIP,  HIP_MID + 30);
      setKnee(SERVO_BR_KNEE, KNEE_MID);   // BR genou pose
      setHip (SERVO_BR_HIP,  HIP_MID - 30);
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

// ─── Marche ARRIÈRE ──────────────────────────────────────────
// Identique à l'avant mais hanches inversées
void walkBackward() {
  if (millis() - lastStep < stepDelay()) return;
  lastStep = millis();

  switch (gaitStep) {
    case 0:
      setKnee(SERVO_FR_KNEE, KNEE_MIN);
      setHip (SERVO_FR_HIP,  HIP_MID + 50);   // hanche inversée
      setKnee(SERVO_BL_KNEE, KNEE_MIN);
      setHip (SERVO_BL_HIP,  HIP_MID - 50);
      break;
    case 1:
      setKnee(SERVO_FR_KNEE, KNEE_MID);
      setHip (SERVO_FR_HIP,  HIP_MID - 30);
      setKnee(SERVO_BL_KNEE, KNEE_MID);
      setHip (SERVO_BL_HIP,  HIP_MID + 30);
      break;
    case 2:
      setKnee(SERVO_FL_KNEE, KNEE_MIN);
      setHip (SERVO_FL_HIP,  HIP_MID + 50);
      setKnee(SERVO_BR_KNEE, KNEE_MIN);
      setHip (SERVO_BR_HIP,  HIP_MID - 50);
      break;
    case 3:
      setKnee(SERVO_FL_KNEE, KNEE_MID);
      setHip (SERVO_FL_HIP,  HIP_MID - 30);
      setKnee(SERVO_BR_KNEE, KNEE_MID);
      setHip (SERVO_BR_HIP,  HIP_MID + 30);
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

// ─── Virage GAUCHE ───────────────────────────────────────────
// Membres gauches reculent, membres droits avancent
void walkLeft() {
  if (millis() - lastStep < stepDelay()) return;
  lastStep = millis();

  switch (gaitStep) {
    case 0:
      setKnee(SERVO_FL_KNEE, KNEE_MIN);
      setHip (SERVO_FL_HIP,  HIP_MID - 40);
      setKnee(SERVO_BL_KNEE, KNEE_MIN);
      setHip (SERVO_BL_HIP,  HIP_MID - 40);
      break;
    case 1:
      setKnee(SERVO_FL_KNEE, KNEE_MID);
      setHip (SERVO_FL_HIP,  HIP_MID + 30);
      setKnee(SERVO_BL_KNEE, KNEE_MID);
      setHip (SERVO_BL_HIP,  HIP_MID + 30);
      break;
    case 2:
      setKnee(SERVO_FR_KNEE, KNEE_MIN);
      setHip (SERVO_FR_HIP,  HIP_MID + 40);
      setKnee(SERVO_BR_KNEE, KNEE_MIN);
      setHip (SERVO_BR_HIP,  HIP_MID + 40);
      break;
    case 3:
      setKnee(SERVO_FR_KNEE, KNEE_MID);
      setHip (SERVO_FR_HIP,  HIP_MID - 30);
      setKnee(SERVO_BR_KNEE, KNEE_MID);
      setHip (SERVO_BR_HIP,  HIP_MID - 30);
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

// ─── Virage DROITE ───────────────────────────────────────────
// Symétrique du virage gauche
void walkRight() {
  if (millis() - lastStep < stepDelay()) return;
  lastStep = millis();

  switch (gaitStep) {
    case 0:
      setKnee(SERVO_FR_KNEE, KNEE_MIN);
      setHip (SERVO_FR_HIP,  HIP_MID + 40);
      setKnee(SERVO_BR_KNEE, KNEE_MIN);
      setHip (SERVO_BR_HIP,  HIP_MID + 40);
      break;
    case 1:
      setKnee(SERVO_FR_KNEE, KNEE_MID);
      setHip (SERVO_FR_HIP,  HIP_MID - 30);
      setKnee(SERVO_BR_KNEE, KNEE_MID);
      setHip (SERVO_BR_HIP,  HIP_MID - 30);
      break;
    case 2:
      setKnee(SERVO_FL_KNEE, KNEE_MIN);
      setHip (SERVO_FL_HIP,  HIP_MID - 40);
      setKnee(SERVO_BL_KNEE, KNEE_MIN);
      setHip (SERVO_BL_HIP,  HIP_MID - 40);
      break;
    case 3:
      setKnee(SERVO_FL_KNEE, KNEE_MID);
      setHip (SERVO_FL_HIP,  HIP_MID + 30);
      setKnee(SERVO_BL_KNEE, KNEE_MID);
      setHip (SERVO_BL_HIP,  HIP_MID + 30);
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
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

      Command prev = currentCmd;
      if      (strcmp(cmd, "forward")  == 0) currentCmd = CMD_FORWARD;
      else if (strcmp(cmd, "backward") == 0) currentCmd = CMD_BACKWARD;
      else if (strcmp(cmd, "left")     == 0) currentCmd = CMD_LEFT;
      else if (strcmp(cmd, "right")    == 0) currentCmd = CMD_RIGHT;
      else                                    currentCmd = CMD_STOP;

      // Repart de l'étape 0 si la direction change
      if (currentCmd != prev) gaitStep = 0;

      Serial.printf("CMD=%d speed=%d\n", (int)currentCmd, speed);
    }
  }

  if (type == WStype_DISCONNECTED) {
    currentCmd = CMD_STOP;
    standStill();
  }
}

// ─── HTTP ────────────────────────────────────────────────────
void handleRoot() {
  httpServer.send(200, "text/plain",
    "ESP32 QuadBot OK\n"
    "WebSocket : ws://IP:81/ws\n"
    "Commandes : {\"type\":\"cmd\",\"cmd\":\"forward|backward|left|right|stop\",\"speed\":0-100}");
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(MQ9_ANALOG_PIN,  INPUT);
  pinMode(MQ9_DIGITAL_PIN, INPUT);

  // ── Fix I2C : pins explicites ──
  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);   // sans setOscillatorFrequency — plus stable
  delay(500);

  standStill();
  Serial.println("PCA9685 OK — servos position neutre");

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

  switch (currentCmd) {
    case CMD_FORWARD:  walkForward();  break;
    case CMD_BACKWARD: walkBackward(); break;
    case CMD_LEFT:     walkLeft();     break;
    case CMD_RIGHT:    walkRight();    break;
    case CMD_STOP:
    default:
      standStill();
      break;
  }

  // ── Envoi MQ-9 toutes les 2 s ──
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