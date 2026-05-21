#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include "esp_system.h"

// ═══════════════════════════════════════════════════════════════
//  WiFi
// ═══════════════════════════════════════════════════════════════
const char* SSID     = "A51 de Cle";
const char* PASSWORD = "sgru3285";

// ═══════════════════════════════════════════════════════════════
//  GPIO ESP32-S3
// ═══════════════════════════════════════════════════════════════
#define I2C_SDA         8
#define I2C_SCL         9
#define MQ9_ANALOG_PIN  4
#define MQ9_DIGITAL_PIN 5
#define RL_VALUE  10.0f
#define RO        10.0f
#define ADC_MAX   4095.0f
#define VCC_V     3.3f

// ═══════════════════════════════════════════════════════════════
//  PCA9685 — canaux
// ═══════════════════════════════════════════════════════════════
#define CH_BR_HIP   0
#define CH_BR_KNEE  1
#define CH_BL_HIP   2
#define CH_BL_KNEE  3
#define CH_FR_HIP   4
#define CH_FR_KNEE  5
#define CH_FL_HIP   6
#define CH_FL_KNEE  7
#define CH_BR_BALL  8
#define CH_BL_BALL  9
#define CH_FR_BALL 10
#define CH_FL_BALL 11

// ═══════════════════════════════════════════════════════════════
//  Limites mécaniques absolues
// ═══════════════════════════════════════════════════════════════
#define LIM_BR_HIP_MIN  130
#define LIM_BR_HIP_MAX  540
#define LIM_BL_HIP_MIN  130
#define LIM_BL_HIP_MAX  505
#define LIM_FR_HIP_MIN  150
#define LIM_FR_HIP_MAX  510
#define LIM_FL_HIP_MIN  125
#define LIM_FL_HIP_MAX  510

#define LIM_BR_KNEE_MIN 210
#define LIM_BR_KNEE_MAX 440
#define LIM_BL_KNEE_MIN 210
#define LIM_BL_KNEE_MAX 435
#define LIM_FR_KNEE_MIN 195
#define LIM_FR_KNEE_MAX 415
#define LIM_FL_KNEE_MIN 135
#define LIM_FL_KNEE_MAX 360

// ═══════════════════════════════════════════════════════════════
//  Positions calibrées — telles que vous les avez réglées
// ═══════════════════════════════════════════════════════════════

// Hanches home (position debout)
#define HOME_BR_HIP   471   // ch0
#define HOME_BL_HIP   181   // ch2
#define HOME_FR_HIP   466   // ch4
#define HOME_FL_HIP   181   // ch6

// Genoux debout (tendu = patte au sol)
#define TENDU_BR  350   // ch1
#define TENDU_BL  285   // ch3
#define TENDU_FR  345   // ch5
#define TENDU_FL  210   // ch7

// Genoux levés (plié = patte en l'air)
#define PLIE_BR   301   // ch1
#define PLIE_BL   344   // ch3
#define PLIE_FR   286   // ch5
#define PLIE_FL   269   // ch7

// Rotules (contre-couple fixe)
#define BALL_BR   280   // ch8
#define BALL_BL   320   // ch9
#define BALL_FR   320   // ch10
#define BALL_FL   325   // ch11

// ═══════════════════════════════════════════════════════════════
//  Amplitude de pas hanches
//  ch0 BR et ch4 FR : montés inversés → avancer = +ticks
//  ch2 BL et ch6 FL : normaux         → avancer = -ticks
// ═══════════════════════════════════════════════════════════════
#define HIP_STEP  75

static inline uint16_t hipBR(int delta) {
  return (uint16_t)constrain(HOME_BR_HIP + delta, LIM_BR_HIP_MIN, LIM_BR_HIP_MAX);
}
static inline uint16_t hipBL(int delta) {
  return (uint16_t)constrain(HOME_BL_HIP - delta, LIM_BL_HIP_MIN, LIM_BL_HIP_MAX);
}
static inline uint16_t hipFR(int delta) {
  return (uint16_t)constrain(HOME_FR_HIP + delta, LIM_FR_HIP_MIN, LIM_FR_HIP_MAX);
}
static inline uint16_t hipFL(int delta) {
  return (uint16_t)constrain(HOME_FL_HIP - delta, LIM_FL_HIP_MIN, LIM_FL_HIP_MAX);
}

// ═══════════════════════════════════════════════════════════════
//  Objets globaux
// ═══════════════════════════════════════════════════════════════
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
WebServer                httpServer(80);
WebSocketsServer         wsServer(81);
uint16_t servoPos[16];

// ═══════════════════════════════════════════════════════════════
//  setPWM direct — envoie immédiatement SANS attente
//  Utilisé pour commander plusieurs servos quasi-simultanément
//  (délai I²C naturel ~300µs entre deux setPWM)
// ═══════════════════════════════════════════════════════════════
void setPWM(uint8_t ch, uint16_t ticks) {
  if (ch >= 16) return;
  ticks = (uint16_t)constrain((int)ticks, 0, 600);
  servoPos[ch] = ticks;
  pca.setPWM(ch, 0, ticks);
}

// ═══════════════════════════════════════════════════════════════
//  waitMove — attend ms tout en maintenant le WebSocket vivant
// ═══════════════════════════════════════════════════════════════
void waitMove(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    wsServer.loop();
    httpServer.handleClient();
    delay(5);
  }
}

// ═══════════════════════════════════════════════════════════════
//  sendServo — pour calibration/init : envoie + attend
// ═══════════════════════════════════════════════════════════════
void sendServo(uint8_t ch, uint16_t ticks) {
  setPWM(ch, ticks);
  waitMove(300);
}

// ═══════════════════════════════════════════════════════════════
//  État machine
// ═══════════════════════════════════════════════════════════════
enum Command { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };
volatile Command pendingCmd = CMD_STOP;
volatile uint8_t speedPct   = 50;
Command       currentCmd  = CMD_STOP;
uint8_t       gaitStep    = 0;
uint32_t      lastSensorMs = 0;
bool          wasMoving   = false;

// ═══════════════════════════════════════════════════════════════
//  Timing de marche selon vitesse
//  speed=0  → 800ms/phase  |  speed=100 → 300ms/phase
// ═══════════════════════════════════════════════════════════════
static inline unsigned long moveMs() {
  return (unsigned long)map(speedPct, 0, 100, 800, 300);
}

// ═══════════════════════════════════════════════════════════════
//  standStill — retour home séquentiel
// ═══════════════════════════════════════════════════════════════
void standStill() {
  gaitStep = 0;
  sendServo(CH_BR_KNEE, (TENDU_BR + PLIE_BR) / 2);
  sendServo(CH_BL_KNEE, (TENDU_BL + PLIE_BL) / 2);
  sendServo(CH_FR_KNEE, (TENDU_FR + PLIE_FR) / 2);
  sendServo(CH_FL_KNEE, (TENDU_FL + PLIE_FL) / 2);
  sendServo(CH_BR_HIP,  HOME_BR_HIP);
  sendServo(CH_BL_HIP,  HOME_BL_HIP);
  sendServo(CH_FR_HIP,  HOME_FR_HIP);
  sendServo(CH_FL_HIP,  HOME_FL_HIP);
  sendServo(CH_BR_KNEE, TENDU_BR);
  sendServo(CH_BL_KNEE, TENDU_BL);
  sendServo(CH_FR_KNEE, TENDU_FR);
  sendServo(CH_FL_KNEE, TENDU_FL);
}

// ═══════════════════════════════════════════════════════════════
//  TROT DIAGONAL — mécanique correcte
//
//  Paire A = FR (avant droit)  + BL (arrière gauche)
//  Paire B = FL (avant gauche) + BR (arrière droit)
//
//  Les deux membres d'une même paire sont commandés ENSEMBLE
//  (deux setPWM() consécutifs sans attente = ~600µs d'écart).
//  L'attente waitMove() vient APRÈS pour laisser les servos
//  atteindre leur position.
//
//  Séquence AVANT — 6 phases :
//
//  Phase 0 : lever paire A ensemble
//    → FR genou plié + BL genou plié   (attente moveMs)
//
//  Phase 1 : avancer paire A ensemble
//    → FR hanche +STEP + BL hanche +STEP   (attente moveMs)
//
//  Phase 2 : poser paire A ensemble
//    → FR genou tendu + BL genou tendu   (attente moveMs)
//
//  Phase 3 : pousser paire B ensemble (propulsion)
//    → FL hanche -STEP + BR hanche -STEP   (attente moveMs)
//
//  Phase 4 : lever paire B ensemble
//    → FL genou plié + BR genou plié   (attente moveMs)
//
//  Phase 5 : avancer paire B ensemble
//    → FL hanche +STEP + BR hanche +STEP   (attente moveMs)
//
//  Phase 6 : poser paire B ensemble
//    → FL genou tendu + BR genou tendu   (attente moveMs)
//
//  Phase 7 : pousser paire A ensemble (propulsion)
//    → FR hanche -STEP + BL hanche -STEP   (attente moveMs)
//
//  Total : 8 phases par cycle
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
//  Trot diagonal — genou ET hanche bougent simultanément
//
//  Paire A = FR + BL  |  Paire B = FL + BR
//
//  Phase 0 : paire A lève ET avance simultanément
//    → 4 setPWM d'affilée (FR knee plié, FR hip +step, BL knee plié, BL hip +step)
//    → un seul waitMove() pour les 4
//
//  Phase 1 : paire A pose (tend les genoux) + paire B pousse
//    → 4 setPWM (FR knee tendu, BL knee tendu, FL hip -step, BR hip -step)
//    → waitMove()
//
//  Phase 2 : paire B lève ET avance simultanément
//  Phase 3 : paire B pose + paire A pousse
//
//  Total : 4 phases par cycle — mouvement fluide et rapide
// ═══════════════════════════════════════════════════════════════
void walkForward() {
  unsigned long ms = moveMs();

  switch (gaitStep) {

    case 0:
      // Paire A (FR+BL) : lever ET avancer en même temps
      setPWM(CH_FR_KNEE, PLIE_FR);
      setPWM(CH_FR_HIP,  hipFR(+HIP_STEP));
      setPWM(CH_BL_KNEE, PLIE_BL);
      setPWM(CH_BL_HIP,  hipBL(+HIP_STEP));
      waitMove(ms);
      break;

    case 1:
      // Paire A : poser (tendre genoux)
      // Paire B : pousser vers l'arrière (propulsion)
      setPWM(CH_FR_KNEE, TENDU_FR);
      setPWM(CH_BL_KNEE, TENDU_BL);
      setPWM(CH_FL_HIP,  hipFL(-HIP_STEP));
      setPWM(CH_BR_HIP,  hipBR(-HIP_STEP));
      waitMove(ms);
      break;

    case 2:
      // Paire B (FL+BR) : lever ET avancer en même temps
      setPWM(CH_FL_KNEE, PLIE_FL);
      setPWM(CH_FL_HIP,  hipFL(+HIP_STEP));
      setPWM(CH_BR_KNEE, PLIE_BR);
      setPWM(CH_BR_HIP,  hipBR(+HIP_STEP));
      waitMove(ms);
      break;

    case 3:
      // Paire B : poser
      // Paire A : pousser vers l'arrière (propulsion)
      setPWM(CH_FL_KNEE, TENDU_FL);
      setPWM(CH_BR_KNEE, TENDU_BR);
      setPWM(CH_FR_HIP,  hipFR(-HIP_STEP));
      setPWM(CH_BL_HIP,  hipBL(-HIP_STEP));
      waitMove(ms);
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

void walkBackward() {
  unsigned long ms = moveMs();

  switch (gaitStep) {

    case 0:
      // Paire A (FR+BL) : lever ET reculer en même temps
      setPWM(CH_FR_KNEE, PLIE_FR);
      setPWM(CH_FR_HIP,  hipFR(-HIP_STEP));
      setPWM(CH_BL_KNEE, PLIE_BL);
      setPWM(CH_BL_HIP,  hipBL(-HIP_STEP));
      waitMove(ms);
      break;

    case 1:
      // Paire A : poser
      // Paire B : pousser vers l'avant
      setPWM(CH_FR_KNEE, TENDU_FR);
      setPWM(CH_BL_KNEE, TENDU_BL);
      setPWM(CH_FL_HIP,  hipFL(+HIP_STEP));
      setPWM(CH_BR_HIP,  hipBR(+HIP_STEP));
      waitMove(ms);
      break;

    case 2:
      // Paire B (FL+BR) : lever ET reculer en même temps
      setPWM(CH_FL_KNEE, PLIE_FL);
      setPWM(CH_FL_HIP,  hipFL(-HIP_STEP));
      setPWM(CH_BR_KNEE, PLIE_BR);
      setPWM(CH_BR_HIP,  hipBR(-HIP_STEP));
      waitMove(ms);
      break;

    case 3:
      // Paire B : poser
      // Paire A : pousser vers l'avant
      setPWM(CH_FL_KNEE, TENDU_FL);
      setPWM(CH_BR_KNEE, TENDU_BR);
      setPWM(CH_FR_HIP,  hipFR(+HIP_STEP));
      setPWM(CH_BL_HIP,  hipBL(+HIP_STEP));
      waitMove(ms);
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

// ═══════════════════════════════════════════════════════════════
//  MQ-9
// ═══════════════════════════════════════════════════════════════
struct SensorData { int raw; float voltage, rs, ratio, ppm; bool alert; };

SensorData readMQ9() {
  SensorData d;
  d.raw     = analogRead(MQ9_ANALOG_PIN);
  d.alert   = (digitalRead(MQ9_DIGITAL_PIN) == LOW);
  d.voltage = (d.raw / ADC_MAX) * VCC_V;
  d.rs      = (d.voltage > 0.01f) ? RL_VALUE * (VCC_V - d.voltage) / d.voltage : 0.0f;
  d.ratio   = (RO > 0.0f) ? d.rs / RO : 0.0f;
  d.ppm     = (d.ratio > 0.0f) ? 599.65f * powf(d.ratio, -2.244f) : 0.0f;
  return d;
}

// ═══════════════════════════════════════════════════════════════
//  WebSocket
// ═══════════════════════════════════════════════════════════════
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
      if (!cmd) return;
      if (doc.containsKey("speed"))
        speedPct = (uint8_t)constrain((int)doc["speed"], 0, 100);
      Command next = CMD_STOP;
      if      (strcmp(cmd, "forward")  == 0) next = CMD_FORWARD;
      else if (strcmp(cmd, "backward") == 0) next = CMD_BACKWARD;
      else if (strcmp(cmd, "left")     == 0) next = CMD_LEFT;
      else if (strcmp(cmd, "right")    == 0) next = CMD_RIGHT;
      if (next != pendingCmd) gaitStep = 0;
      pendingCmd = next;
      Serial.printf("[CMD] %s spd=%d\n", cmd, speedPct);
    }

    if (strcmp(t, "cal") == 0) {
      uint8_t  ch    = (uint8_t)(int)doc["ch"];
      uint16_t ticks = (uint16_t)constrain((int)doc["ticks"], 0, 600);
      if (ch >= 16) return;
      pca.setPWM(ch, 0, ticks);
      servoPos[ch] = ticks;
      Serial.printf("[CAL] ch=%d ticks=%d\n", ch, ticks);
    }
  }

  if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] #%d deconnecte\n", num);
    pendingCmd = CMD_STOP;
  }
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] #%d connecte\n", num);
  }
}

// ═══════════════════════════════════════════════════════════════
//  HTTP
// ═══════════════════════════════════════════════════════════════
void handleRoot() {
  httpServer.send(200, "text/plain", "QuadBot S3 OK\nws://IP:81");
}

// ═══════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);

  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("\n[BOOT] ESP32-S3 reset=%d", (int)r);
  if (r == ESP_RST_BROWNOUT) Serial.print(" *** BROWNOUT ***");
  if (r == ESP_RST_PANIC)    Serial.print(" *** PANIC ***");
  Serial.println();

  pinMode(MQ9_ANALOG_PIN,  INPUT);
  pinMode(MQ9_DIGITAL_PIN, INPUT);
  memset(servoPos, 0xFF, sizeof(servoPos));

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  pca.begin();
  pca.setOscillatorFrequency(25000000);
  pca.setPWMFreq(50);
  delay(150);
  Serial.println("[PCA] OK 25MHz/50Hz");

  Serial.println("[CFG] Positions :");
  Serial.printf("  BR hip home=%d avant=%d arriere=%d knee tendu=%d plie=%d\n",
    HOME_BR_HIP, hipBR(+HIP_STEP), hipBR(-HIP_STEP), TENDU_BR, PLIE_BR);
  Serial.printf("  BL hip home=%d avant=%d arriere=%d knee tendu=%d plie=%d\n",
    HOME_BL_HIP, hipBL(+HIP_STEP), hipBL(-HIP_STEP), TENDU_BL, PLIE_BL);
  Serial.printf("  FR hip home=%d avant=%d arriere=%d knee tendu=%d plie=%d\n",
    HOME_FR_HIP, hipFR(+HIP_STEP), hipFR(-HIP_STEP), TENDU_FR, PLIE_FR);
  Serial.printf("  FL hip home=%d avant=%d arriere=%d knee tendu=%d plie=%d\n",
    HOME_FL_HIP, hipFL(+HIP_STEP), hipFL(-HIP_STEP), TENDU_FL, PLIE_FL);

  // Init séquentielle
  Serial.println("[SERVO] Init home...");
  sendServo(CH_BR_HIP,  HOME_BR_HIP);
  sendServo(CH_BR_KNEE, TENDU_BR);
  sendServo(CH_BL_HIP,  HOME_BL_HIP);
  sendServo(CH_BL_KNEE, TENDU_BL);
  sendServo(CH_FR_HIP,  HOME_FR_HIP);
  sendServo(CH_FR_KNEE, TENDU_FR);
  sendServo(CH_FL_HIP,  HOME_FL_HIP);
  sendServo(CH_FL_KNEE, TENDU_FL);
  sendServo(CH_BR_BALL, BALL_BR);
  sendServo(CH_BL_BALL, BALL_BL);
  sendServo(CH_FR_BALL, BALL_FR);
  sendServo(CH_FL_BALL, BALL_FL);
  Serial.println("[SERVO] Home OK");

  Serial.printf("[WiFi] Connexion a %s", SSID);
  WiFi.begin(SSID, PASSWORD);
  uint8_t att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 40) {
    delay(500); Serial.print("."); att++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] IP : %s\n",       WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] WS : ws://%s:81\n",  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] ECHEC");
  }

  httpServer.on("/", handleRoot);
  httpServer.begin();
  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println("[SERVER] OK");
}

// ═══════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════
void loop() {
  httpServer.handleClient();
  wsServer.loop();

  Command cmd = pendingCmd;
  if (cmd != currentCmd) {
    if (cmd == CMD_STOP && wasMoving) {
      standStill();
      wasMoving = false;
    }
    currentCmd = cmd;
    gaitStep = 0;
  }

  switch (currentCmd) {
    case CMD_FORWARD:  wasMoving = true; walkForward();  break;
    case CMD_BACKWARD: wasMoving = true; walkBackward(); break;
    case CMD_STOP:
    default: break;
  }

  if (millis() - lastSensorMs > 2000) {
    lastSensorMs = millis();
    SensorData d = readMQ9();
    StaticJsonDocument<160> doc;
    doc["type"]    = "sensor";
    doc["raw"]     = d.raw;
    doc["voltage"] = serialized(String(d.voltage, 3));
    doc["rs"]      = serialized(String(d.rs,      3));
    doc["ratio"]   = serialized(String(d.ratio,   3));
    doc["ppm"]     = serialized(String(d.ppm,     1));
    doc["alert"]   = d.alert;
    String out; serializeJson(doc, out);
    wsServer.broadcastTXT(out);
    Serial.printf("[MQ9] %.1fppm step=%d cmd=%d\n",
      d.ppm, gaitStep, (int)currentCmd);
  }
}