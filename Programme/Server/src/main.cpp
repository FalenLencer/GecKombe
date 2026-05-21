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
//  Limites mécaniques absolues (ne jamais dépasser)
//  Mesurées sur le robot + 10 ticks de marge de sécurité
// ═══════════════════════════════════════════════════════════════
#define LIM_BR_HIP_MIN  130   // ch0
#define LIM_BR_HIP_MAX  540
#define LIM_BL_HIP_MIN  130   // ch2
#define LIM_BL_HIP_MAX  505
#define LIM_FR_HIP_MIN  150   // ch4
#define LIM_FR_HIP_MAX  510
#define LIM_FL_HIP_MIN  125   // ch6
#define LIM_FL_HIP_MAX  510

#define LIM_BR_KNEE_MIN 210   // ch1
#define LIM_BR_KNEE_MAX 440
#define LIM_BL_KNEE_MIN 210   // ch3
#define LIM_BL_KNEE_MAX 435
#define LIM_FR_KNEE_MIN 195   // ch5
#define LIM_FR_KNEE_MAX 415
#define LIM_FL_KNEE_MIN 135   // ch7
#define LIM_FL_KNEE_MAX 360

// ═══════════════════════════════════════════════════════════════
//  Positions de marche
//
//  HOME = milieu mécanique de chaque hanche (position debout)
//  TENDU = genou étendu (patte au sol)
//  PLIE  = genou fléchi pour la marche (~37° depuis tendu = 56 ticks)
//
//  Hanches : ±33 ticks autour du home (~22°)
//  ch0 BR et ch4 FR sont montés inversés :
//    delta positif → ticks augmentent (avancer)
//  ch2 BL et ch6 FL sont normaux :
//    delta positif → ticks diminuent (avancer)
// ═══════════════════════════════════════════════════════════════

// Positions home (milieu mécanique)
#define HOME_BR_HIP   403   // 335 + 68 ticks (+45°)
#define HOME_BL_HIP   249   // 317 - 68 ticks (+45°)
#define HOME_FR_HIP   398   // 330 + 68 ticks (+45°)
#define HOME_FL_HIP   249   // 317 - 68 ticks (+45°)

// Genoux : position debout (tendu)
#define TENDU_BR  210
#define TENDU_BL  435
#define TENDU_FR  195
#define TENDU_FL  360

// Genoux : position levée pour la marche (~37° = 56 ticks depuis tendu)
// BR/FR : plie = tendu + 56  (plier = augmenter)
// BL/FL : plie = tendu - 56  (plier = diminuer)
#define PLIE_BR   301   // 210 + 91 (~60°)
#define PLIE_BL   344   // 435 - 91 (~60°)
#define PLIE_FR   286   // 195 + 91 (~60°)
#define PLIE_FL   269   // 360 - 91 (~60°)

// Amplitude hanche : 33 ticks (~22°) de chaque côté du home
#define HIP_STEP  75  // ~49° — bien visible

// Rotules
#define BALL_BR   325
#define BALL_BL   320
#define BALL_FR   320
#define BALL_FL   325

// ═══════════════════════════════════════════════════════════════
//  Helpers hanche avec direction et limites
//  delta > 0 = avancer la patte
//  delta < 0 = reculer la patte
// ═══════════════════════════════════════════════════════════════
static inline uint16_t hipBR(int delta) {
  // ch0 inversé : avancer = +ticks
  return (uint16_t)constrain(HOME_BR_HIP + delta, LIM_BR_HIP_MIN, LIM_BR_HIP_MAX);
}
static inline uint16_t hipBL(int delta) {
  // ch2 normal : avancer = -ticks
  return (uint16_t)constrain(HOME_BL_HIP - delta, LIM_BL_HIP_MIN, LIM_BL_HIP_MAX);
}
static inline uint16_t hipFR(int delta) {
  // ch4 inversé : avancer = +ticks
  return (uint16_t)constrain(HOME_FR_HIP + delta, LIM_FR_HIP_MIN, LIM_FR_HIP_MAX);
}
static inline uint16_t hipFL(int delta) {
  // ch6 normal : avancer = -ticks
  return (uint16_t)constrain(HOME_FL_HIP - delta, LIM_FL_HIP_MIN, LIM_FL_HIP_MAX);
}

// ═══════════════════════════════════════════════════════════════
//  sendServo — bloquant, attend la fin du mouvement
//  wsServer.loop() appelé pendant l'attente → pas de déconnexion
// ═══════════════════════════════════════════════════════════════
#define SERVO_MOVE_MS  400

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
WebServer                httpServer(80);
WebSocketsServer         wsServer(81);
uint16_t servoPos[16];

void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    wsServer.loop();
    httpServer.handleClient();
    delay(5);
  }
}

void sendServo(uint8_t ch, uint16_t ticks) {
  if (ch >= 16) return;
  ticks = (uint16_t)constrain((int)ticks, 0, 600);
  // On envoie toujours la commande (pas de skip sur position identique)
  // pour garantir que le servo maintient sa position
  servoPos[ch] = ticks;
  pca.setPWM(ch, 0, ticks);
  // Attente que le servo arrive en position
  waitMs(SERVO_MOVE_MS);
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
//  standStill — retour home, un servo à la fois
// ═══════════════════════════════════════════════════════════════
void standStill() {
  gaitStep = 0;
  // 1. Genoux mi-course (évite les à-coups)
  sendServo(CH_BR_KNEE, (TENDU_BR + PLIE_BR) / 2);
  sendServo(CH_BL_KNEE, (TENDU_BL + PLIE_BL) / 2);
  sendServo(CH_FR_KNEE, (TENDU_FR + PLIE_FR) / 2);
  sendServo(CH_FL_KNEE, (TENDU_FL + PLIE_FL) / 2);
  // 2. Hanches au centre
  sendServo(CH_BR_HIP, HOME_BR_HIP);
  sendServo(CH_BL_HIP, HOME_BL_HIP);
  sendServo(CH_FR_HIP, HOME_FR_HIP);
  sendServo(CH_FL_HIP, HOME_FL_HIP);
  // 3. Genoux tendus (debout)
  sendServo(CH_BR_KNEE, TENDU_BR);
  sendServo(CH_BL_KNEE, TENDU_BL);
  sendServo(CH_FR_KNEE, TENDU_FR);
  sendServo(CH_FL_KNEE, TENDU_FL);
}

// ═══════════════════════════════════════════════════════════════
//  Trot diagonal — 16 étapes, un servo à la fois
//
//  Paire A = FR + BL  (diagonal)
//  Paire B = FL + BR  (diagonal)
//
//  Séquence par demi-cycle (8 étapes) :
//   0. Plier genou FR
//   1. Plier genou BL
//   2. Avancer hanche FR
//   3. Avancer hanche BL
//   4. Tendre genou FR
//   5. Tendre genou BL
//   6. Reculer hanche FL (propulsion, patte au sol)
//   7. Reculer hanche BR (propulsion, patte au sol)
//  Puis symétrique pour paire B (étapes 8-15)
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
//  Séquence de marche à 4 temps
//
//  Lever un membre = 3 sous-étapes séquentielles :
//    a) plier genou
//    b) avancer hanche
//    c) tendre genou
//  Pendant ce temps, les 3 autres pattes au sol poussent.
//
//  Temps 1 : lever FR  → pousser BL+FL+BR
//  Temps 2 : lever BL  → pousser FR+FL+BR
//  Temps 3 : lever FL  → pousser BR+FR+BL
//  Temps 4 : lever BR  → pousser FL+FR+BL
//
//  Total : 4 temps × 5 étapes = 20 étapes par cycle complet
//    étapes 0-2   : lever FR  (plier, avancer, tendre)
//    étape  3-4   : pousser pattes au sol après pose FR
//    étapes 5-7   : lever BL
//    étape  8-9   : pousser
//    étapes 10-12 : lever FL
//    étape  13-14 : pousser
//    étapes 15-17 : lever BR
//    étape  18-19 : pousser
// ═══════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════
//  Trot diagonal — 2 temps, un servo à la fois
//
//  Paire A = FR (avant droit) + BL (arrière gauche)
//  Paire B = FL (avant gauche) + BR (arrière droit)
//
//  Séquence AVANT — 14 étapes par cycle :
//
//  Phase 1 — Lever paire A (FR+BL) et avancer :
//    0  plier genou FR
//    1  avancer hanche FR  (+HIP_STEP vers avant)
//    2  plier genou BL
//    3  avancer hanche BL  (+HIP_STEP vers avant)
//    4  tendre genou FR    (pose FR)
//    5  tendre genou BL    (pose BL)
//
//  Phase 2 — Paire B au sol : pousser vers l'arrière (propulsion)
//    6  reculer hanche FL  (-HIP_STEP)
//    7  reculer hanche BR  (-HIP_STEP)
//
//  Phase 3 — Lever paire B (FL+BR) et avancer :
//    8  plier genou FL
//    9  avancer hanche FL
//   10  plier genou BR
//   11  avancer hanche BR
//   12  tendre genou FL
//   13  tendre genou BR
//
//  Phase 4 — Paire A au sol : pousser
//   (implicite au début du cycle suivant via phases 6-7 inversées)
//   14  reculer hanche FR
//   15  reculer hanche BL
//
//  Total : 16 étapes
// ═══════════════════════════════════════════════════════════════
void walkForward() {
  switch (gaitStep) {

    // ── Phase 1 : lever paire A (FR+BL), avancer ────────────
    case  0: sendServo(CH_FR_KNEE, PLIE_FR);           break; // plier FR
    case  1: sendServo(CH_FR_HIP,  hipFR(+HIP_STEP));  break; // avancer FR
    case  2: sendServo(CH_BL_KNEE, PLIE_BL);           break; // plier BL
    case  3: sendServo(CH_BL_HIP,  hipBL(+HIP_STEP));  break; // avancer BL
    case  4: sendServo(CH_FR_KNEE, TENDU_FR);          break; // poser FR
    case  5: sendServo(CH_BL_KNEE, TENDU_BL);          break; // poser BL

    // ── Phase 2 : paire B au sol pousse vers l'arrière ───────
    case  6: sendServo(CH_FL_HIP,  hipFL(-HIP_STEP));  break; // FL pousse
    case  7: sendServo(CH_BR_HIP,  hipBR(-HIP_STEP));  break; // BR pousse

    // ── Phase 3 : lever paire B (FL+BR), avancer ────────────
    case  8: sendServo(CH_FL_KNEE, PLIE_FL);           break; // plier FL
    case  9: sendServo(CH_FL_HIP,  hipFL(+HIP_STEP));  break; // avancer FL
    case 10: sendServo(CH_BR_KNEE, PLIE_BR);           break; // plier BR
    case 11: sendServo(CH_BR_HIP,  hipBR(+HIP_STEP));  break; // avancer BR
    case 12: sendServo(CH_FL_KNEE, TENDU_FL);          break; // poser FL
    case 13: sendServo(CH_BR_KNEE, TENDU_BR);          break; // poser BR

    // ── Phase 4 : paire A au sol pousse vers l'arrière ───────
    case 14: sendServo(CH_FR_HIP,  hipFR(-HIP_STEP));  break; // FR pousse
    case 15: sendServo(CH_BL_HIP,  hipBL(-HIP_STEP));  break; // BL pousse
  }
  gaitStep = (gaitStep + 1) % 16;
}

void walkBackward() {
  switch (gaitStep) {

    // ── Phase 1 : lever paire A (FR+BL), reculer ────────────
    case  0: sendServo(CH_FR_KNEE, PLIE_FR);           break;
    case  1: sendServo(CH_FR_HIP,  hipFR(-HIP_STEP));  break; // reculer FR
    case  2: sendServo(CH_BL_KNEE, PLIE_BL);           break;
    case  3: sendServo(CH_BL_HIP,  hipBL(-HIP_STEP));  break; // reculer BL
    case  4: sendServo(CH_FR_KNEE, TENDU_FR);          break;
    case  5: sendServo(CH_BL_KNEE, TENDU_BL);          break;

    // ── Phase 2 : paire B au sol pousse vers l'avant ─────────
    case  6: sendServo(CH_FL_HIP,  hipFL(+HIP_STEP));  break;
    case  7: sendServo(CH_BR_HIP,  hipBR(+HIP_STEP));  break;

    // ── Phase 3 : lever paire B (FL+BR), reculer ────────────
    case  8: sendServo(CH_FL_KNEE, PLIE_FL);           break;
    case  9: sendServo(CH_FL_HIP,  hipFL(-HIP_STEP));  break; // reculer FL
    case 10: sendServo(CH_BR_KNEE, PLIE_BR);           break;
    case 11: sendServo(CH_BR_HIP,  hipBR(-HIP_STEP));  break; // reculer BR
    case 12: sendServo(CH_FL_KNEE, TENDU_FL);          break;
    case 13: sendServo(CH_BR_KNEE, TENDU_BR);          break;

    // ── Phase 4 : paire A au sol pousse vers l'avant ─────────
    case 14: sendServo(CH_FR_HIP,  hipFR(+HIP_STEP));  break;
    case 15: sendServo(CH_BL_HIP,  hipBL(+HIP_STEP));  break;
  }
  gaitStep = (gaitStep + 1) % 16;
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

  // Résumé des positions de marche
  Serial.println("[CFG] Positions de marche :");
  Serial.printf("  BR hip  home=%d  avant=%d  arriere=%d\n", HOME_BR_HIP, hipBR(+HIP_STEP), hipBR(-HIP_STEP));
  Serial.printf("  BL hip  home=%d  avant=%d  arriere=%d\n", HOME_BL_HIP, hipBL(+HIP_STEP), hipBL(-HIP_STEP));
  Serial.printf("  FR hip  home=%d  avant=%d  arriere=%d\n", HOME_FR_HIP, hipFR(+HIP_STEP), hipFR(-HIP_STEP));
  Serial.printf("  FL hip  home=%d  avant=%d  arriere=%d\n", HOME_FL_HIP, hipFL(+HIP_STEP), hipFL(-HIP_STEP));
  Serial.printf("  BR knee tendu=%d plie=%d\n", TENDU_BR, PLIE_BR);
  Serial.printf("  BL knee tendu=%d plie=%d\n", TENDU_BL, PLIE_BL);
  Serial.printf("  FR knee tendu=%d plie=%d\n", TENDU_FR, PLIE_FR);
  Serial.printf("  FL knee tendu=%d plie=%d\n", TENDU_FL, PLIE_FL);
  Serial.printf("[CFG] HIP_STEP=%d  SERVO_MOVE_MS=%d\n", HIP_STEP, SERVO_MOVE_MS);

  // Init séquentielle un servo à la fois
  Serial.println("[SERVO] Init home...");
  sendServo(CH_BR_HIP,  HOME_BR_HIP);
  sendServo(CH_BR_KNEE, TENDU_BR);
  sendServo(CH_BL_HIP,  HOME_BL_HIP);
  sendServo(CH_BL_KNEE, TENDU_BL);
  sendServo(CH_FR_HIP,  HOME_FR_HIP);
  sendServo(CH_FR_KNEE, TENDU_FR);
  sendServo(CH_FL_HIP,  HOME_FL_HIP);
  sendServo(CH_FL_KNEE, TENDU_FL);
  // Rotules
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
