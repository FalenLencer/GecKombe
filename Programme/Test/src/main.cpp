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

// ─── PCA9685 ─────────────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ─── Web ─────────────────────────────────────────────────────
WebServer httpServer(80);
WebSocketsServer wsServer(81);

// ─── Servos (à ajuster selon ton robot) ──────────────────────
#define SERVO_MIN  150
#define SERVO_MID  375
#define SERVO_MAX  600

#define SERVO_BR_HIP 0
#define SERVO_BL_HIP 2
#define SERVO_FR_HIP 4
#define SERVO_FL_HIP 6

// ─── Commandes ───────────────────────────────────────────────
enum Command { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };
volatile Command currentCmd = CMD_STOP;

// ─── Variables marche ────────────────────────────────────────
uint8_t step = 0;
unsigned long lastStep = 0;

// ─── Servo helper ────────────────────────────────────────────
void setServo(uint8_t ch, uint16_t ticks) {
  ticks = constrain(ticks, SERVO_MIN, SERVO_MAX);
  pca.setPWM(ch, 0, ticks);
}

// ─── Position neutre ─────────────────────────────────────────
void standStill() {
  setServo(SERVO_BR_HIP, SERVO_MID);
  setServo(SERVO_BL_HIP, SERVO_MID);
  setServo(SERVO_FR_HIP, SERVO_MID);
  setServo(SERVO_FL_HIP, SERVO_MID);
}

// ─── Marche AVANT (simple et robuste) ────────────────────────
void walkForward() {
  if (millis() - lastStep < 300) return;
  lastStep = millis();

  switch (step) {

    case 0: // lever diagonale FR + BL
      setServo(SERVO_FR_HIP, SERVO_MIN);
      setServo(SERVO_BL_HIP, SERVO_MAX);
      break;

    case 1: // retour centre
      setServo(SERVO_FR_HIP, SERVO_MID);
      setServo(SERVO_BL_HIP, SERVO_MID);
      break;

    case 2: // lever diagonale FL + BR
      setServo(SERVO_FL_HIP, SERVO_MAX);
      setServo(SERVO_BR_HIP, SERVO_MIN);
      break;

    case 3: // retour centre
      setServo(SERVO_FL_HIP, SERVO_MID);
      setServo(SERVO_BR_HIP, SERVO_MID);
      break;
  }

  step = (step + 1) % 4;
}

// ─── Marche ARRIÈRE (inverse simple) ─────────────────────────
void walkBackward() {
  if (millis() - lastStep < 300) return;
  lastStep = millis();

  switch (step) {

    case 0:
      setServo(SERVO_FR_HIP, SERVO_MAX);
      setServo(SERVO_BL_HIP, SERVO_MIN);
      break;

    case 1:
      setServo(SERVO_FR_HIP, SERVO_MID);
      setServo(SERVO_BL_HIP, SERVO_MID);
      break;

    case 2:
      setServo(SERVO_FL_HIP, SERVO_MIN);
      setServo(SERVO_BR_HIP, SERVO_MAX);
      break;

    case 3:
      setServo(SERVO_FL_HIP, SERVO_MID);
      setServo(SERVO_BR_HIP, SERVO_MID);
      break;
  }

  step = (step + 1) % 4;
}

// ─── WebSocket ───────────────────────────────────────────────
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {

  if (type == WStype_TEXT) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload, length)) return;

    const char* t = doc["type"];
    if (!t) return;

    if (strcmp(t, "cmd") == 0) {
      const char* cmd = doc["cmd"];

      if      (strcmp(cmd, "forward")  == 0) currentCmd = CMD_FORWARD;
      else if (strcmp(cmd, "backward") == 0) currentCmd = CMD_BACKWARD;
      else currentCmd = CMD_STOP;

      Serial.printf("CMD = %d\n", currentCmd);
    }
  }

  if (type == WStype_DISCONNECTED) {
    currentCmd = CMD_STOP;
  }
}

// ─── HTTP ────────────────────────────────────────────────────
void handleRoot() {
  httpServer.send(200, "text/plain", "ESP32 QuadBot OK");
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // I2C FIX (important)
  Wire.begin(21, 22);

  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  standStill();

  // WiFi
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connecté");

  httpServer.on("/", handleRoot);
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  Serial.println("Serveurs prêts");
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  httpServer.handleClient();
  wsServer.loop();

  switch (currentCmd) {

    case CMD_FORWARD:
      walkForward();
      break;

    case CMD_BACKWARD:
      walkBackward();
      break;

    case CMD_STOP:
    default:
      standStill();
      break;
  }
}