#include "Arduino.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "WiFi.h"

// ─── WiFi ────────────────────────────────────────────────────
const char* SSID     = "A51 de Cle";
const char* PASSWORD = "sgru3285";

// ─── Broches caméra AI Thinker ───────────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


#define LED_FLASH_PIN      4

#define LED_STATUS_PIN    33

// ─── Config streaming ─────────────────────────────────────────
// Ajuste la résolution selon les performances WiFi :
//   FRAMESIZE_QVGA  = 320×240  
//   FRAMESIZE_VGA   = 640×480  
//   FRAMESIZE_SVGA  = 800×600
//   FRAMESIZE_XGA   = 1024×768 
#define CAM_FRAMESIZE   FRAMESIZE_VGA
#define CAM_JPEG_QUALITY  12 
#define CAM_XCLK_FREQ  20000000 

// ─── MJPEG stream : délai entre frames (ms) ──────────────────
#define STREAM_DELAY_MS  0

// ─── Boundary MJPEG ──────────────────────────────────────────
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ─── Handle httpd ─────────────────────────────────────────────
static httpd_handle_t stream_httpd = NULL;
static httpd_handle_t camera_httpd = NULL;

// ─── Handler : flux MJPEG ─────────────────────────────────────
static esp_err_t stream_handler(httpd_req_t* req) {
    camera_fb_t* fb   = NULL;
    esp_err_t    res  = ESP_OK;
    char         part_buf[64];

    // Headers CORS pour autoriser l'accès depuis le dashboard
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    // Blink LED status pendant le stream
    digitalWrite(LED_STATUS_PIN, LOW);  // allume (active LOW)

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[CAM] Échec capture frame");
            res = ESP_FAIL;
            break;
        }

        // Boundary
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                   strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Header de la part
        size_t hlen = snprintf(part_buf, sizeof(part_buf),
                               _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Données JPEG
        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;

        if (STREAM_DELAY_MS > 0) delay(STREAM_DELAY_MS);
    }

    digitalWrite(LED_STATUS_PIN, HIGH);  // éteint
    return res;
}

// ─── Handler : capture JPEG unique ───────────────────────────
static esp_err_t capture_handler(httpd_req_t* req) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "inline; filename=capture.jpg");
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Length",
                       String(fb->len).c_str());
    httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return ESP_OK;
}

// ─── Handler : status JSON ────────────────────────────────────
static esp_err_t status_handler(httpd_req_t* req) {
    sensor_t* s = esp_camera_sensor_get();
    char json[512];
    snprintf(json, sizeof(json),
        "{"
        "\"framesize\":%d,"
        "\"quality\":%d,"
        "\"brightness\":%d,"
        "\"contrast\":%d,"
        "\"saturation\":%d,"
        "\"sharpness\":%d,"
        "\"awb\":%d,"
        "\"awb_gain\":%d,"
        "\"aec\":%d,"
        "\"aec_value\":%d,"
        "\"agc\":%d,"
        "\"agc_gain\":%d,"
        "\"hmirror\":%d,"
        "\"vflip\":%d,"
        "\"ip\":\"%s\""
        "}",
        s->status.framesize,
        s->status.quality,
        s->status.brightness,
        s->status.contrast,
        s->status.saturation,
        s->status.sharpness,
        s->status.awb,
        s->status.awb_gain,
        s->status.aec,
        s->status.aec_value,
        s->status.agc,
        s->status.agc_gain,
        s->status.hmirror,
        s->status.vflip,
        WiFi.localIP().toString().c_str()
    );
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// ─── Handler : contrôle caméra ────────────────────────────────
// Exemples : /control?var=framesize&val=6
//            /control?var=quality&val=10
//            /control?var=vflip&val=1
//            /control?var=hmirror&val=1
//            /control?var=flash&val=1
static esp_err_t control_handler(httpd_req_t* req) {
    char  buf[64];
    char  var[32] = {0};
    char  val[8]  = {0};

    // Parse query string
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "var", var, sizeof(var));
        httpd_query_key_value(buf, "val", val, sizeof(val));
    }

    int value = atoi(val);
    sensor_t* s = esp_camera_sensor_get();
    int res = 0;

    if      (!strcmp(var, "framesize"))   res = s->set_framesize(s, (framesize_t)value);
    else if (!strcmp(var, "quality"))     res = s->set_quality(s, value);
    else if (!strcmp(var, "brightness"))  res = s->set_brightness(s, value);
    else if (!strcmp(var, "contrast"))    res = s->set_contrast(s, value);
    else if (!strcmp(var, "saturation"))  res = s->set_saturation(s, value);
    else if (!strcmp(var, "sharpness"))   res = s->set_sharpness(s, value);
    else if (!strcmp(var, "hmirror"))     res = s->set_hmirror(s, value);
    else if (!strcmp(var, "vflip"))       res = s->set_vflip(s, value);
    else if (!strcmp(var, "awb"))         res = s->set_whitebal(s, value);
    else if (!strcmp(var, "awb_gain"))    res = s->set_awb_gain(s, value);
    else if (!strcmp(var, "aec"))         res = s->set_exposure_ctrl(s, value);
    else if (!strcmp(var, "aec_value"))   res = s->set_aec_value(s, value);
    else if (!strcmp(var, "agc"))         res = s->set_gain_ctrl(s, value);
    else if (!strcmp(var, "agc_gain"))    res = s->set_agc_gain(s, value);
    else if (!strcmp(var, "flash")) {
        digitalWrite(LED_FLASH_PIN, value ? HIGH : LOW);
    }
    else { res = -1; }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (res < 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// ─── Démarrage des serveurs HTTP ─────────────────────────────
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;

    // ── Serveur de stream (port 81) ───────────────────────────
    config.server_port      = 81;
    config.ctrl_port        = 32769;

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri      = "/stream",
            .method   = HTTP_GET,
            .handler  = stream_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("[HTTP] Stream  : http://IP:81/stream");
    }

    // ── Serveur de contrôle (port 80) ─────────────────────────
    config.server_port = 80;
    config.ctrl_port   = 32768;

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_uri_t capture_uri = {
            .uri     = "/capture",
            .method  = HTTP_GET,
            .handler = capture_handler,
            .user_ctx = NULL
        };
        httpd_uri_t status_uri = {
            .uri     = "/status",
            .method  = HTTP_GET,
            .handler = status_handler,
            .user_ctx = NULL
        };
        httpd_uri_t control_uri = {
            .uri     = "/control",
            .method  = HTTP_GET,
            .handler = control_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &control_uri);
        Serial.println("[HTTP] Capture : http://IP/capture");
        Serial.println("[HTTP] Status  : http://IP/status");
        Serial.println("[HTTP] Control : http://IP/control?var=...&val=...");
    }
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    Serial.println("\n[BOOT] ESP32-CAM QuadBot");

    // LEDs
    pinMode(LED_FLASH_PIN,  OUTPUT);
    pinMode(LED_STATUS_PIN, OUTPUT);
    digitalWrite(LED_FLASH_PIN,  LOW);
    digitalWrite(LED_STATUS_PIN, HIGH); // éteint (active LOW)

    // ── Config caméra ─────────────────────────────────────────
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAM_XCLK_FREQ;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = CAM_FRAMESIZE;
    config.jpeg_quality = CAM_JPEG_QUALITY;
    // PSRAM disponible sur AI Thinker → double buffer pour plus de fluidité
    config.fb_count     = psramFound() ? 2 : 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM
                                       : CAMERA_FB_IN_DRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init échouée — erreur 0x%x\n", err);
        // Blink rapide pour signaler l'erreur
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_STATUS_PIN, LOW);  delay(100);
            digitalWrite(LED_STATUS_PIN, HIGH); delay(100);
        }
        return;
    }
    Serial.println("[CAM] Init OK");

    // ── Ajustements capteur OV2640 ────────────────────────────
    sensor_t* s = esp_camera_sensor_get();
    // Orientation : retourne l'image si la cam est montée à l'envers
    s->set_vflip(s, 0);     // 1 = retourner verticalement
    s->set_hmirror(s, 0);   // 1 = miroir horizontal
    // Balance des blancs auto
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    // Exposition auto
    s->set_exposure_ctrl(s, 1);
    // Gain auto
    s->set_gain_ctrl(s, 1);

    // ── WiFi ──────────────────────────────────────────────────
    WiFi.begin(SSID, PASSWORD);
    Serial.printf("[WiFi] Connexion à %s", SSID);
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connecté — IP : %s\n",
                      WiFi.localIP().toString().c_str());
        Serial.printf("[CAM] Stream : http://%s:81/stream\n",
                      WiFi.localIP().toString().c_str());
        // 3 blinks lents = OK
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_STATUS_PIN, LOW);  delay(200);
            digitalWrite(LED_STATUS_PIN, HIGH); delay(200);
        }
    } else {
        Serial.println("\n[WiFi] ÉCHEC connexion");
        // Blink SOS
        for (int i = 0; i < 6; i++) {
            digitalWrite(LED_STATUS_PIN, LOW);  delay(50);
            digitalWrite(LED_STATUS_PIN, HIGH); delay(50);
        }
    }

    startCameraServer();
    Serial.println("[BOOT] Prêt.");
}

// ─── Loop ────────────────────────────────────────────────────
// Le streaming est géré par les tâches internes du httpd ESP-IDF.
// La loop n'a rien à faire — on peut y ajouter un watchdog si besoin.
void loop() {
    delay(10000);
}