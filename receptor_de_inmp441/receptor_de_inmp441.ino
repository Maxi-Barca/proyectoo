#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ANCHO_PANTALLA 128
#define ALTO_PANTALLA 64

Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, -1);

#define MOTOR 33
#define PIN_NEOPIXEL 4
#define NUMPIXELS 10

// Estructura de mensaje
typedef struct struct_message {
  int counter;
} struct_message;

struct_message incomingData;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool alarmaActiva = false; 
unsigned long ultimoCambio = 0;
bool estadoLed = false;

void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  if (incomingData.counter == 1) {
    Serial.println("Activar alarma (motor + titileo)");
    alarmaActiva = true;

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Alarma sonando");
    display.display();

    digitalWrite(MOTOR, HIGH);
  }

  if (incomingData.counter == 0) {
    Serial.println("Desactivar alarma");

    alarmaActiva = false;
    pixels.clear();
    pixels.show();

    display.clearDisplay();
    display.display();

    digitalWrite(MOTOR, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se encontró pantalla SSD1306"));
    for (;;);
  }

  pinMode(MOTOR, OUTPUT);

  pixels.begin();
  pixels.setBrightness(50);
  pixels.show();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  // Registrar callback de recepción
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Setup listo, esperando mensajes...");
}

void loop() {
  if (alarmaActiva) {
    unsigned long ahora = millis();
    if (ahora - ultimoCambio >= 500) {
      ultimoCambio = ahora;
      estadoLed = !estadoLed;

      if (estadoLed) {
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(255, 0, 0));
        }
      } else {
        pixels.clear();
      }
      pixels.show();
    }
  }
}
