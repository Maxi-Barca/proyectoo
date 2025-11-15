#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
//#include <Wire.h>

#define MOTOR2 26
#define MOTOR 25
#define PIN_NEOPIXEL 16
#define NUMPIXELS 10

typedef struct struct_message {
  int counter;
} struct_message;

struct_message incomingData;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool alarmaActiva = false;
unsigned long ultimoCambio = 0;
bool estadoLed = false;

int tipoAlarma = 0;   // 1 = rojo, 2 = azul

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  // ---------------------------------------------
  //           MENSAJE 1  → ALARMA ROJA
  // ---------------------------------------------
  if (incomingData.counter == 1) {
    Serial.println("Activar alarma ROJA");
    alarmaActiva = true;
    tipoAlarma = 1;

    digitalWrite(MOTOR, HIGH);
    digitalWrite(MOTOR2, HIGH);
  }

  // ---------------------------------------------
  //           MENSAJE 2  → ALARMA AZUL
  // ---------------------------------------------
  if (incomingData.counter == 2) {
    Serial.println("Activar alarma AZUL");
    alarmaActiva = true;
    tipoAlarma = 2;

    digitalWrite(MOTOR, HIGH);
    digitalWrite(MOTOR2, HIGH);
  }

  // ---------------------------------------------
  //               MENSAJE 0 → APAGAR TODO
  // ---------------------------------------------
  if (incomingData.counter == 0) {
    Serial.println("Desactivar alarma");

    alarmaActiva = false;
    tipoAlarma = 0;

    pixels.clear();
    pixels.show();

    digitalWrite(MOTOR, LOW);
    digitalWrite(MOTOR2, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //Wire.begin(21, 22);

  pinMode(MOTOR, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pixels.begin();
  pixels.setBrightness(50);
  pixels.show();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

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
          
          if (tipoAlarma == 1) {
            pixels.setPixelColor(i, pixels.Color(0, 0, 255));
          }
          
          if (tipoAlarma == 2) {
            pixels.setPixelColor(i, pixels.Color(255, 0, 0));
          }
        }
      } else {
        pixels.clear();
      }
      pixels.show();
    }
  }
}