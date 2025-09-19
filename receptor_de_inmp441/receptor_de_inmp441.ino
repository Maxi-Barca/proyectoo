#include <esp_now.h>
#include <WiFi.h>
#include <string.h> // por seguridad para memcpy
#include <Adafruit_NeoPixel.h>

#define PIN        6          // pin al que se conecta la tira de leds
#define NUMPIXELS  80         // numero de leds en la tira

#define MOTOR 33

// Estructura de mensaje
typedef struct struct_message {
  int counter;
} struct_message;

struct_message incomingData;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); // inicializa la tira

// Callback cuando llegan datos
void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  if (incomingData.counter == 1) {
    Serial.println("prender Led");
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0)); // rojo (R, G, B)
    }
    pixels.show();
    digitalWrite(MOTOR, HIGH);
  }
  if (incomingData.counter == 0) {
    Serial.println("apagar Led");
    pixels.clear();
    pixels.show();
    digitalWrite(MOTOR, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // Poner WiFi en modo estación
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pinMode(MOTOR, OUTPUT);

  pixels.begin();
  pixels.setBrightness(50); // brillo (0-255)

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  // Registrar callback de recepción
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {


}
