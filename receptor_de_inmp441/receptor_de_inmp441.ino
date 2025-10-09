#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>

#define MOTOR 33
#define PIN_NEOPIXEL 4     // Pin seguro para ESP32
#define NUMPIXELS 10       // Número de LEDs para test

// Estructura de mensaje
typedef struct struct_message {
  int counter;
} struct_message;

struct_message incomingData;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  if (incomingData.counter == 1) {
    Serial.println("Prender Led y motor");

    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0)); // rojo
    }
    pixels.show();

    digitalWrite(MOTOR, HIGH);
  }

  if (incomingData.counter == 0) {
    Serial.println("Apagar Led y motor");

    // Apagar NeoPixels
    pixels.clear();
    pixels.show();

    digitalWrite(MOTOR, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pinMode(MOTOR, OUTPUT);

  pixels.begin();
  pixels.setBrightness(50);
  pixels.show(); 

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  // Registrar callback de recepción
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Setup listo, esperando mensajes...");
}

void loop() {
  
}
