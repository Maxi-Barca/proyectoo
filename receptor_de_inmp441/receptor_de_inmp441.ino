#include <esp_now.h>
#include <WiFi.h>
#include <string.h> // por seguridad para memcpy

// Estructura de mensaje
typedef struct struct_message {
  int counter;
} struct_message;

struct_message incomingData;

// Callback cuando llegan datos
void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  if(incomingData.counter = 1){
    Serial.println("prender Led");
  }
}

void setup() {
  Serial.begin(115200);

  // Poner WiFi en modo estación
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  // Registrar callback de recepción
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // El receptor solo espera mensajes
}
