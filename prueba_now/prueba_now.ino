#include <WiFi.h>
#include <esp_now.h>

typedef struct struct_message {
  int counter;
} struct_message;

struct_message incomingData;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataRaw, int len) {

  Serial.print("Paquete recibido. len = ");
  Serial.println(len);

  if (len != sizeof(struct_message)) {
    Serial.println("Tamaño incorrecto -> se descarta");
    return;
  }

  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  Serial.print("Counter: ");
  Serial.println(incomingData.counter);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);       // Fuerza canal estable (muy importante)

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {}
