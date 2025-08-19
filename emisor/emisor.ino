#include <esp_now.h>
#include <WiFi.h>

// Dirección MAC del receptor
uint8_t broadcastAddress[] = {0xD4, 0x8A, 0xFC, 0xCE, 0xE1, 0xD0};

// Estructura de mensaje
typedef struct struct_message {
  int counter;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

// Callback de confirmación de envío
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Estado del último envío: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

void setup() {
  Serial.begin(115200);

  // Poner WiFi en modo estación (obligatorio para ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  // Registrar callback de envío
  esp_now_register_send_cb(OnDataSent);

  // Registrar al receptor como peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error agregando peer");
    return;
  }
}

void loop() {
  static int counter = 0;
  myData.counter = counter++;

  // Enviar datos
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Enviado correctamente");
  } else {
    Serial.println("Error enviando");
  }

  delay(2000);
}
