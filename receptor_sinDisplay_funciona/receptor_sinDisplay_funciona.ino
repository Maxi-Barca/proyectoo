#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
*/


#define ANCHO_PANTALLA 128
#define ALTO_PANTALLA 64

/*
// Objeto del display (DESACTIVADO)
Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, -1);
*/

#define MOTOR2 26
#define MOTOR 25
#define PIN_NEOPIXEL 16
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

// Se usaba para controlar el texto en pantalla, ahora desactivado
// String ultimaPantalla = "";


/*
// -----------------------------------------------------------------------------
//                         TAREA OLED - (DESACTIVADA)
// -----------------------------------------------------------------------------
void OLED_task(void *pvParameters) {
  while (true) {
    if (alarmaActiva) {
      if (ultimaPantalla != "Alarma sonando") {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Alarma sonando");
        display.display();
        ultimaPantalla = "Alarma sonando";
      }
    } else {
      if (ultimaPantalla != "Alarma desactivada") {
        display.clearDisplay();
        display.display();
        ultimaPantalla = "Alarma desactivada";
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
*/


void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  if (incomingData.counter == 1) {
    Serial.println("Activar alarma (motor + titileo)");
    alarmaActiva = true;

    digitalWrite(MOTOR, HIGH);
    digitalWrite(MOTOR2, HIGH);
  }

  if (incomingData.counter == 0) {
    Serial.println("Desactivar alarma");

    alarmaActiva = false;
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

  //Wire.begin(21, 22);  // Pines I2C, aunque el display está desactivado
  delay(500);

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());


  /*
  // ---------------------------------------------------------
  // INICIALIZACIÓN DEL DISPLAY (DESACTIVADA)
  // ---------------------------------------------------------
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se encontró pantalla SSD1306"));
    for (;;);
  }
  */


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

  /*
  // Tarea del display desactivada
  xTaskCreate(OLED_task, "OLED Task", 4096, NULL, 1, NULL);
  */

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
