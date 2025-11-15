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

bool mostrarMensajePendiente = false;
unsigned long tiempoFinAlarma = 0;
unsigned long tiempoInicioMensaje = 0;
bool mensajeEnPantalla = false; 



void OLED_task(void *pvParameters) {
  while (true) {

    if (mostrarMensajePendiente) {
      if (!mensajeEnPantalla) {
        if (millis() - tiempoFinAlarma >= 1000) {

          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 0);
          display.println("El timbre sono");
          display.display();

          tiempoInicioMensaje = millis();
          mensajeEnPantalla = true;
        }
      } 
      else {
        if (millis() - tiempoInicioMensaje >= 3000) {

          display.clearDisplay();
          display.display();

          mostrarMensajePendiente = false;
          mensajeEnPantalla = false;
        }
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}


void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  if (incomingData.counter == 1) {
    Serial.println("Activar alarma");

    alarmaActiva = true;
    mostrarMensajePendiente = false;
    mensajeEnPantalla = false;

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

    tiempoFinAlarma = millis();
    mostrarMensajePendiente = true;
  }
}


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(200);                    // tiempo para que la pantalla energice  

  Wire.begin(21, 22);
  delay(100);

  // Reset manual
  Wire.beginTransmission(0x3C);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(20);

  // Inicializar OLED (dos intentos)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED no detectada (1er intento)");
  }
  delay(50);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED no detectada (2do intento)");
  }

  display.clearDisplay();
  display.display();

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

  xTaskCreate(OLED_task, "OLED Task", 4096, NULL, 1, NULL);

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