#include <Arduino.h>
#include "arduinoFFT.h"
#include <driver/i2s.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14

// MAC del receptor (tal como la pasaste)
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0x34, 0x8A, 0x04};

typedef struct struct_message {
  int counter;
} struct_message;
struct_message myData;

// FFT / I2S
const uint16_t samples = 64;
const double samplingFrequency = 16000.0;
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// Estado de envío
unsigned long millisInicio = 0;
bool enviando = false;

// callback de envío (core 3.x)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Estado del último envío: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Emisor arrancando...");

  // --- WiFi / ESP-NOW configuración robusta ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);             // borra credenciales, evita auto connect
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  WiFi.setChannel(1);                // forzar canal 1 (debe coincidir con receptor)

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    // continue: podremos intentar agregar peers / enviar y ver fallos
  }

  esp_now_register_send_cb(OnDataSent);

  // Registrar peer (estructura limpia)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (!esp_now_is_peer_exist(broadcastAddress)) {
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("Peer agregado OK (emisor)");
    } else {
      Serial.println("Error agregando peer (emisor) - seguiré intentando al enviar");
    }
  } else {
    Serial.println("Peer ya existía");
  }

  // --- I2S init para INMP441 ---
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = (int)samplingFrequency,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("Emisor listo");
}

void loop() {
  unsigned long millisActual = millis();

  // --- Lectura I2S y FFT cada 1s ---
  static unsigned long ultimaLectura = 0;
  if (millisActual - ultimaLectura >= 1000) {
    ultimaLectura = millisActual;

    int32_t buffer[samples];
    size_t bytes_read = 0;
    esp_err_t r = i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    if (r != ESP_OK) {
      Serial.print("i2s_read error: ");
      Serial.println(r);
    } else {
      for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = (double)buffer[i];
        vImag[i] = 0.0;
      }
      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();
      double picoFrecuencia = FFT.majorPeak();
      // ajuste de umbral/ventana si necesitás otro rango
      if (picoFrecuencia >= 7000.0 && picoFrecuencia <= 7150.0) {
        if (!enviando) {
          enviando = true;
          millisInicio = millisActual;
          Serial.println("Inicio deteccion timbre -> comenzamos a enviar");
        }
      }
    }
  }

  // --- Envío protegido con rate-limit y reintentos ---
  static unsigned long ultimaEnvio = 0;
  const unsigned long intervaloEnvio = 200; // ms entre envíos
  if (enviando && (millisActual - millisInicio < 5000)) {
    if (millisActual - ultimaEnvio >= intervaloEnvio) {
      ultimaEnvio = millisActual;
      myData.counter = 1;

      // Si peer no existe, intentamos agregarlo
      if (!esp_now_is_peer_exist(broadcastAddress)) {
        esp_now_peer_info_t p = {};
        memcpy(p.peer_addr, broadcastAddress, 6);
        p.channel = 1;
        p.encrypt = false;
        p.ifidx = WIFI_IF_STA;
        esp_now_add_peer(&p);
        delay(10);
      }

      esp_err_t res = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      Serial.println("Enviando: timbre detectado");
      if (res != ESP_OK) {
        Serial.print("esp_now_send fallo, err=");
        Serial.println(res);
        // backoff pequeño para no bombardear el buffer
        delay(50);
      }
    }
  }

  if (enviando && (millisActual - millisInicio >= 5000)) {
    enviando = false;
    Serial.println("Fin del envío de timbre (envío 0 once)");
    myData.counter = 0;

    if (!esp_now_is_peer_exist(broadcastAddress)) {
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, broadcastAddress, 6);
      p.channel = 1;
      p.encrypt = false;
      p.ifidx = WIFI_IF_STA;
      esp_now_add_peer(&p);
      delay(10);
    }

    esp_err_t res0 = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (res0 != ESP_OK) {
      Serial.print("esp_now_send fallo al enviar 0, err=");
      Serial.println(res0);
    }
    // asegurar ultimoEnvio
    ultimaEnvio = millisActual;
  }

  // yield para mantener sistema responsivo
  yield();
}
