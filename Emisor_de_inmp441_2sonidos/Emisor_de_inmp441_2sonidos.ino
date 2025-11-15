#include <Arduino.h>
#include "arduinoFFT.h"
#include <driver/i2s.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14

// Dirección MAC del receptor
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0x34, 0x8A, 0x04};

// Estructura de mensaje
typedef struct struct_message {
  int counter;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// Callback envío
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Estado del último envío: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

// FFT
const uint16_t samples = 64;
const double samplingFrequency = 16000;

double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

unsigned long millisInicio_1 = 0;
unsigned long millisInicio_2 = 0;

bool enviando_1 = false;
bool enviando_2 = false;

// -----------------------------------------------------------
// SETUP
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  Serial.println("Ready");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Registrar peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error agregando peer");
    return;
  }

  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
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
}

// -----------------------------------------------------------
// LOOP
// -----------------------------------------------------------
void loop() {
  unsigned long millisActual = millis();

  // --- Lectura cada 1 segundo ---
  static unsigned long ultimaLectura = 0;
  if (millisActual - ultimaLectura >= 1000) {
    ultimaLectura = millisActual;

    // Leer micrófono
    int32_t buffer[samples];
    size_t bytes_read;

    i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

    for (uint16_t i = 0; i < samples; i++) {
      vReal[i] = (double)buffer[i];
      vImag[i] = 0.0;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    double picoFrecuencia = FFT.majorPeak();

    // --------------------------
    // DETECTOR #1 → Enviar "1"
    // --------------------------
    if (picoFrecuencia >= 7000.00 && picoFrecuencia <= 7150.00) {
      if (!enviando_1) {
        enviando_1 = true;
        millisInicio_1 = millisActual;
        Serial.println("Detectado timbre → comenzar envío 1");
      }
    }

    // --------------------------
    // DETECTOR #2 → Enviar "2"
    // --------------------------
    if (picoFrecuencia >= 5750.00 && picoFrecuencia <= 5950.00) {
      if (!enviando_2) {
        enviando_2 = true;
        millisInicio_2 = millisActual;
        Serial.println("Detectado señal 5800Hz → comenzar envío 2");
      }
    }
  }

  // ==========================================================
  // ENVÍO CONTINUADO DE "1"
  // ==========================================================
  if (enviando_1 && (millisActual - millisInicio_1 < 5000)) {
    myData.counter = 1;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    Serial.println("Enviando: 1 (timbre)");
  }

  if (enviando_1 && (millisActual - millisInicio_1 >= 5000)) {
    enviando_1 = false;
    Serial.println("Fin envío 1");
    myData.counter = 0;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }

  // ==========================================================
  // ENVÍO CONTINUADO DE "2"
  // ==========================================================
  if (enviando_2 && (millisActual - millisInicio_2 < 5000)) {
    myData.counter = 2;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    Serial.println("Enviando: 2 (5800Hz)");
  }

  if (enviando_2 && (millisActual - millisInicio_2 >= 5000)) {
    enviando_2 = false;
    Serial.println("Fin envío 2");
    myData.counter = 0;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }
}


// -----------------------------------------------------------
// Imprimir FFT (no usado, lo dejo por si lo necesitás)
// -----------------------------------------------------------
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;
    switch (scaleType) {
      case 0: abscissa = (i * 1.0); break;
      case 1: abscissa = ((i * 1.0) / samplingFrequency); break;
      case 2: abscissa = ((i * 1.0 * samplingFrequency) / samples); break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == 2) Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}