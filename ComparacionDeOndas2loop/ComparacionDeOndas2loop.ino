#include <Arduino.h>
#include "arduinoFFT.h"
#include <driver/i2s.h>

#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14

const uint16_t samples = 64;
const double samplingFrequency = 16000;
const double signalFrequency = 1000;
const uint8_t amplitude = 100;

double vRealMic[samples];
double vImagMic[samples];

double vRealGen[samples];
double vImagGen[samples];

ArduinoFFT<double> FFTmic = ArduinoFFT<double>(vRealMic, vImagMic, samples, samplingFrequency);
ArduinoFFT<double> FFTgen = ArduinoFFT<double>(vRealGen, vImagGen, samples, samplingFrequency);

QueueHandle_t colaMuestras;

void leerAudioTask(void *parameter);
void procesarAudioTask(void *parameter);
double calcularMSE(const double* a, const double* b, int size);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // El ESP32 actúa como maestro y solo recibe datos (RX)
    .sample_rate = 16000,                              // Toma 16000 muestras por segundo (frecuencia de muestreo)
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,      // Cada muestra ocupa 32 bits (4 bytes)
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       // Solo se escucha el canal izquierdo (porque L/R del micrófono va a GND)
    .communication_format = I2S_COMM_FORMAT_I2S,       // Usa el protocolo estándar I2S
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,          // Nivel de interrupción (1 es suficiente)
    .dma_buf_count = 8,                                // Usa 8 buffers DMA
    .dma_buf_len = 64,                                 // Cada buffer tiene 64 muestras
    .use_apll = false,                                 // No usa APLL (reloj más preciso, pero opcional)
    .tx_desc_auto_clear = false,                       // Solo relevante para salida (TX), no afecta aquí
    .fixed_mclk = 0                                    // No se usa reloj maestro externo
  };
  
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,               // Pin del reloj de bits
    .ws_io_num = I2S_WS,                 // Pin de selección de palabra (izq/der)
    .data_out_num = I2S_PIN_NO_CHANGE,   // No se usa salida (TX), así que no se configura
    .data_in_num = I2S_SD                // Pin de entrada de datos (audio digital del micrófono)
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL); // Instala el driver de I2S en el canal 0 con esa configuración
  i2s_set_pin(I2S_NUM_0, &pin_config);                // Aplica la configuración de pines al canal I2S 0
  i2s_zero_dma_buffer(I2S_NUM_0);                     // Limpia los buffers de DMA antes de comenzar


  colaMuestras = xQueueCreate(4, samples * sizeof(int32_t));

  xTaskCreatePinnedToCore(leerAudioTask, "LeerAudio", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(procesarAudioTask, "ProcesarAudio", 8192, NULL, 1, NULL, 1);
}

void loop() {
}

void leerAudioTask(void *parameter) {
  int32_t buffer[samples];
  size_t bytes_read;

  while (true) {
    if (i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY) == ESP_OK
        && bytes_read == sizeof(buffer)) {
      xQueueSend(colaMuestras, buffer, portMAX_DELAY);
    }
  }
}

void procesarAudioTask(void *parameter) {
  int32_t buffer[samples];

  while (true) {
    if (xQueueReceive(colaMuestras, buffer, portMAX_DELAY)) {

      for (int i = 0; i < samples; i++) {
        vRealMic[i] = (double)buffer[i] / 2147483648.0; // divido por 2^31 si querés normalizar
        vImagMic[i] = 0.0;
      }

      FFTmic.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFTmic.compute(FFTDirection::Forward);
      FFTmic.complexToMagnitude();


      double ratio = 2 * PI * signalFrequency / samplingFrequency;
      for (int i = 0; i < samples; i++) {
        vRealGen[i] = amplitude * sin(i * ratio);
        vImagGen[i] = 0.0;
      }

      FFTgen.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFTgen.compute(FFTDirection::Forward);
      FFTgen.complexToMagnitude();

      // Extraer magnitudes
      double magMic[samples / 2];
      double magGen[samples / 2];

      for (int i = 0; i < samples / 2; i++) {
        magMic[i] = vRealMic[i];
        magGen[i] = vRealGen[i];
      }

      // Comparar
      double mse = calcularMSE(magMic, magGen, samples / 2);
      Serial.print("MSE: ");
      Serial.println(mse, 2);

      if (mse < 124000.0) {  
        Serial.println("Las señales se parecen");
      } else {
        Serial.println("Son distintas");
      }

      delay(500);  
    }
  }
}

double calcularMSE(const double* a, const double* b, int size) {
  double suma = 0.0;
  for (int i = 0; i < size; ++i) {
    double diff = a[i] - b[i];
    suma += diff * diff;
  }
  return suma / size;
}
