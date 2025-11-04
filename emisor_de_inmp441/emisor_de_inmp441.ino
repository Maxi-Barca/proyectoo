#include <Arduino.h>
#include "arduinoFFT.h"
#include <driver/i2s.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2S_WS 15    // Word Select (también llamado LRCLK): define si el canal es izquierdo o derecho
#define I2S_SD 32    // Serial Data: pin por donde el micrófono envía los datos de audio
#define I2S_SCK 14   // Serial Clock (Bit Clock o BCLK): pin del reloj que sincroniza los datos

// Dirección MAC del receptor
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0x37, 0x69, 0x80};

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


const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 16000;


double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

unsigned long millisInicio = 0;
bool enviando = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {};
  Serial.println("Ready");

  WiFi.mode(WIFI_STA);

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
}

void loop() {
  unsigned long millisActual = millis();

  // --- Control de lectura cada 1 segundo ---
  static unsigned long ultimaLectura = 0;
  if (millisActual - ultimaLectura >= 1000) {
    ultimaLectura = millisActual;

    // Leer datos del micrófono
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

    // Detectar timbre
    if (picoFrecuencia >= 7000.00 && picoFrecuencia <= 7150.00) {
      if (!enviando) {
        enviando = true;
        millisInicio = millisActual;
      }
    }
  }

  // --- Control de envío por 5 segundos ---
  if (enviando && (millisActual - millisInicio < 5000)) {
    myData.counter = 1;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.println("Enviando: timbre detectado");
  }

  if (enviando && (millisActual - millisInicio >= 5000)) {
    enviando = false;
    Serial.println("Fin del envío de timbre");
    myData.counter = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
