#include <driver/i2s.h>

#define I2S_WS 15    // Word Select (también llamado LRCLK): define si el canal es izquierdo o derecho
#define I2S_SD 32    // Serial Data: pin por donde el micrófono envía los datos de audio
#define I2S_SCK 14   // Serial Clock (Bit Clock o BCLK): pin del reloj que sincroniza los datos

void setup() {
  Serial.begin(115200);

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
  int32_t buffer[64];     // Arreglo para almacenar 64 muestras de 32 bits cada una
  size_t bytes_read;      // Variable para saber cuántos bytes se leyeron realmente

  i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);  //esta funcion lee datos desde el micrófono, y los guarda en buffer
                                                                             //sizeof(buffer) = 64 × 4 bytes = 256 bytes 
                                                                             //portMAX_DELAY = espera indefinidamente hasta que haya suficientes datos
  Serial.println(buffer[0]);  // Imprime la primera muestra
  delay(100);
}
