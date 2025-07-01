#include <Arduino.h>
#include "arduinoFFT.h"
#include <driver/i2s.h>

#define I2S_WS 15    // Word Select (también llamado LRCLK): define si el canal es izquierdo o derecho
#define I2S_SD 32    // Serial Data: pin por donde el micrófono envía los datos de audio
#define I2S_SCK 14   // Serial Clock (Bit Clock o BCLK): pin del reloj que sincroniza los datos

/*
  These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 16000;
const uint8_t amplitude = 100;

const double samplingFrequency2 = 16000;

/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

double vReal2[samples];
double vImag2[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
ArduinoFFT<double> FFT2 = ArduinoFFT<double>(vReal2, vImag2, samples, samplingFrequency2);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  Serial.begin(115200);
  while (!Serial) {};
  Serial.println("Ready");

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

void loop()
{
  int32_t buffer[samples];     // buffer para las muestras de 32 bits
  size_t bytes_read;

  i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
  /* Build raw data */
  double ratio = twoPi * signalFrequency / samplingFrequency; // Fraction of a complete cycle stored at each sample (in radians)
  for (uint16_t i = 0; i < samples; i++)
  {
    vReal[i] = int8_t(amplitude * sin(i * ratio) / 2.0);/* Build data with positive and negative values*/
    //vReal[i] = uint8_t((amplitude * (sin(i * ratio) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }

  for (uint16_t i = 0; i < samples; i++) {
    // Normalizá la señal de 32 bits a valores entre -1.0 y 1.0 (opcional para FFT)
    vReal[i] = (double)buffer[i];// / 2147483648.0;  // dividir por 2^31, para llevar esos numeros grandes a un rango estandar que FFT maneja mejor. Tmb transformo el valor  a double
    vImag[i] = 0.0;
  }

  
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  FFT.complexToMagnitude(); /* Compute magnitudes */

  FFT2.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT2.compute(FFTDirection::Forward);
  FFT2.complexToMagnitude();

  double fftMagnitudes[samples / 2];  // Solo se usa la mitad de la FFT
  double fftMagnitudes2[samples / 2];
  
  for (int i = 0; i < samples / 2; i++) {
    fftMagnitudes[i] = vReal[i];
  }
  
  for (int i = 0; i < samples / 2; i++) {
    fftMagnitudes2[i] = vReal2[i];
  }
  
  //Serial.println("magnitud guardada:");
  //PrintVector(fftMagnitudes, samples/2, SCL_FREQUENCY);

  //Serial.println("magnitud2 guardada:");
  //PrintVector(fftMagnitudes2, samples/2, SCL_FREQUENCY);
  
  double picoFrecuencia = FFT.majorPeak();
  Serial.print("Frecuencia: ");
  Serial.print(picoFrecuencia, 6);
  Serial.print("Hz");

  double picoFrecuencia2 = FFT2.majorPeak();
  Serial.print("Frecuencia: ");
  Serial.print(picoFrecuencia2, 6);
  Serial.print("Hz");

  double mseComparacion = calcularMSE(fftMagnitudes,fftMagnitudes2 , samples/2);
  Serial.println(mseComparacion);
  if (mseComparacion < 1000.0) {
  Serial.println("Se aproximan bastante");
}
  delay(2000); /* Repeat after delay */
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

double calcularMSE(const double* a, const double* b, int size) {  //comparamos error cuadratico medio entre señales (con el fft aplicado)
    double suma = 0.0;
    for (int i = 0; i < size; ++i) {
        double diff = a[i] - b[i];
        suma += diff * diff;
    }
    return suma / size;    //cuanto mayor MSE, más distintos son los vectores
}
