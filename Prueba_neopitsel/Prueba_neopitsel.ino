#include <Adafruit_NeoPixel.h>

#define PIN        6          // pin al que se conecta la tira de leds
#define NUMPIXELS  60         // numero de leds en la tira

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); // inicializa la tira

void setup() {
  pixels.begin(); 
  pixels.setBrightness(50); // brillo (0-255)
}

void loop() {

  for(int i=0; i<NUMPIXELS; i++) { 
    pixels.setPixelColor(i, pixels.Color(255, 0, 0)); // rojo (R, G, B)
  }
  pixels.show(); // muestra los cambios en la tira
}
