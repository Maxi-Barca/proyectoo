#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ANCHO_PANTALLA 128
#define ALTO_PANTALLA 64   


Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, -1);

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22); 
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("No se encontró pantalla SSD1306"));
    for(;;); 
  }

  display.clearDisplay();        // limpia buffer
  display.setTextSize(2);        // tamaño de texto
  display.setTextColor(SSD1306_WHITE); // color (blanco)
  display.setCursor(0,0);        // posicion inicial
  display.println("Hola Mundo!");
  display.display();             // muestra en pantalla
}

void loop() {
}
