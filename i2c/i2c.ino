#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // SDA=21, SCL=22 para ESP32

  Serial.println("\nEscaneando I2C...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print("Error desconocido en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No se encontraron dispositivos I2C\n");
  } else {
    Serial.println("Escaneo finalizado\n");
  }

  delay(2000); // Espera 2 segundos y vuelve a escanear
}
