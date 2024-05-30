#include "Wire.h"
#include <MechaQMC5883.h>

 

MechaQMC5883 magnetometro;

int x, y, z;
float declinacion = 1.467; // No parece usarse, pero lo incluyo por si acaso
const int pinSensor = 2;
int estadoSensor;

void setup() {
    Serial.begin(9600);
    Serial.println("Inicializando Magnetometro y configurando pin del sensor...");

    Wire.begin();
    magnetometro.init();

    pinMode(pinSensor, INPUT_PULLUP);
}

void loop() {
    //Obtenemos del magnetometro las componentes del campo magnético
    magnetometro.read(&x,&y,&z);

    float heading = atan2(y, x);
  
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.026;
    heading += declinationAngle;
    
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
      
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
    
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI; 

    // Enviar el rumbo a través del puerto serie
    Serial.print("Heading: ");
    Serial.println(headingDegrees);

    // Leer el estado del sensor y enviarlo a través del puerto serie
    estadoSensor = digitalRead(pinSensor);
    Serial.print("Correa: ");
    Serial.println(estadoSensor);

    delay(200); // Ajusta este tiempo si es necesario
}