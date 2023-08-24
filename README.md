# Proyecto1-Digital2
#Ernesto Chavez - 21441

//Librerías

#include <Arduino.h>

// Variables Globales

#define ADC_VREF_mV    3300.0 // en mV

#define ADC_RESOLUTION 4096.0

#define PIN_LM35       34 // ESP32 pin GPIO34 (ADC6) conectado al LM35

#define BUTTON_PIN     13 // ESP32 pin GPIO13 conectado al botón

volatile bool buttonPressed = false;

// Prototipo de funciones

// ISR - Vector de interrupción

void IRAM_ATTR handleButtonPress() {

  buttonPressed = true;
  
}

// Funcion setup (inicial)

void setup() {

  Serial.begin(9600);
  
  // Configurar el pin del botón como entrada y habilitar la interrupción
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
  
}

// Loop principal

void loop() {

  if (buttonPressed) {
  
    // Lee el valor ADC del sensor 
    int adcVal = analogRead(PIN_LM35);
    // Convierte el valor ADC a milivoltios
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
    // Convierte el voltaje a temperatura en °C
    float tempC = milliVolt / 5;  
    
    // Imprime los valores de temperatura en el monitor serial
    Serial.print("Temperatura: ");
    Serial.print(tempC);   // Imprime la temperatura en °C
    Serial.println("°C");
    
    buttonPressed = false; // Resetea la bandera para la próxima lectura
  }
  
  delay(500);  // Pequeño retardo para evitar el rebote del botón
}

