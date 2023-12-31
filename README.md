//Universidad del Valle de Guatemala
//Electrónica digital 2
//Sección 10
//Ernesto Salvador Chavez González
//Carné 21441


//Librerías
#include <Arduino.h>
#include "config.h"


//Canal de Adafruit al que se envían los datos de temperatura

AdafruitIO_Feed *tempcanal = io.feed("Proyecto 1");

// Variables para el sensor LM35

#define ADC_VREF_mV 3300.0 // en mV
#define ADC_RESOLUTION 4096.0
#define PIN_LM35 34 // ESP32 pin GPIO34 (ADC6) conectado al LM35
volatile float tempC;

// Configuración del PWM de las leds y del servo

#define pwmChannel 0
#define ledRChannel 1
#define ledGChannel 2
#define ledYChannel 3
#define freqPWM 50
#define freqPWM1 1000
#define freqPWM2 500
#define resolution 16

// Pines de los LEDs

#define pinLedR 14
#define pinLedG 13
#define pinLedY 12

// Pin del servo

int servoPin = 3;

// Tiempos en microsegundos para las posiciones del servo. Estos dan los movimientos según el ancho del pulso PWM.

int Ogrados = 500;
int MedioGrados = 1200;
int Fullgrados = 2000;

// Definiciones para los segmentos y sus pines

#define SEG_A 4
#define SEG_B 5
#define SEG_C 16
#define SEG_D 17
#define SEG_E 18
#define SEG_F 19
#define SEG_G 21
#define SEG_H 0  


// Definiciones para los transistores y sus pines

#define TRANS0 25
#define TRANS1 26
#define TRANS2 27

// Pin del botón
#define BUTTON_PIN 33


volatile bool readTemperature = true; // Lee la temperatura en la primera iteración

// Variables para almacenar la última temperatura
int lastIntPart = 0;
int lastDecPart = 0;

//Interrupción para cambiar el valor de readTemperature para señalar que es necesario leer la temperatura.
void IRAM_ATTR isr() {
  readTemperature = true;
}

//Configuración del PWM para el servo y para cada led.
void configurarPWM() {
  ledcSetup(ledRChannel, freqPWM1, resolution);
  ledcSetup(ledGChannel, freqPWM2, resolution);
  ledcSetup(ledYChannel, freqPWM, resolution);
  ledcAttachPin(pinLedR, ledRChannel);
  ledcAttachPin(pinLedG, ledGChannel);
  ledcAttachPin(pinLedY, ledYChannel);
  ledcSetup(pwmChannel, freqPWM, resolution);
  ledcAttachPin(servoPin, pwmChannel);
}

//Configuración del display y de sus respectivos transistores y segmentos.
void displayDigit(int digit, int display) {
  // Apagar todos los transistores
  digitalWrite(TRANS0, LOW);
  digitalWrite(TRANS1, LOW);
  digitalWrite(TRANS2, LOW);

  // Apagar el segmento H
  digitalWrite(SEG_H, HIGH); // Suponiendo que es un display de ánodo común

  // Tabla de verdad para los segmentos que sirve para saber qué segmentos encender para mostrar un número específico en los displays. 
  int segmentos[10][7] = {
    {LOW, LOW, LOW, LOW, LOW, LOW, HIGH},
    {HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH},
    {LOW, LOW, HIGH, LOW, LOW, HIGH, LOW},
    {LOW, LOW, LOW, LOW, HIGH, HIGH, LOW},
    {HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW},
    {LOW, HIGH, LOW, LOW, HIGH, LOW, LOW},
    {LOW, HIGH, LOW, LOW, LOW, LOW, LOW},
    {LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH},
    {LOW, LOW, LOW, LOW, LOW, LOW, LOW},
    {LOW, LOW, LOW, LOW, HIGH, LOW, LOW}
  };

  digitalWrite(SEG_A, segmentos[digit][0]);
  digitalWrite(SEG_B, segmentos[digit][1]);
  digitalWrite(SEG_C, segmentos[digit][2]);
  digitalWrite(SEG_D, segmentos[digit][3]);
  digitalWrite(SEG_E, segmentos[digit][4]);
  digitalWrite(SEG_F, segmentos[digit][5]);
  digitalWrite(SEG_G, segmentos[digit][6]);

  if (display == 0) {
    digitalWrite(TRANS0, HIGH);
  } else if (display == 1) {
    digitalWrite(TRANS1, HIGH);
    digitalWrite(SEG_H, LOW); // Encender el segmento H para el display de las unidades
  } else if (display == 2) {
    digitalWrite(TRANS2, HIGH);
  }

  delay(5);  // Este delay puede ajustarse para optimizar la velocidad de actualización
}

//SETUP

//declaración de cada elemento como input/output
void setup() {
  Serial.begin(9600);
  configurarPWM();
  pinMode(SEG_A, OUTPUT);
  pinMode(SEG_B, OUTPUT);
  pinMode(SEG_C, OUTPUT);
  pinMode(SEG_D, OUTPUT);
  pinMode(SEG_E, OUTPUT);
  pinMode(SEG_F, OUTPUT);
  pinMode(SEG_G, OUTPUT);
  pinMode(SEG_H, OUTPUT);  // Configuración de pin para el segmento H
  pinMode(TRANS0, OUTPUT);
  pinMode(TRANS1, OUTPUT);
  pinMode(TRANS2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr, RISING);
  Serial.println("Setup completo");
}

//LOOP PRINCIPAL


void loop() {
  if (readTemperature) //Detecta cuando el botón se presiona
  {
    int adcVal = analogRead(PIN_LM35);
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
    float tempC = milliVolt / 5.0; //convierte la señal ADC en valores de temperatura. 
    Serial.print("Temperatura: ");
    Serial.print(tempC, 1);
    Serial.println("°C");
    lastIntPart = (int)tempC;
    lastDecPart = (int)((tempC - lastIntPart) * 10); 
    io.run(); //Envía los datos al Adafruit
    Serial.print("enviando -->");
    Serial.println(tempC);
    tempcanal->save(tempC);
  

  // Código para LEDs y servo según la temperatura registrada cuando el botón se presiona
    int servoPulseWidth;
    if (tempC < 37) {
      servoPulseWidth = Ogrados;
      ledcWrite(ledGChannel, 255);
      ledcWrite(ledRChannel, 0);
      ledcWrite(ledYChannel, 0);
    } else if (tempC >= 37 && tempC < 37.5) {
      servoPulseWidth = MedioGrados;
      ledcWrite(ledYChannel, 255);
      ledcWrite(ledGChannel, 0);
      ledcWrite(ledRChannel, 0);
    } else if (tempC >= 37.5) {
      servoPulseWidth = Fullgrados;
      ledcWrite(ledRChannel, 255);
      ledcWrite(ledGChannel, 0);
      ledcWrite(ledYChannel, 0);
    }
    int dutyCycle = (servoPulseWidth * pow(2, resolution)) / 20000;
    ledcWrite(pwmChannel, dutyCycle);
    readTemperature = false;
  }

  // Mostrar temperatura en los displays
  displayDigit(lastDecPart, 0);
  displayDigit(lastIntPart % 10, 1);
  displayDigit(lastIntPart / 10, 2);
}


