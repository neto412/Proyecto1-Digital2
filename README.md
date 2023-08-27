# Proyecto1-Digital2
#Ernesto Chavez - 21441

#include <Arduino.h>

// Variables para el sensor LM35
#define ADC_VREF_mV    3300.0 // en mV
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       34 // ESP32 pin GPIO34 (ADC6) conectado al LM35

// Configuración del PWM
#define pwmChannel 0
#define ledRChannel 1
#define ledGChannel 2
#define ledYChannel 3

#define freqPWM 50 // Frecuencia para servo, típicamente 50 Hz (20 ms)
#define freqPWM1 1000
#define freqPWM2 500
#define resolution 16 // Mayor resolución para mayor precisión

// Pines de los LEDs
#define pinLedR 14
#define pinLedG 13
#define pinLedY 12

// Pines del servo
int servoPin = 3;

// Tiempos en microsegundos para las posiciones del servo
int Ogrados = 500;   // 1 ms pulse width
int MedioGrados = 1200; // 1.5 ms pulse width
int Fullgrados = 2000; // 2 ms pulse width

void configurarPWM() {
    // Configuración de PWM para los LEDs
    ledcSetup(ledRChannel, freqPWM1, resolution);
    ledcSetup(ledGChannel, freqPWM2, resolution);
    ledcSetup(ledYChannel, freqPWM, resolution);

    ledcAttachPin(pinLedR, ledRChannel);
    ledcAttachPin(pinLedG, ledGChannel);
    ledcAttachPin(pinLedY, ledYChannel);

    // Configuración de PWM para el servo
    ledcSetup(pwmChannel, freqPWM, resolution);
    ledcAttachPin(servoPin, pwmChannel);
}

void setup() {
    Serial.begin(9600);
    configurarPWM();
    Serial.println("Setup completo"); // Mensaje para confirmar que el setup se ha completado
}

void loop() {
    int adcVal = analogRead(PIN_LM35);
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
    float tempC = milliVolt / 5;

    // Imprimir la temperatura
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.println("°C");

    int servoPulseWidth;

    if (tempC < 37) {
        servoPulseWidth = Ogrados;
        ledcWrite(ledGChannel, 255);
        ledcWrite(ledRChannel, 0);
        ledcWrite(ledYChannel, 0);
        Serial.println("LED verde y servo a 0 grados");
    } else if (tempC >= 37 && tempC < 37.5) {
        servoPulseWidth = MedioGrados;
        ledcWrite(ledYChannel, 255);
        ledcWrite(ledGChannel, 0);
        ledcWrite(ledRChannel, 0);
        Serial.println("LED amarilla y servo a 90 grados");
    } else if (tempC >= 37.5) {
        servoPulseWidth = Fullgrados;
        ledcWrite(ledRChannel, 255);
        ledcWrite(ledGChannel, 0);
        ledcWrite(ledYChannel, 0);
        Serial.println("LED roja y servo a 180 grados");
    }

    // Calcular el valor del dutycycle en función de la resolución
    int dutyCycle = (servoPulseWidth * pow(2, resolution)) / 20000;
    ledcWrite(pwmChannel, dutyCycle);

    delay(500); // antirebote
}
