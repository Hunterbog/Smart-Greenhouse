#include <DHT.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>
#include <Wire.h>
#include <BH1750.h>
/*************VARIABILE PT TIMP START**************/
uint16_t year = 2000;
uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
/*************VARIABILE PT TIMP SFARSIT**************/
uint8_t TemperatureDayLowerLimit;
uint8_t TemperatureDayUpperLimit;
uint8_t TemperatureNightLowerLimit;
uint8_t TemperatureNightUpperLimit;

uint8_t HumidityDayLowerLimit;
uint8_t HumidityDayUpperLimit;
uint8_t HumidityNightLowerLimit;
uint8_t HumidityNightUpperLimit;

uint8_t TemperatureLowerLimit;
uint8_t TemperatureUpperLimit;
uint8_t HumidityLowerLimit;
uint8_t HumidityUpperLimit;
/*************VARIABILE PT PARAMETRI START**************/


const uint8_t comfortRange = 6;
float hysteresisBuffer = 1.0;
unsigned long windowIsOpenedTime = 0;
/*************VARIABILE PT PAARAMETRI SFARSIT**************/

// constante globale
/*********************CONSTANTE GLOBALE INCEPUT*********************/
#define FRAME_START 0xAA
#define STD_ON 1
#define STD_OFF 0
#define DEBUG STD_OFF  // compile switch
#define VALID 0
#define RELEY_ON 0
#define RELEY_OFF 1
#define SEND_FRAME(pkt) writeFrameWithRetry((uint8_t *)&(pkt), sizeof(pkt))
#define DEBUG STD_ON
#define MAX_RETRY 3
#define TIMEOUT_MS 10
#define ACK 0x06
#define NAK 0x15
#define LATURA 15.5
#define LUNGIME 15.5
#define INALTIME 11

#define ACT_BUFF_ID 'A'
#define ACT_BUFF_SIZE 12

#define SENS_BUFF_SIZE 20
#define SENS_BUFF_ID  'S'

#define VOLUM_SUFICIENT_APA 1.0
#define CONTROL_SERA 1
#define POMPA1 2
#define POMPA2 3
#define POMPA3 4
#define VENTILATOR1 5
#define VENTILATOR2 6
#define INCALZIRE1 7
#define INCALZIRE2 8
#define UMIDIFICATOR 9
#define GEAM 10
#define LUMINA 11

#define TIME_1_HOUR (60UL * 60UL * 1000UL)  // 1 hour = 3,600,000 ms
#define TIME_3_HOUR (3 * TIME_1_HOUR)
#define TIME_10_MINUTES (10UL * 60UL * 1000UL)  // 10 minutes = 600,000 ms
#define TIME_10_SECONDS (10000UL)
#define MIN_IDLE_TIME TIME_10_SECONDS
unsigned long lastWaterTime1 = 0;
unsigned long lastWaterTime2 = 0;
unsigned long lastWaterTime3 = 0;


#define RX_BUF_MAX 32
const char ACTUATOR_PKT_ID = 'A';
const char HARVEST_PKT_ID = 'H';
//CRC-CCITT
const int poly = 0x1021;  // CRC-CCITT polynomial

#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define TX_BUFF_SIZE 28
#define NOTALLOWED 0
/*********************CONSTANTE GLOBALE SFARSIT*********************/


typedef struct
{
  uint8_t name;
  uint8_t value;
} ActuatorState;

ActuatorState actuator;

/********VARIABILE PT FSM SERIAL INCEPUT***********/
enum RxState { WAIT_START,
               WAIT_LEN,
               WAIT_PAYLOAD };
static RxState rxState = WAIT_START;
static uint8_t rxBuf[RX_BUF_MAX];
static uint8_t packetLength = 0;
static uint8_t index = 0;
/********VARIABILE PT FSM SERIAL SFARSIT***********/

#define setbit(data, bit) ((data) |= (1 << (bit)))

// Definire constante pentru timpi
#define DHT22Update 2000         // Interval citire DHT22 (ms)
#define RTCsHumidityUpdate 1000  // Interval citire umiditate sol (ms)
#define LightSensorUpdate 5000   // Interval citire senzor lumina (ms)
#define UpdateTxBuffer 200       // Interval trimitere date serial
#define RainSensorRead 1000
#define WaterLvlHumUpdate 100
#define WaterLevelRead 200
// Definire pinii pentru senzori și actuatori

/*========================Resurse Haardware=======================*/
#define DHTPIN A7
#define DHTTYPE DHT22
#define moisturePin A0
const int pinHigrometru1 = A6;  // intrare analogica senzor umiditate parcela 1
const int pinHigrometru2 = A7;  // intrare analogica senzor umiditate parcela 2
const int pinHigrometru3 = A8;  // intrare analogica senzor umiditate parcela 3
const int pinUmiLvl = A9;       // intrare analogica senzor umiditate parcela 4
#define lightSensorPin A5
#define trigPin 52
#define echoPin 50
const int pumpPin1 = 43;
const int pumpPin2 = 53;
const int pumpPin3 = 51;
const int pumpPin4 = 45;
const int heatingPin_S1b = 3;  //iesire PWM
const int heatingPin_S1a = 26;
const int heatingPin_S2b = 4;  //iesire PWM
const int heatingPin_S2a = 27;
const int rainPin = 47;
#define humidifierPin 9        // Pinul pentru umidificator (și sistemul de mist)
#define ventilatorLeftPin 10   // Pinul pentru ventilatoare
#define growLightPin 12        // Pinul pentru lumina de creștere
#define ventilatorRightPin 11  // Pinul pentru ventilatoare
// Definire pinii și constante pentru servo (geam de aerisire)
#define SERVO_PIN 2       // Pinul pentru servo
#define SERVO_CLOSED 10   // Poziția închisă a servo-ului
#define SERVO_OPENED 140  // Poziția deschisă a servo-ului
#define SERVO_DELAY 15    // Întârzierea între incrementări (ms)
#define LOW_LVL_HUM 100
#define HIGH_LVL_HUM 500
#define LOW_LIGHT_LEVEL 50
#define OPTIMAL_LIGHT_LEVEL 450
int SOIL_MOISTURE_DRY_THRESHOLD = 800;  // Solul e considerat uscat peste acest prag
int SOIL_MOISTURE_OPTIMAL_MIN = 301;    // Începutul zonei de umiditate optimă
int SOIL_MOISTURE_OPTIMAL_MAX = 799;    // Sfârșitul zonei de umiditate optimă
int SOIL_MOISTURE_WET_THRESHOLD = 300;  // Sub acest prag, solul e considerat prea ud

/*========================Sfarsit Resurse Haardware=======================*/


/********************variabile pt control a serei******************/

enum CommandAction {
  IDLE,
  STOP,
  START,
  RUNNING,
};

CommandAction command = IDLE;

// Enum pentru moduri de funcționare
enum ControlMode {
  AUTOMATED,
  MANUAL
};
ControlMode mode = AUTOMATED;

/********************variabile pt control a serei******************/

////Variabile pt control Weather system
volatile bool requireWindow = false;
typedef enum {
  NORMAL_TEMP,
  HIGH_TEMP,
  LOW_TEMP,
} Temperature;
unsigned long IdleTime = 0;
unsigned long FansTime = 0;
bool FansState = false;
Temperature temperature = NORMAL_TEMP;
// Prag și timp pentru lumina naturală
#define LIGHT_THRESHOLD 500                       // Valoarea minimă considerată "lumina suficientă"
#define LOW_LIGHT_THRESHOLD 100                   // Prag pentru lumină scăzută (innorat)
#define LIGHT_REQUIRED_HOURS 8                    // Orele necesare de lumină naturală
#define MAX_ARTIFICIAL_LIGHT_DURATION 36000000UL  // 10 ore în ms
// Definire DHT
DHT dht(DHTPIN, DHTTYPE);
uint8_t FirstReadSensors = 0;
// Declarație variabile pentru senzori și timp
/*===========================SENZORI-INCEPUT===========================*/
unsigned int soilMoisture1, soilMoisture2, soilMoisture3;
float dhtTemp, dhtHumidity;
int lightLevel = 0;
uint16_t waterLvlHum = 800;
bool allowHumToWork = true;
float waterVolume = 0.0;
BH1750 lightMeter;
/*===========================SENZORI-SFARSIT===========================*/

unsigned long pompaTime1 = 0;
unsigned long pompaTime2 = 0;
unsigned long pompaTime3 = 0;  // Timpul la pornirea pompei
unsigned long pompaTime4 = 0;
// Variabile pentru controlul luminii de creștere (artificiale)
bool growLightState = false;                 // Starea luminii artificiale
unsigned long artificialLightStartTime = 0;  // Timpul de pornire a luminii artificiale
unsigned char Tx_BufferSensors[SENS_BUFF_SIZE] = { 0 };
unsigned char Tx_BufferActuators[ACT_BUFF_SIZE] = { 0 };
// Variabile pentru controlul servo (geam de aerisire)
Servo windowServo;
volatile int servoPos = SERVO_CLOSED;
unsigned long prevServoTime = 0;
bool isDeatached = false;
volatile uint8_t allowWindow = 0xF;
// Declarație funcții
void readLightSensor();
void readMoistureSensor();
void readTemperatureHumiditySensor();
float calculeazaVolumApaInLitri();
float Distance();
void applicationLogic(ControlMode mode);
void controlGrowLight();  // Include logica luminii artificiale
void controlWindowServo();
void controlWaterSystem();
void controlWeatherSystem();
void receiveSerialBuffer();
void updateSerialBufferActuators();
void allowHumidifierToWork();
uint16_t computeCRC(uint8_t *data, size_t length);

/******************************Configurare Timer4*********************************************
Vreau sa generez o intrerupere la fiecare 20ms => Fint = 1/20ms = 1/0.02s = 200Hz
OCR = (Fclk/P*Fint) - 1 => OCR = (16Mhz/P*200Hz) - 1 => pt P = 8 avem OCR = 10000 - 1 = 9999
**********************************************************************************************/

void setupTimer4() {
  cli();       // Dezactivează întreruperile
  TCCR4A = 0;  // Resetează registrul Timer4
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = 19999;            // Configurat pentru contorizare
  TCCR4B |= (1 << WGM12);   // Mod CTC
  TCCR4B |= (1 << CS41);    // Prescaler 8
  TIMSK4 |= (1 << OCIE4A);  // Permite întreruperea la comparare
  sei();                    // Activează întreruperile
}



// Variabile pentru citirea senzorilor la intervale specifice
unsigned long previousDHT22Millis = 0;
unsigned long previousMoistureMillis = 0;
unsigned long previousLightMillis = 0;
unsigned long prevUpdateTxBufferActuators = 0;
unsigned long prevUpdateTxBufferSensors = 0;
unsigned long prevRainSensorRead = 0;
unsigned long prevWaterLevel = 0;
unsigned long previousWaterLvlHumMillis = 0;
unsigned long timpAnte;
void setup() {
  timpAnte = millis();
  // Configurare pini pentru senzori și actuatori
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pinHigrometru1, INPUT);
  pinMode(pinHigrometru2, INPUT);
  pinMode(pinHigrometru3, INPUT);
  pinMode(pinUmiLvl, INPUT);
  Wire.begin();
  lightMeter.begin();
  pinMode(pumpPin1, OUTPUT);  // Set mode for pump 1
  pinMode(pumpPin2, OUTPUT);  // Set mode for pump 2
  pinMode(pumpPin3, OUTPUT);  // Set mode for pump 3
  pinMode(pumpPin4, OUTPUT);  // Set mode for pump 4
  pinMode(heatingPin_S2a, OUTPUT);
  pinMode(heatingPin_S2b, OUTPUT);
  pinMode(heatingPin_S1b, OUTPUT);
  pinMode(heatingPin_S1a, OUTPUT);
  digitalWrite(heatingPin_S1a, RELAY_OFF);
  digitalWrite(heatingPin_S2a, RELAY_OFF);
  pinMode(humidifierPin, OUTPUT);
  pinMode(ventilatorLeftPin, OUTPUT);   // Set mode for left fan
  pinMode(ventilatorRightPin, OUTPUT);  // Set mode for right fan
  pinMode(growLightPin, OUTPUT);
  pinMode(rainPin, INPUT);
  Serial.begin(115200);
  dht.begin();
  // Configurare Timer4
  setupTimer4();
  // Inițializări pentru lumina de creștere (artificială)
  growLightState = false;
  // Inițializare servo (geam de aerisire)
  windowServo.attach(SERVO_PIN);
  servoPos = SERVO_CLOSED;
  windowServo.write(servoPos);
  prevServoTime = millis();
}

void allowWindowToWork() {
  allowWindow = (allowWindow << 1) | digitalRead(rainPin);
#if DEBUG == STD_ON
  Serial.println("Ploaie daca vad 0:");
  Serial.println((allowWindow & 0xF));
#endif
}

void loop() {
  updateTime();
  while (Serial.available() > 0) {
    uint8_t octet = Serial.read();
    uartByteReceived(octet);
  }
  switch (command) {
    case IDLE:
      break;
    case STOP:
      digitalWrite(pumpPin1, LOW);
      digitalWrite(pumpPin2, LOW);
      digitalWrite(pumpPin3, LOW);
      analogWrite(heatingPin_S1b, LOW);
      analogWrite(heatingPin_S2b, LOW);
      digitalWrite(heatingPin_S1a, RELAY_OFF);
      digitalWrite(heatingPin_S2a, RELAY_OFF);
      digitalWrite(humidifierPin, RELAY_OFF);
      analogWrite(ventilatorLeftPin, LOW);
      analogWrite(ventilatorRightPin, LOW);
      digitalWrite(growLightPin, LOW);
      digitalWrite(rainPin, LOW);
      TIMSK4 &= ~(1 << OCIE4A);
      ADCSRA &= ~(1 << ADEN);  // Disable ADC to save power
      command = IDLE;
      //SERA nu functioneaza
      break;
    case START:
      mode = AUTOMATED;
      TIMSK4 |= (1 << OCIE4A);
      ADCSRA |= (1 << ADEN);
      command = RUNNING;
      break;
    case RUNNING:
      timpAnte = millis();
      // Citire senzor DHT22 la intervale specifice
      if (millis() - previousDHT22Millis >= DHT22Update) {
        previousDHT22Millis = millis();
        readTemperatureHumiditySensor();
      }

      // Citire senzor umiditate sol la intervale specifice
      if (millis() - previousMoistureMillis >= RTCsHumidityUpdate) {
        previousMoistureMillis = millis();
        readMoistureSensor();
      }

      // Citire senzor lumină la intervale specifice
      if (millis() - previousLightMillis >= LightSensorUpdate) {
        previousLightMillis = millis();
        readLightSensor();
      }

      if (millis() - previousWaterLvlHumMillis >= WaterLvlHumUpdate) {
        previousWaterLvlHumMillis = millis();
        readWaterLvlHum();
      }

      if (millis() - prevRainSensorRead >= RainSensorRead) {
        prevRainSensorRead = millis();
        allowWindowToWork();
      }

      if ((millis() - prevWaterLevel >= WaterLevelRead) && (digitalRead(pumpPin1) == LOW && digitalRead(pumpPin2) == LOW && digitalRead(pumpPin3) == LOW)) {
        prevWaterLevel = millis();
        readWaterLevel();
      }

      if (millis() - prevUpdateTxBufferActuators >= UpdateTxBuffer) {
        prevUpdateTxBufferActuators = millis();
        updateSerialBufferActuators();
      }
      if (millis() - prevUpdateTxBufferSensors >= 50 * UpdateTxBuffer) {
        prevUpdateTxBufferSensors = millis();
        updateSerialBufferSensors();
      }
      applicationLogic(mode);
      break;

    default:
      break;
  }
}

void readLightSensor() {
  const int numSamples = 12;
  static int samples[numSamples] = { 0 };
  static int index = 0;

  // Take a new reading
  samples[index++] = lightMeter.readLightLevel();
  if (index == numSamples) {
    index = 0;
    setbit(FirstReadSensors, 0);
    lightLevel = getMedian(samples, numSamples);
#if DEBUG == STD_ON
    Serial.print("Light: ");
    Serial.println(lightLevel);
#endif
  }
}

void readWaterLvlHum() {
  const int numSamples = 4;
  static int samples[numSamples] = { 0 };
  static int index = 0;

  // Take a new reading
  samples[index++] = analogRead(pinUmiLvl);
  if (index == numSamples) {
    index = 0;
    waterLvlHum = getMedian(samples, numSamples);

    // Serial.print("Water Level Humf: ");
    // Serial.println(waterLvlHum);
  }
}

void readMoistureSensor() {
  const int numSamples = 60;
  const int numSensors = 3;  // 4 moisture sensors
  static int samples[numSensors][numSamples];
  static int index = 0;
  static bool bufferReady = false;

  // Read all sensors into the buffer
  samples[0][index] = analogRead(pinHigrometru1);
  samples[1][index] = analogRead(pinHigrometru2);
  samples[2][index] = analogRead(pinHigrometru3);

  index++;

  if (index >= numSamples) {
    bufferReady = true;
    index = 0;
  }
  if (bufferReady) {
    setbit(FirstReadSensors, 1);

    soilMoisture1 = getMedian(samples[0], numSamples);
    soilMoisture2 = getMedian(samples[1], numSamples);
    soilMoisture3 = getMedian(samples[2], numSamples);

    bufferReady = false;
#if DEBUG == STD_ON
    Serial.println("Soil Moistures: ");
    Serial.println(soilMoisture1);
    Serial.println(", ");
    Serial.println(soilMoisture2);
    Serial.println(", ");
    Serial.println(soilMoisture3);
#endif
  }
}

void readTemperatureHumiditySensor() {
  const int numSamples = 30;
  static float tempSamples[numSamples], humiditySamples[numSamples];
  static int index = 0;
  static bool bufferReady = false;
  tempSamples[index] = dht.readTemperature();
  humiditySamples[index] = dht.readHumidity();
  index++;

  // Process when buffer is full
  if (index >= numSamples) {
    bufferReady = true;
    index = 0;
  }

  // Compute medians and print
  if (bufferReady) {
    setbit(FirstReadSensors, 2);
    dhtTemp = getMedianFloat(tempSamples, numSamples);
    dhtHumidity = getMedianFloat(humiditySamples, numSamples);
    bufferReady = false;
#if DEBUG == STD_ON
    Serial.print("Temp: ");
    Serial.print(dhtTemp);
    Serial.print("C, Humidity: ");
    Serial.print(dhtHumidity);
    Serial.println("%");
#endif
  }
}

void readWaterLevel() {
  const int numSamples = 10;
  static float samples[numSamples] = { 0 };
  static int index = 0;

  // Take a new reading
  samples[index++] = calculeazaVolumApaInLitri();
  if (index == numSamples) {
    index = 0;
    setbit(FirstReadSensors, 3);
    waterVolume = getMedianFloat(samples, numSamples);
#if DEBUG == STD_ON
    Serial.println("Volum:");
    Serial.println(waterVolume);
#endif
  }
}

float calculeazaVolumApaInLitri() {
  float distanta = Distance();
  distanta = INALTIME - distanta;
  if (distanta <= 0.5) {
    distanta = 0;
  }
  float volumCm3 = LATURA * LUNGIME * distanta;
  //Serial.println(volumCm3);
  return volumCm3 / 1000.0;
}

float Distance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2.0;
}

// Logica pentru lumina artificială
// - Daca nivelul de lumină este sub LOW_LIGHT_THRESHOLD, se pornește lumina artificială
// - Se oprește dacă se îmbunătățește nivelul sau se depășește durata maximă
void controlGrowLight() {
  // Interzicem lumina artificială între 22:00 și 6:00
  if (hour >= 22 || hour < 6) {
    if (growLightState) {
      digitalWrite(growLightPin, LOW);
      growLightState = false;
    }
    return;
  }

  if (lightLevel < LOW_LIGHT_THRESHOLD) {
    if (!growLightState) {
      digitalWrite(growLightPin, HIGH);
      growLightState = true;
      artificialLightStartTime = millis();
      //Serial.println("Lumina artificială pornită (lipsă lumină naturală).");
    }
  } else {
    // Oprim lumina artificială dacă nivelul se îmbunătățește sau se atinge durata maximă
    if (growLightState) {
      if (lightLevel >= LOW_LIGHT_THRESHOLD || (millis() - artificialLightStartTime) >= MAX_ARTIFICIAL_LIGHT_DURATION) {
        digitalWrite(growLightPin, LOW);
        growLightState = false;
        //Serial.println("Lumina artificială oprită (lumina naturală a revenit sau durata maximă atinsă).");
      }
    }
  }
}

// Funcția controlWaterSystem() gestionează udarea plantelor.
// Modificările adăugate:
//
// - Se pornește pompa doar dacă senzorii indică condiții potrivite și există suficientă apă în rezervor
void controlWaterSystem() {
  // Pornirea pompelor dacă sunt îndeplinite toate condițiile + idle time
  if (dhtHumidity <= HumidityUpperLimit && lightLevel < OPTIMAL_LIGHT_LEVEL && waterVolume >= VOLUM_SUFICIENT_APA) {

    if (digitalRead(pumpPin1) == LOW && soilMoisture1 >= SOIL_MOISTURE_DRY_THRESHOLD && pompaTime1 == 0 && (millis() - lastWaterTime1 >= MIN_IDLE_TIME)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 1 pornită");
#endif
      digitalWrite(pumpPin1, HIGH);
      pompaTime1 = millis();
    }

    if (digitalRead(pumpPin2) == LOW && soilMoisture2 >= SOIL_MOISTURE_DRY_THRESHOLD && pompaTime2 == 0 && (millis() - lastWaterTime2 >= MIN_IDLE_TIME)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 2 pornită");
#endif
      digitalWrite(pumpPin2, HIGH);
      pompaTime2 = millis();
    }

    if (digitalRead(pumpPin3) == LOW && soilMoisture3 >= SOIL_MOISTURE_DRY_THRESHOLD && pompaTime3 == 0 && (millis() - lastWaterTime3 >= MIN_IDLE_TIME)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 3 pornită");
#endif
      digitalWrite(pumpPin3, HIGH);
      pompaTime3 = millis();
    }
  }

  // Oprirea pompelor după 10 secunde sau dacă solul devine prea ud
  if (digitalRead(pumpPin1) == HIGH) {
    if (soilMoisture1 <= SOIL_MOISTURE_WET_THRESHOLD || (millis() - pompaTime1 > TIME_10_SECONDS)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 1 oprită");
#endif
      digitalWrite(pumpPin1, LOW);
      pompaTime1 = 0;
      lastWaterTime1 = millis();  // marcăm ultima udare
    }
  }

  if (digitalRead(pumpPin2) == HIGH) {
    if (soilMoisture2 <= SOIL_MOISTURE_WET_THRESHOLD || (millis() - pompaTime2 > TIME_10_SECONDS)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 2 oprită");
#endif
      digitalWrite(pumpPin2, LOW);
      pompaTime2 = 0;
      lastWaterTime2 = millis();
    }
  }

  if (digitalRead(pumpPin3) == HIGH) {
    if (soilMoisture3 <= SOIL_MOISTURE_WET_THRESHOLD || (millis() - pompaTime3 > TIME_10_SECONDS)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 3 oprită");
#endif
      digitalWrite(pumpPin3, LOW);
      pompaTime3 = 0;
      lastWaterTime3 = millis();
    }
  }
}


// Funcția applicationLogic() gestionează logica principală:
// - Udarea plantelor (inclusiv controlul apei)
// - Activarea forțată a ventilatoarelor și a sistemului de mist la temperaturi ridicate
// - Controlul luminii artificiale
void applicationLogic(ControlMode mode) {
  switch (mode) {
    case AUTOMATED:
      allowHumidifierToWork();
      // timpAnte = millis();
      if (FirstReadSensors == 0b1111) {
        controlWeatherSystem();
        //Serial.println("Durata sa fac operatie de controlVentilationSystem " + String(millis() - timpAnte));
        // timpAnte = millis();
        //Controlul sistemului de lumina artificiala
        controlGrowLight();
        //Serial.println("Durata sa fac operatie de controlGrowLight " + String(millis() - timpAnte));
        // Controlul sistemului de udare
        //timpAnte = millis();
        controlWaterSystem();
      }
      // Serial.println("Durata sa fac operatie de controlWaterSystem " + String(millis() - timpAnte));
      //Controlul sistemului de ventilatie
      break;

    case MANUAL:
      switch (actuator.name) {
        case POMPA1:
          digitalWrite(pumpPin1, actuator.value);
          break;

        case POMPA2:
          digitalWrite(pumpPin2, actuator.value);
          break;

        case POMPA3:
          digitalWrite(pumpPin3, actuator.value);
          break;

        case VENTILATOR1:
          analogWrite(ventilatorLeftPin, actuator.value);
          break;

        case VENTILATOR2:
          analogWrite(ventilatorRightPin, actuator.value);
          break;

        case INCALZIRE1:
          digitalWrite(heatingPin_S1b, RELAY_ON);
          analogWrite(heatingPin_S1a, actuator.value);
          break;

        case INCALZIRE2:
          digitalWrite(heatingPin_S2b, RELAY_ON);
          analogWrite(heatingPin_S2a, actuator.value);
          break;

        case UMIDIFICATOR:
          digitalWrite(humidifierPin, actuator.value);
          break;

        case GEAM:
          requireWindow = true;
          break;

        case LUMINA:
          digitalWrite(growLightPin, actuator.value);
          break;
        default:
          break;
      }
      actuator.name = actuator.value = 0;
      break;
  }
}

ISR(TIMER4_COMPA_vect) {
  if (requireWindow && (allowWindow & 0xF)) {
    if (servoPos < SERVO_OPENED) {
      windowServo.write(++servoPos);
    }
  } else {
    if (!(requireWindow || ((allowWindow & 0xF) == NOTALLOWED))) {
      if (servoPos > SERVO_CLOSED) {
        windowServo.write(--servoPos);
      }
    }
  }
}


int getMedian(int arr[], int size) {
  int sorted[size];
  memcpy(sorted, arr, sizeof(sorted));
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (sorted[i] > sorted[j]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  return sorted[size / 2];
}

float getMedianFloat(float arr[], int size) {
  float sorted[size];
  memcpy(sorted, arr, sizeof(sorted));
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (sorted[i] > sorted[j]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  return sorted[size / 2];
}

uint16_t computeCRC(uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= (data[i] << 8);

    for (int j = 0; j < 8; ++j) {
      if (crc >> 15 & 1) {  // sau crc & 0x8000
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void controlWeatherSystem() {
  bool isDay = (lightLevel > LOW_LIGHT_LEVEL);

  if (isDay) {
    TemperatureLowerLimit = TemperatureDayLowerLimit;
    TemperatureUpperLimit = TemperatureDayUpperLimit;
    HumidityLowerLimit = HumidityDayLowerLimit;
    HumidityUpperLimit = HumidityDayLowerLimit;
  } else {
    TemperatureLowerLimit = TemperatureNightLowerLimit;
    TemperatureUpperLimit = TemperatureNightUpperLimit;
    HumidityLowerLimit = HumidityNightLowerLimit;
    HumidityUpperLimit = HumidityNightLowerLimit;
  }
  if ((dhtTemp > TemperatureLowerLimit) && (dhtTemp < TemperatureUpperLimit))
    temperature = NORMAL_TEMP;
  else if (dhtTemp >= TemperatureUpperLimit + hysteresisBuffer)
    temperature = HIGH_TEMP;
  else if (dhtTemp <= TemperatureLowerLimit - hysteresisBuffer)
    temperature = LOW_TEMP;
  switch (temperature) {
    case HIGH_TEMP:
      if (dhtHumidity < (float)HumidityLowerLimit - hysteresisBuffer) {  //Nivelul de umidate este MIC
        // Hot and dry
        //Serial.println("//Nivelul de umidate este MIC");
        requireWindow = false;
        if (allowHumToWork) { digitalWrite(humidifierPin, RELAY_ON); }
        analogWrite(ventilatorLeftPin, 80);
        analogWrite(ventilatorRightPin, 80);
        FansTime = millis();
        FansState = true;
        break;
      } else if (dhtHumidity > (float)HumidityUpperLimit + hysteresisBuffer) {  //Nivelul de umidate este MARE
                                                                                // Serial.println("//Nivelul de umidate este MARE");
        requireWindow = true;
        windowIsOpenedTime = millis();
        digitalWrite(humidifierPin, RELAY_OFF);
        analogWrite(ventilatorLeftPin, 255);
        analogWrite(ventilatorRightPin, 255);
        FansTime = millis();
        FansState = true;
        break;
      } else {  //Nivelul de umidaitate este OPTIM -- impart in doua daca trece peste valoarea optima sting umidifcatorul, daca e sub el il las aprins ca sa racesc camera
                //Serial.println("Nivelul de umidate este Mediu");
        if (dhtHumidity < (HumidityLowerLimit + HumidityUpperLimit) / 2.0)
          if (allowHumToWork) {
            digitalWrite(humidifierPin, RELAY_ON);
          } else if (dhtHumidity > (HumidityLowerLimit + HumidityUpperLimit) / 2.0 + hysteresisBuffer)
            digitalWrite(humidifierPin, RELAY_OFF);

        if (millis() - windowIsOpenedTime > TIME_10_MINUTES && requireWindow == true) {  // las geamul deschis timp de 10 minute in caz ca nivelul de umiditate este in parametrii normali
          requireWindow = false;
          IdleTime = millis();
        } else if (millis() - IdleTime >= TIME_3_HOUR && requireWindow == false) {
          windowIsOpenedTime = millis();
          IdleTime = 0;
          requireWindow = true;
        }
        //Ventilatoarele le las mereu pornite
        analogWrite(ventilatorLeftPin, 160);
        analogWrite(ventilatorRightPin, 160);
        FansTime = millis();
        FansState = true;
      }
      break;

    case NORMAL_TEMP:
      {
        ControlHeatingSystem(STD_OFF);
        if (dhtHumidity < HumidityLowerLimit - hysteresisBuffer) {  //Nivelul de umidate este MIC
          requireWindow = false;
          if (allowHumToWork) { digitalWrite(humidifierPin, RELAY_ON); }
          analogWrite(ventilatorLeftPin, 0);
          analogWrite(ventilatorRightPin, 0);
          IdleTime = millis();
        } else if (dhtHumidity > HumidityUpperLimit + hysteresisBuffer) {  //Nivelul de umidate este MARE
          //digitalWrite(humidifierPin, RELAY_OFF);
          analogWrite(ventilatorLeftPin, 200);
          analogWrite(ventilatorRightPin, 200);
          IdleTime = 0;
          FansTime = windowIsOpenedTime = millis();
          FansState = requireWindow = true;
        } else {  //Nivelul de umiditate este OPTIM -- deschid geamul la diferite intervale de timp si dau drumul la ventilatoare la diferite intervale de timp - la fiecare ora le las timp de 10 min

          if (dhtHumidity > (HumidityLowerLimit + HumidityUpperLimit) / 2.0)  // Verific nivelul de umiditate pt a controla umidificatorul
            digitalWrite(humidifierPin, RELAY_OFF);
          if ((millis() - FansTime > TIME_10_MINUTES && FansState) || ((millis() - windowIsOpenedTime > TIME_10_MINUTES && requireWindow))) {
            requireWindow = FansState = false;
            analogWrite(ventilatorLeftPin, 0);
            analogWrite(ventilatorRightPin, 0);
            FansTime = windowIsOpenedTime = 0;
            IdleTime = millis();
          }
          if (millis() - IdleTime >= TIME_1_HOUR && (!FansState || !requireWindow)) {
            requireWindow = FansState = true;
            FansTime = windowIsOpenedTime = millis();
            analogWrite(ventilatorLeftPin, 200);
            analogWrite(ventilatorRightPin, 200);
            IdleTime = 0;
          }
        }
        break;
      }
    case LOW_TEMP:
      {
        requireWindow = false;                                             //In cazul temperaturilor scazute nu deschid geamul dar aerisesc folosind ventilatoare
        ControlHeatingSystem(STD_ON);                                      // pornesc caldura
        if (dhtHumidity < (float)HumidityLowerLimit - hysteresisBuffer) {  //Nivelul de umidate este JOS
          if (allowHumToWork) { digitalWrite(humidifierPin, RELAY_ON); }
          analogWrite(ventilatorLeftPin, 0);
          analogWrite(ventilatorRightPin, 0);
          IdleTime = millis();
        } else if (dhtHumidity > (float)HumidityUpperLimit + hysteresisBuffer) {  //Nivelul de umidate este MARE
          digitalWrite(humidifierPin, RELAY_OFF);
          analogWrite(ventilatorLeftPin, 120);
          analogWrite(ventilatorRightPin, 120);
          IdleTime = 0;
          FansTime = millis();
          FansState = true;
        } else {                                 //Nivelul de umidaitate este OPTIM -- dau drumul la ventilatoare la diferite intervale de timp - la fiecare 3 ore le las timp de 10 min
          if (dhtHumidity > HumidityLowerLimit)  // Verific nivelul de umiditate pt a controla umidificatorul
            digitalWrite(humidifierPin, RELAY_OFF);
          if (millis() - FansTime > TIME_10_MINUTES && FansState) {
            FansState = false;
            FansTime = millis();
            analogWrite(ventilatorLeftPin, 0);
            analogWrite(ventilatorRightPin, 0);
            IdleTime = millis();
          }
          if (millis() - IdleTime >= TIME_3_HOUR && !FansState) {
            IdleTime = 0;
            requireWindow = true;
            FansState = true;
            FansTime = millis();
            analogWrite(ventilatorLeftPin, 100);
            analogWrite(ventilatorRightPin, 100);
          }
        }
        break;
      }
    default:
      break;
  }
}


void updateSerialBufferActuators() {
  /* -------- PWM actuators -------- */
  Tx_BufferActuators[0] = FRAME_START;
  Tx_BufferActuators[1] = ACT_BUFF_SIZE - 2;
  Tx_BufferActuators[2] = ACT_BUFF_ID;
  Tx_BufferActuators[3] = analogRead(SERVO_PIN);
  Tx_BufferActuators[4] = analogRead(ventilatorLeftPin);
  Tx_BufferActuators[5] = analogRead(ventilatorRightPin);
  Tx_BufferActuators[6] = analogRead(heatingPin_S1b);
  Tx_BufferActuators[7] = analogRead(heatingPin_S2b);

  /* -------- digital mask -------- */
  uint8_t dig = (digitalRead(pumpPin1) << 0) | (digitalRead(pumpPin2) << 1) | (digitalRead(pumpPin3) << 2) | (digitalRead(pumpPin4) << 3) | (digitalRead(humidifierPin) << 4) | (digitalRead(growLightPin) << 5) | ((!digitalRead(heatingPin_S1a)) << 6) | ((!digitalRead(heatingPin_S1a)) << 7);
  Tx_BufferActuators[8] = dig;
  Tx_BufferActuators[9] = 0;
  /* -------- CRC -------- */
  uint16_t crc = computeCRC(Tx_BufferActuators, ACT_BUFF_SIZE - 2);
  Tx_BufferActuators[10] = uint8_t(crc >> 8);
  Tx_BufferActuators[11] = uint8_t(crc & 0xFF);
  SEND_FRAME(Tx_BufferActuators);
}


void updateSerialBufferSensors() {
  /* -------- header -------- */
  Tx_BufferSensors[0] = FRAME_START;
  Tx_BufferSensors[1] = SENS_BUFF_SIZE;
  Tx_BufferSensors[2] = SENS_BUFF_ID;

  /* -------- soil moisture (3 × 16‑bit) -------- */
  Tx_BufferSensors[3] = uint8_t(soilMoisture1 >> 8);
  Tx_BufferSensors[4] = uint8_t(soilMoisture1);
  Tx_BufferSensors[5] = uint8_t(soilMoisture2 >> 8);
  Tx_BufferSensors[6] = uint8_t(soilMoisture2);
  Tx_BufferSensors[7] = uint8_t(soilMoisture3 >> 8);
  Tx_BufferSensors[8] = uint8_t(soilMoisture3);

  /* -------- water sensors -------- */
  Tx_BufferSensors[9] = uint8_t(100 - (uint32_t)waterLvlHum * 100 / 1023);

  uint16_t volX100 = uint16_t(waterVolume * 100.0f);
  Tx_BufferSensors[10] = uint8_t(waterVolume);
  Tx_BufferSensors[11] = uint8_t(volX100 % 100);

  /* -------- DHT22 -------- */
  uint16_t tX100 = int16_t(dhtTemp * 100.0);
  uint16_t hX100 = int16_t(dhtHumidity * 100.0);

  Tx_BufferSensors[12] = uint8_t(tX100 >> 8);
  Tx_BufferSensors[13] = uint8_t(tX100);
  Tx_BufferSensors[14] = uint8_t(hX100 >> 8);
  Tx_BufferSensors[15] = uint8_t(hX100);

  /* -------- light -------- */
  Tx_BufferSensors[16] = uint8_t(lightLevel >> 8);
  Tx_BufferSensors[17] = uint8_t(lightLevel);

  /* -------- CRC -------- */
  uint16_t crc = computeCRC(Tx_BufferSensors, SENS_BUFF_SIZE - 2);
  Tx_BufferSensors[18] = uint8_t(crc >> 8);
  Tx_BufferSensors[19] = uint8_t(crc & 0xFF);

  SEND_FRAME(Tx_BufferSensors);
}


void uartByteReceived(uint8_t octet) {
  switch (rxState) {
    case WAIT_START:
      if (octet == FRAME_START) {
        rxBuf[0] = octet;
        rxState = WAIT_LEN;
      }
      break;
    case WAIT_LEN:
      rxBuf[1] = octet;
      packetLength = octet;
      if (packetLength > RX_BUF_MAX) {
        rxState = WAIT_START;
        break;
      }
      index = 0;
      rxState = WAIT_PAYLOAD;
      break;


    case WAIT_PAYLOAD:
      rxBuf[2 + index++] = octet;
      if (index == packetLength - 2) {
        if (computeCRC(rxBuf, packetLength) == VALID) {
          Serial.write(ACK);
          if (rxBuf[2] == ACTUATOR_PKT_ID) {
            handleActuatorFrame(&rxBuf[3]);
          } else if (rxBuf[2] == HARVEST_PKT_ID) {
            handleHarvestFrame(&rxBuf[3]);
          }
        } else {
          Serial.write(NAK);
        }
        rxState = WAIT_START;
      }
      break;
  }
}


void handleActuatorFrame(const uint8_t *data) {
  switch (data[0]) {
    case CONTROL_SERA:
      mode = (data[0] == STD_ON) ? MANUAL : AUTOMATED;
      break;

    case POMPA1:
    case POMPA2:
    case POMPA3:
    case VENTILATOR1:
    case VENTILATOR2:
    case INCALZIRE1:
    case INCALZIRE2:
    case UMIDIFICATOR:
    case GEAM:
    case LUMINA:
      actuator.name = data[1];
      actuator.value = data[2];
      break;

    default:
      break;
  }

  memset(rxBuf, 0, RX_BUF_MAX);
}

void handleHarvestFrame(const uint8_t *data) {

  if(data[0] == 'P'){
    command = START;
  }
  else if(data[0] == 'O')
  {
    command = STOP;
  } else if (data[0] == 'R') {
    command = RUNNING;  // teoretic nu fac nimic;
  }


  TemperatureDayLowerLimit = data[1];
  TemperatureDayUpperLimit = data[2];
  HumidityDayLowerLimit = data[3];
  HumidityDayUpperLimit = data[4];
  TemperatureNightLowerLimit = data[5];
  TemperatureNightUpperLimit = data[6];
  HumidityNightLowerLimit = data[7];
  HumidityNightUpperLimit = data[8];
  year += data[9];
  month = data[10];
  day = data[11];
  hour = data[12];
  minute = data[13];
  second = data[14];

  memset(rxBuf, 0, RX_BUF_MAX);
}


void ControlHeatingSystem(uint8_t state) {
  if (state == STD_ON) {
    digitalWrite(heatingPin_S2b, RELAY_ON);
    analogWrite(heatingPin_S2a, 200);
    digitalWrite(heatingPin_S1b, RELAY_ON);
    analogWrite(heatingPin_S1a, 200);
  } else {
    digitalWrite(heatingPin_S2b, RELAY_OFF);
    analogWrite(heatingPin_S2a, 0);
    digitalWrite(heatingPin_S1b, RELAY_OFF);
    analogWrite(heatingPin_S1a, 0);
  }
}



void allowHumidifierToWork() {
  if (waterLvlHum <= LOW_LVL_HUM && waterVolume > VOLUM_SUFICIENT_APA) {
    digitalWrite(pumpPin4, HIGH);
#if DEBUG == STD_ON
    Serial.println("Pornesc pompa 4");
#endif
    allowHumToWork = false;
  } else if (waterLvlHum >= HIGH_LVL_HUM) {
    {
      //  Serial.println("Opresc pompa 4");
      digitalWrite(pumpPin4, LOW);
      allowHumToWork = true;
    }
  }
}
/**************COD CEAS******************/

int weekday(int d, int m, int y) {
  if (m < 3) {
    m += 12;
    y--;
  }
  return ((d + (13 * (m + 1)) / 5 + y + y / 4 - y / 100 + y / 400 + 6) % 7) + 1;
}

bool isLastSunday(int d, int m, int y) {
  if (weekday(d, m, y) != 7) return false;
  return (d + 7 > daysInMonth(m, y));
}

int daysInMonth(int m, int y) {
  if (m == 2)
    return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 29 : 28;
  else if (m == 4 || m == 6 || m == 9 || m == 11)
    return 30;
  else
    return 31;
}

unsigned long lastMillis;
void updateTime() {
  if (millis() - lastMillis >= 1000) {
    lastMillis += 1000;

    second++;
    if (second >= 60) {
      second = 0;
      minute++;

      if (minute >= 60) {
        minute = 0;
        hour++;

        // Incrementare zi
        if (hour >= 24) {
          hour = 0;
          day++;

          if (day > daysInMonth(month, year)) {
            day = 1;
            month++;
            if (month > 12) {
              month = 1;
              year++;
            }
          }
        }
        if (month == 3 && isLastSunday(day, month, year) && hour == 3) {
          hour = 4;
        }
        if (month == 10 && isLastSunday(day, month, year) && hour == 4) {
          hour = 3;
        }
      }
    }
  }
}
/**************COD CEAS******************/

bool writeFrameWithRetry(const uint8_t *buf, size_t len) {
  for (uint8_t attempt = 0; attempt < MAX_RETRY; ++attempt) {
    Serial.write(buf, len);

    unsigned long t = millis();
    while (millis() - t < TIMEOUT_MS) {
      if (Serial.available()) {
        uint8_t r = Serial.read();
        if (r == ACK) return true;
        if (r == NAK) break;
      }
    }
  }
  return false;
}
