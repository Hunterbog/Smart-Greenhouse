#include <DHT.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>
#include <Wire.h>
#include <BH1750.h>

#define setbit(data, bit) ((data) |= (1 << (bit)))
#define clrbit(data, bit) ((data) &= ~(1 << (bit)))
#define testbit(data, bit) (((data) >> (bit)) & 0x01)
#define SEND_FRAME(pkt) writeFrameWithRetry((uint8_t *)&(pkt), sizeof(pkt))

/*************VARIABILE PT TIMP START**************/
uint16_t year = 2000;
uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
/*************VARIABILE PT TIMP SFARSIT**************/

/*************VARIABILE PT PARAMETRI DE PRAG START**************/
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
/*************VARIABILE PT PARAMETRI DE PRAG SFARSIT**************/

/*************VARIABILE PT PARAMETRI START**************/
const uint8_t comfortRange = 6;
float hysteresisBuffer = 1.0;
unsigned long windowIsOpenedTime = 0;
/*************VARIABILE PT PAARAMETRI SFARSIT**************/


/*********************CONSTANTE GLOBALE INCEPUT*********************/
#define DHT22Update 2000             // Interval citire DHT22 (ms)
#define RTCsHumidityUpdate 1000      // Interval citire umiditate sol (ms)
#define LightSensorUpdate 5000       // Interval citire senzor lumina (ms)
#define UpdateTxBufferSensors 60000  // Interval trimitere date serial
#define UpdateTxBufferActuators 2000
#define RainSensorRead 1000
#define WaterLvlHumUpdate 200
#define WaterLevelRead 200

#define INVALID_TIMESTAMP 0xFFFFFFFFUL
#define FRAME_START 0xAA
#define STD_ON 1
#define STD_OFF 0
#define VALID 0
#define RELEY_ON 0
#define RELEY_OFF 1
#define DEBUG STD_ON
#define MAX_RETRY 3
#define TIMEOUT_MS 10
#define ACK 0x06
#define NAK 0x15
#define LATURA 15.5
#define LUNGIME 15.5
#define INALTIME 11

#define ACT_BUFF_ID 'a'
#define ACT_BUFF_SIZE 12

#define SENS_BUFF_SIZE 20
#define SENS_BUFF_ID 's'

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
#define POMPA4 12

#define TIME_1_HOUR (60UL * 1000UL)  // 1 hour = 3,600,000 ms
#define TIME_3_HOUR (3 * TIME_1_HOUR)
#define TIME_10_MINUTES (10UL * 1000UL)  // 10 minutes = 600,000 ms
#define TIME_20_MINUTES (2 * TIME_10_MINUTES)
#define TIME_10_SECONDS (10000UL)
#define TIME_5_SECONDS (5000UL)
#define MIN_IDLE_TIME TIME_10_MINUTES
#define RX_BUF_MAX 32
const char ACTUATOR_PKT_ID = 'A';
const char HARVEST_PKT_ID = 'H';
//CRC-CCITT
const int poly = 0x1021;  // CRC-CCITT polynomial

#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define TX_BUFF_SIZE 28
#define NOTALLOWED 0
#define ALLOWED 0xF
#define FANS_OFF 0
#define FANS_ON 3

#define SERVO_CLOSED 165  // Poziția închisă a servo-ului
#define SERVO_OPENED 40   // Poziția deschisă a servo-ului
#define SERVO_DELAY 15    // Întârzierea între incrementări (ms)
#define LOW_LVL_HUM 80
#define HIGH_LVL_HUM 500
#define LOW_LIGHT_LEVEL 20
#define OPTIMAL_LIGHT_LEVEL 450

const int SOIL_MOISTURE_DRY_THRESHOLD = 800;  // Solul e considerat uscat peste acest prag
const int SOIL_MOISTURE_OPTIMAL_MIN = 301;    // Începutul zonei de umiditate optimă
const int SOIL_MOISTURE_OPTIMAL_MAX = 799;    // Sfârșitul zonei de umiditate optimă
const int SOIL_MOISTURE_WET_THRESHOLD = 300;  // Sub acest prag, solul e considerat prea ud

#define HIGH_SPEED 255
#define MEDIUM_SPEED 160
#define LOW_SPEED 80
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



/*========================Resurse Haardware=======================*/
#define DHTPIN A7
#define DHTTYPE DHT22
const int pinHigrometru1 = A0;  // intrare analogica senzor umiditate parcela 1
const int pinHigrometru2 = A1;  // intrare analogica senzor umiditate parcela 2
const int pinHigrometru3 = A2;  // intrare analogica senzor umiditate parcela 3
const int pinUmiLvl = A9;       // intrare analogica senzor umiditate parcela 4
#define trigPin 52
#define echoPin 50
const int pumpPin1 = 41;
const int pumpPin2 = 53;
const int pumpPin3 = 51;
const int pumpPin4 = 39;
const int heatingPin_S1b = 3;  //iesire PWM
const int heatingPin_S1a = 27;
const int heatingPin_S2b = 4;  //iesire PWM
const int heatingPin_S2a = 25;
const int rainPin = 47;
#define humidifierPin 22       // Pinul pentru umidificator (și sistemul de mist)
#define ventilatorLeftPin 10   // Pinul pentru ventilatoare
#define growLightPin 23        // Pinul pentru lumina de creștere
#define ventilatorRightPin 11  // Pinul pentru ventilatoare
#define SERVO_PIN 2            // Pinul pentru servo

//TEMPORAR
const int red = 30;
const int blue = 31;
const int yellow = 32;
//TEMPORAR
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
enum Humidity {
  HUMIDITY_LOW,
  HUMIDITY_NORMAL,
  HUMIDITY_HIGH
};
Humidity humidity = HUMIDITY_NORMAL;
unsigned long IdleTimeFans = 0;
unsigned long IdleTimeWindow = 0;
unsigned long FansTime = 0;
unsigned char FansState = FANS_OFF;
Temperature temperature = NORMAL_TEMP;
// Prag și timp pentru lumina naturală
#define LIGHT_THRESHOLD 500                       // Valoarea minimă considerată "lumina suficientă"
#define LOW_LIGHT_THRESHOLD 100                   // Prag pentru lumină scăzută (innorat)
#define LIGHT_REQUIRED_HOURS 8                    // Orele necesare de lumină naturală
#define MAX_ARTIFICIAL_LIGHT_DURATION 36000000UL  // 10 ore în ms
// Definire DHT
DHT dht(DHTPIN, DHTTYPE);
uint8_t FirstReadSensors = 0;
#define FIRST_SENSORS_READ 0xF
// Declarație variabile pentru senzori și timp
/*===========================SENZORI-INCEPUT===========================*/
unsigned int soilMoisture1, soilMoisture2, soilMoisture3;
float dhtTemp, dhtHumidity;
int lightLevel = 0;
uint16_t waterLvlHum = 0;
bool allowHumToWork = true;
float waterVolume = 0.0;
BH1750 lightMeter;
/*===========================SENZORI-SFARSIT===========================*/

/*===========================VARIABILE-INCEPUT===========================*/
unsigned long timeHumOn = 0;
unsigned long lastWaterTime1 = 0;
unsigned long lastWaterTime2 = 0;
unsigned long lastWaterTime3 = 0;
unsigned long pompaTime1 = 0;
unsigned long pompaTime2 = 0;
unsigned long pompaTime3 = 0;  // Timpul la pornirea pompei
unsigned long pompaTime4 = 0;
// Variabile pentru controlul luminii de creștere (artificiale)
bool growLightState = false;                 // Starea luminii artificiale
unsigned long artificialLightStartTime = 0;  // Timpul de pornire a luminii artificiale
unsigned char Tx_BufferSensors[SENS_BUFF_SIZE] = { 0 };
unsigned char Tx_BufferActuators[ACT_BUFF_SIZE] = { 0 };
unsigned char ventilatorLastValue = 0;
unsigned char heatingLastValue = 0;
// Variabile pentru controlul servo (geam de aerisire)
Servo windowServo;
volatile int servoPos = SERVO_CLOSED;
volatile int curentPos = SERVO_CLOSED;
unsigned long prevServoTime = 0;
bool isDeatached = false;
volatile uint8_t allowWindow = 0xF;
unsigned long ClosedTimeRain = 0;
/*===========================VARIABILE-SFARSIT===========================*/

// Variabile pentru citirea senzorilor la intervale specifice
unsigned long previousDHT22Millis = 0;
unsigned long previousMoistureMillis = 0;
unsigned long previousLightMillis = 0;
unsigned long prevUpdateTxBufferActuators = 0;
unsigned long prevUpdateTxBufferSensors = 0;
unsigned long prevRainSensorRead = 0;
unsigned long prevWaterLevel = 0;
unsigned long previousWaterLvlHumMillis = 0;

void setup() {

  // Configurare pini pentru senzori și actuatori
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(blue, OUTPUT);
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
  pinMode(ventilatorLeftPin, OUTPUT);   // Set mode for left fan
  pinMode(ventilatorRightPin, OUTPUT);  // Set mode for right fan
  pinMode(growLightPin, OUTPUT);
  pinMode(humidifierPin, OUTPUT);
  digitalWrite(humidifierPin, RELAY_OFF);
  digitalWrite(growLightPin, RELAY_OFF);
  pinMode(rainPin, INPUT);
  Serial.begin(115200);
  Serial3.begin(115200);
  dht.begin();
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
  // Serial.println("Ploaie daca vad 0:");
  // Serial.println((allowWindow & 0xF));
#endif
}

void loop() {
  while (Serial3.available() > 0) {
   // Serial.println("Eu");
    uint8_t octet = Serial3.read();
    uartByteReceived(octet);
  }
  switch (command) {
    case IDLE:
      break;
    case STOP:
      shutdownAllActuators();
      FirstReadSensors = 0;
      command = IDLE;
      break;
    case START:
      mode = AUTOMATED;
      command = RUNNING;
      break;
    case RUNNING:
      readAllSensors();
      updateCommunicationBuffers();
      applicationLogic(mode);

      break;

    default:
      break;
  }
  updateTime();
}

void readLightSensor() {
  const int numSamples = 12;
  static int samples[numSamples] = { 0 };
  static int index = 0;
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
  const int numSamples = 3;
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
  const int numSensors = 3;
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
    dhtTemp = getMedian(tempSamples, numSamples);
    dhtHumidity = getMedian(humiditySamples, numSamples);
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
    waterVolume = getMedian(samples, numSamples);
#if DEBUG == STD_ON
    //  Serial.println("Volum:");
    // Serial.println(waterVolume);
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
      digitalWrite(growLightPin, RELAY_OFF);
      growLightState = false;
    }
    return;
  }

  if (lightLevel < LOW_LIGHT_THRESHOLD) {
    if (!growLightState) {
      digitalWrite(growLightPin, RELAY_ON);
      growLightState = true;
      artificialLightStartTime = millis();
      //Serial.println("Lumina artificială pornită (lipsă lumină naturală).");
    }
  } else {
    // Oprim lumina artificială dacă nivelul se îmbunătățește sau se atinge durata maximă
    if (growLightState) {
      if (lightLevel >= LOW_LIGHT_THRESHOLD || (millis() - artificialLightStartTime) >= MAX_ARTIFICIAL_LIGHT_DURATION) {
        digitalWrite(growLightPin, RELAY_OFF);
        growLightState = false;
        //Serial.println("Lumina artificială oprită (lumina naturală a revenit sau durata maximă atinsă).");
      }
    }
  }
}

// Funcția controlWaterSystem() gestionează udarea plantelor.
void controlWaterSystem() {
  // Pornirea pompelor dacă sunt îndeplinite toate condițiile + idle time
  if (dhtHumidity <= HumidityUpperLimit && lightLevel < OPTIMAL_LIGHT_LEVEL && waterVolume >= VOLUM_SUFICIENT_APA) {

    if (digitalRead(pumpPin1) == LOW && soilMoisture1 >= SOIL_MOISTURE_DRY_THRESHOLD && (millis() - lastWaterTime1 >= MIN_IDLE_TIME)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 1 pornită");
#endif
      digitalWrite(pumpPin1, HIGH);
      pompaTime1 = millis();
    }

    if (digitalRead(pumpPin2) == LOW && soilMoisture2 >= SOIL_MOISTURE_DRY_THRESHOLD && (millis() - lastWaterTime2 >= MIN_IDLE_TIME)) {
#if DEBUG == STD_ON
      Serial.println("Pompa 2 pornită");
#endif
      digitalWrite(pumpPin2, HIGH);
      pompaTime2 = millis();
    }

    if (digitalRead(pumpPin3) == LOW && soilMoisture3 >= SOIL_MOISTURE_DRY_THRESHOLD && (millis() - lastWaterTime3 >= MIN_IDLE_TIME)) {
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
  static unsigned long lastMoveTime = 0;
  if (millis() - lastMoveTime >= 20) {  // la fiecare 20ms
    lastMoveTime = millis();
    // Logica ta pentru mișcarea servo-ului
    if (requireWindow && (allowWindow & 0xF) == ALLOWED) {
      if (servoPos > SERVO_OPENED) {
        servoPos--;
        windowServo.write(servoPos);
      }
    } else {
        if (requireWindow) {
          ClosedTimeRain += 20;
        }
        if (servoPos < SERVO_CLOSED) {
          servoPos++;
          windowServo.write(servoPos);
        }
    }
  }

  switch (mode) {
    case AUTOMATED:
      if (FirstReadSensors == FIRST_SENSORS_READ) {
        allowHumidifierToWork();
        controlWeatherSystem();
        controlGrowLight();
        controlWaterSystem();
      }
      break;

    case MANUAL:
      switch (actuator.name) {
        case POMPA1:
          Serial.println("POMPA1");
          digitalWrite(pumpPin1, actuator.value);
          lastWaterTime1 = (actuator.value == LOW ? millis() : INVALID_TIMESTAMP);
          pompaTime1 = (actuator.value == HIGH ? millis() : INVALID_TIMESTAMP);
          break;

        case POMPA2:
          Serial.println("POMPA2");
          digitalWrite(pumpPin2, actuator.value);
          lastWaterTime2 = (actuator.value == LOW ? millis() : INVALID_TIMESTAMP);
          pompaTime2 = (actuator.value == HIGH ? millis() : INVALID_TIMESTAMP);
          break;

        case POMPA3:
          Serial.println("POMPA3");
          digitalWrite(pumpPin3, actuator.value);
          lastWaterTime3 = (actuator.value == LOW ? millis() : INVALID_TIMESTAMP);
          pompaTime3 = (actuator.value == HIGH ? millis() : INVALID_TIMESTAMP);
          break;

        case VENTILATOR1:
          Serial.println("VENTILATOR1");
          analogWrite(ventilatorLeftPin, actuator.value == LOW ? LOW : map(actuator.value, 1, 100, 50, 255));
          if (actuator.value == LOW) {
            clrbit(FansState, 0);
            if (FansState == FANS_OFF) FansTime = INVALID_TIMESTAMP;
          } else if (testbit(FansState, 0) == 0) {
            setbit(FansState, 0);
            if (testbit(FansState, 1) == 0) FansTime = millis();
          }
          IdleTimeFans = (FansState == FANS_OFF ? millis() : INVALID_TIMESTAMP);
          Serial.println(IdleTimeFans);
          Serial.println(FansTime);
          break;

        case VENTILATOR2:
          Serial.println("VENTILATOR2");
          analogWrite(ventilatorRightPin, actuator.value == LOW ? LOW : map(actuator.value, 1, 100, 50, 255));
          if (actuator.value == LOW) {
            clrbit(FansState, 1);
            if (FansState == FANS_OFF) FansTime = INVALID_TIMESTAMP;
          } else if (testbit(FansState, 1) == 0) {
            setbit(FansState, 1);
            if (testbit(FansState, 0) == 0) FansTime = millis();
          }
          IdleTimeFans = (FansState == FANS_OFF ? millis() : INVALID_TIMESTAMP);
          Serial.println(IdleTimeFans);
          Serial.println(FansTime);
          break;

        case INCALZIRE1:
          Serial.println("INCALZIRE1");
          digitalWrite(heatingPin_S1a, (actuator.value == LOW) ? RELAY_OFF : RELAY_ON);
          analogWrite(heatingPin_S1b, actuator.value == LOW ? LOW : map(actuator.value, 1, 100, 30, 50));
          break;

        case INCALZIRE2:
          Serial.println("INCALZIRE2");
          digitalWrite(heatingPin_S2a, (actuator.value == LOW) ? RELAY_OFF : RELAY_ON);
          analogWrite(heatingPin_S2b, actuator.value == LOW ? LOW : map(actuator.value, 1, 100, 30, 50));
          break;

        case UMIDIFICATOR:
          Serial.println("Activare: UMIDIFICATOR");
          digitalWrite(humidifierPin, (actuator.value == LOW) ? RELAY_OFF : RELAY_ON);
          break;

        case GEAM:
          Serial.println("GEAM");
          requireWindow = actuator.value;

          if (actuator.value == HIGH) allowWindow = true;
          windowIsOpenedTime = (actuator.value == HIGH ? millis() : INVALID_TIMESTAMP);
          IdleTimeWindow = (actuator.value == LOW ? millis() : INVALID_TIMESTAMP);
          Serial.println(requireWindow);
          Serial.println(allowWindow);
          break;

        case LUMINA:
          Serial.println("Activare: LUMINA");
          digitalWrite(growLightPin, (actuator.value == HIGH) ? RELAY_ON : RELAY_OFF);
          artificialLightStartTime = (actuator.value == HIGH ? millis() : INVALID_TIMESTAMP);
          growLightState = actuator.value;
          break;

        case POMPA4:
          Serial.println("POMPA4");
          digitalWrite(pumpPin4, actuator.value);
          break;

        default:
          break;
      }

      actuator.name = actuator.value = 0;
      break;
  }
}

template <typename T>
int partition(T* arr, int low, int high) {
  T pivot = arr[high];
  int i = low - 1;

  for (int j = low; j <= high - 1; j++) {
    if (arr[j] < pivot) {
      i++;
      if (i != j) {
        T temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
  i++;
  if (i != high) {
    T temp = arr[i];
    arr[i] = arr[high];
    arr[high] = temp;
  }
  return i;
}

template <typename T>
void quickSort(T* arr, int low, int high) {
  if (low < high) {
    int pi = partition(arr, low, high);
    quickSort(arr, low, pi - 1);
    quickSort(arr, pi + 1, high);
  }
}

template <typename T>
T getMedian(T* arr, int size) {
  T sorted[size]; 
  memcpy(sorted, arr, sizeof(sorted));
  quickSort(sorted, 0, size - 1);
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
    HumidityUpperLimit = HumidityDayUpperLimit;
  } else {
    TemperatureLowerLimit = TemperatureNightLowerLimit;
    TemperatureUpperLimit = TemperatureNightUpperLimit;
    HumidityLowerLimit = HumidityNightLowerLimit;
    HumidityUpperLimit = HumidityNightLowerLimit;
  }

  if (temperature == NORMAL_TEMP) {
    if (dhtTemp >= TemperatureUpperLimit + hysteresisBuffer) {
      temperature = HIGH_TEMP;
    } else if (dhtTemp <= TemperatureLowerLimit - hysteresisBuffer) {
      temperature = LOW_TEMP;
    }
  } else if (temperature == HIGH_TEMP) {
    if (dhtTemp <= TemperatureUpperLimit - hysteresisBuffer) {
      temperature = NORMAL_TEMP;
    }
  } else if (temperature == LOW_TEMP) {
    if (dhtTemp >= TemperatureLowerLimit + hysteresisBuffer) {
      temperature = NORMAL_TEMP;
    }
  }

  if (humidity == HUMIDITY_LOW && dhtHumidity > HumidityLowerLimit + hysteresisBuffer) {
    humidity = HUMIDITY_NORMAL;
  } else if (humidity == HUMIDITY_HIGH && dhtHumidity < HumidityUpperLimit - hysteresisBuffer) {
    humidity = HUMIDITY_NORMAL;
  } else if (humidity == HUMIDITY_NORMAL) {
    if (dhtHumidity < HumidityLowerLimit - hysteresisBuffer) {
      humidity = HUMIDITY_LOW;
    } else if (dhtHumidity > HumidityUpperLimit + hysteresisBuffer) {
      humidity = HUMIDITY_HIGH;
    }
  }
  switch (temperature) {
    case HIGH_TEMP:
      digitalWrite(red, HIGH);
      digitalWrite(blue, LOW);
      digitalWrite(yellow, LOW);
      if (humidity == HUMIDITY_LOW) {  //Nivelul de umidate este MIC
        //Serial.println("//Nivelul de umidate este MIC");
        if (requireWindow) {
          IdleTimeWindow = millis();
          requireWindow = false;  //Amprenta de timp pentru inchidere geam
          windowIsOpenedTime = INVALID_TIMESTAMP;
          ClosedTimeRain = 0;
        }

        if (allowHumToWork) { digitalWrite(humidifierPin, RELAY_ON); }
        analogWrite(ventilatorLeftPin, LOW_SPEED);
        analogWrite(ventilatorRightPin, LOW_SPEED);
        ventilatorLastValue = LOW_SPEED;
        if (!FansState) {
          FansTime = millis();  //Amprenta de timp pentru deschidere ventilatoare
          FansState = FANS_ON;
        }
        break;
      } else if (humidity == HUMIDITY_HIGH) {  //Nivelul de umidate este MARE
                                               // Serial.println("//Nivelul de umidate este MARE");
        if (!requireWindow) {
          requireWindow = true;
          windowIsOpenedTime = millis();  //Amprenta de timp pt geam and il deschid
        }
        digitalWrite(humidifierPin, RELAY_OFF);

        if (!FansState) {
          FansTime = millis();  //Amprenta de timp pentru ventilatoare cand le deschid
          FansState = FANS_ON;
        }
        analogWrite(ventilatorLeftPin, HIGH_SPEED);
        analogWrite(ventilatorRightPin, HIGH_SPEED);
        ventilatorLastValue = HIGH_SPEED;
        break;
      } else if (humidity == HUMIDITY_NORMAL) {  //Nivelul de umidaitate este OPTIM -- impart in doua daca trece peste valoarea optima sting umidifcatorul, daca e sub el il las aprins ca sa racesc camera
                                                 //Serial.println("Nivelul de umidate este Mediu");
        if (digitalRead(humidifierPin) == RELAY_OFF && dhtHumidity <= ((HumidityLowerLimit + HumidityUpperLimit) / 2.0)) {
          digitalWrite(humidifierPin, RELAY_ON);
        } else if (digitalRead(humidifierPin) == RELAY_ON && dhtHumidity > ((HumidityLowerLimit + HumidityUpperLimit) / 2.0) + hysteresisBuffer) {
          digitalWrite(humidifierPin, RELAY_OFF);
        }

        if (millis() - windowIsOpenedTime > (TIME_10_MINUTES + ClosedTimeRain) && requireWindow == true) {  // las geamul deschis timp de 10 minute in caz ca nivelul de umiditate este in parametrii normali
          ClosedTimeRain = 0;
          requireWindow = false;                                                                            //Amprenta de timp pentru inchidere geam
          windowIsOpenedTime = INVALID_TIMESTAMP;
          IdleTimeWindow = millis();
        } else if (millis() - IdleTimeWindow >= TIME_3_HOUR && requireWindow == false) {
          windowIsOpenedTime = millis();
          IdleTimeWindow = INVALID_TIMESTAMP;
          requireWindow = true;
        }
        //Ventilatoarele le las mereu pornite
        analogWrite(ventilatorLeftPin, MEDIUM_SPEED);
        analogWrite(ventilatorRightPin, MEDIUM_SPEED);
        ventilatorLastValue = MEDIUM_SPEED;
        if (!FansState) {
          FansTime = millis();
          FansState = FANS_ON;
        }  //Amprenta de timp pentru ventilatoare
      }
      break;

    case NORMAL_TEMP:
      {
        digitalWrite(red, LOW);
        digitalWrite(blue, LOW);
        digitalWrite(yellow, HIGH);
        ControlHeatingSystem(STD_OFF);
        if (humidity == HUMIDITY_LOW) {  //Nivelul de umidate este MIC
          if (requireWindow) {
            requireWindow = false;  //Amprenta de timp pentru inchidere geam
            windowIsOpenedTime = INVALID_TIMESTAMP;
            ClosedTimeRain = 0;
            IdleTimeWindow = millis();
          }
          if (allowHumToWork) { digitalWrite(humidifierPin, RELAY_ON); }
          if (millis() - FansTime > TIME_10_MINUTES && FansState) {
            FansState = FANS_OFF;
            analogWrite(ventilatorLeftPin, 0);
            analogWrite(ventilatorRightPin, 0);
            ventilatorLastValue = 0;
            FansTime = INVALID_TIMESTAMP;
            IdleTimeFans = millis();
          }
          if (millis() - IdleTimeFans >= TIME_1_HOUR && !FansState) {
            FansState = FANS_ON;
            IdleTimeFans = INVALID_TIMESTAMP;
            FansTime = millis();
            analogWrite(ventilatorLeftPin, LOW_SPEED);
            analogWrite(ventilatorRightPin, LOW_SPEED);
            ventilatorLastValue = LOW_SPEED;
          }
        } else if (humidity == HUMIDITY_HIGH) {  //Nivelul de umidate este MARE
          if (!requireWindow) {
            requireWindow = true;  //Amprenta de timp pt geam cand il deschid
            windowIsOpenedTime = millis();
          }
          if (!FansState) {
            FansTime = millis();  //Amprenta de timp pentru ventilatoare cand le deschid
            FansState = FANS_ON;
          }
          digitalWrite(humidifierPin, RELAY_OFF);
          analogWrite(ventilatorLeftPin, MEDIUM_SPEED);
          analogWrite(ventilatorRightPin, MEDIUM_SPEED);
          ventilatorLastValue = MEDIUM_SPEED;
        } else if (humidity == HUMIDITY_NORMAL) {  //Nivelul de umiditate este OPTIM -- deschid geamul la diferite intervale de timp si dau drumul la ventilatoare la diferite intervale de timp - la fiecare ora le las timp de 10 min
          // Mai jos verific cum proedez pt. umidiifator
          if (digitalRead(humidifierPin) == RELAY_OFF && dhtHumidity <= ((HumidityLowerLimit + HumidityUpperLimit) / 2.0)) {
            digitalWrite(humidifierPin, RELAY_ON);
          } else if (digitalRead(humidifierPin) == RELAY_ON && dhtHumidity > ((HumidityLowerLimit + HumidityUpperLimit) / 2.0) + hysteresisBuffer) {
            digitalWrite(humidifierPin, RELAY_OFF);
          }
          //Mai jos verific daca inchid geamul si ventilatorul
          if (millis() - FansTime > TIME_10_MINUTES && FansState) {
            FansState = FANS_OFF;
            FansTime = INVALID_TIMESTAMP;
            analogWrite(ventilatorLeftPin, 0);
            analogWrite(ventilatorRightPin, 0);
            ventilatorLastValue = 0;
            IdleTimeFans = millis();  // Amprenta de timp ventilator cand e stins
          }
          if (millis() - windowIsOpenedTime > (TIME_10_MINUTES + ClosedTimeRain) && requireWindow) {
            ClosedTimeRain = 0;
            requireWindow = false;
            windowIsOpenedTime = INVALID_TIMESTAMP;
            IdleTimeWindow = millis();  //Amprenta de timp pentu geam cand e inchis
          }

          //Mai jos verific daca deschid geamul si ventilatorul
          if (millis() - IdleTimeWindow >= TIME_1_HOUR && !requireWindow) {
            requireWindow = true;
            IdleTimeWindow = INVALID_TIMESTAMP;
            windowIsOpenedTime = millis();  // amprenta de tmp pt geam cand e deschis
          }
          if (millis() - IdleTimeFans >= TIME_1_HOUR && !FansState) {
            FansState = FANS_ON;
            IdleTimeFans = INVALID_TIMESTAMP;
            FansTime = millis();  // amprenta de tmp pt ventilatoare cand sunt deschise
            analogWrite(ventilatorLeftPin, MEDIUM_SPEED);
            analogWrite(ventilatorRightPin, MEDIUM_SPEED);
            ventilatorLastValue = MEDIUM_SPEED;
          }
        }
        break;
      }
    case LOW_TEMP:
      {
        digitalWrite(red, LOW);
        digitalWrite(blue, HIGH);
        digitalWrite(yellow, LOW);

        // Nu se folosește geamul în temperatură joasă
        if (requireWindow) {
          requireWindow = false;  //Amprenta de timp pentru inchidere geam
          windowIsOpenedTime = INVALID_TIMESTAMP;
          ClosedTimeRain = 0;
          IdleTimeWindow = millis();
        }

        ControlHeatingSystem(STD_ON);            // Pornește încălzirea
        digitalWrite(humidifierPin, RELAY_OFF);  // Umidificatorul rămâne oprit

        // Determinăm timpul de funcționare ON în funcție de nivelul de umiditate
        unsigned long onTime = (humidity == HUMIDITY_LOW) ? TIME_10_MINUTES : TIME_20_MINUTES;

        // Oprire ventilatoare după durata ON
        if (millis() - FansTime > onTime && FansState) {
          FansState = FANS_OFF;
          analogWrite(ventilatorLeftPin, 0);
          analogWrite(ventilatorRightPin, 0);
          ventilatorLastValue = 0;
          IdleTimeFans = millis();
          FansTime = INVALID_TIMESTAMP;
        }

        // Pornire ventilatoare după pauză de 3 ore
        if (millis() - IdleTimeFans >= TIME_3_HOUR && !FansState) {
          FansState = FANS_ON;
          FansTime = millis();
          IdleTimeFans = INVALID_TIMESTAMP;
          analogWrite(ventilatorLeftPin, LOW_SPEED);
          analogWrite(ventilatorRightPin, LOW_SPEED);
          ventilatorLastValue = LOW_SPEED;
        }

        break;
      }
    default:
      break;
  }
}


void updateSerialBufferActuators() {
  uint8_t currentState[ACT_BUFF_SIZE];
  currentState[0] = FRAME_START;
  currentState[1] = ACT_BUFF_SIZE;
  currentState[2] = ACT_BUFF_ID;

  /* -------- Servo-------- */
  currentState[3] = requireWindow && (allowWindow & ALLOWED == ALLOWED);
  /* -------- PWM actuators -------- */

  currentState[6] = (heatingLastValue == 0) ? 0 : map(heatingLastValue, 30, 50, 1, 100);
  currentState[7] = (heatingLastValue == 0) ? 0 : map(heatingLastValue, 30, 50, 1, 100);
  currentState[4] = (ventilatorLastValue == 0) ? 0 : map(ventilatorLastValue, 1, 255, 1, 100);
  currentState[5] = (ventilatorLastValue == 0) ? 0 : map(ventilatorLastValue, 1, 255, 1, 100);

  /* -------- digital mask -------- */
  uint8_t dig = (digitalRead(pumpPin1) << 0) | (digitalRead(pumpPin2) << 1) | (digitalRead(pumpPin3) << 2) | (digitalRead(pumpPin4) << 3) | (digitalRead(humidifierPin) << 4) | (digitalRead(growLightPin) << 5) | ((!digitalRead(heatingPin_S1a)) << 6) | ((!digitalRead(heatingPin_S1a)) << 7);
  currentState[8] = dig;
  currentState[9] = 0;
  bool changed = false;
  for (uint8_t i = 3; i < ACT_BUFF_SIZE - 2; i++) {
    if (currentState[i] != Tx_BufferActuators[i]) {
      changed = true;
      break;
    }
  }

  /* -------- CRC -------- */
  if (changed) {
    memcpy(Tx_BufferActuators, currentState, ACT_BUFF_SIZE - 2);
    uint16_t crc = computeCRC(Tx_BufferActuators, ACT_BUFF_SIZE - 2);
    Tx_BufferActuators[10] = uint8_t(crc >> 8);
    Tx_BufferActuators[11] = uint8_t(crc & 0xFF);
    SEND_FRAME(Tx_BufferActuators);
  } else {
  }
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
  Tx_BufferSensors[9] = uint8_t((uint32_t)waterLvlHum * 100 / 600);

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
          Serial3.write(ACK);
          if (rxBuf[2] == ACTUATOR_PKT_ID) {
            handleActuatorFrame(&rxBuf[3]);
          } else if (rxBuf[2] == HARVEST_PKT_ID) {
            handleHarvestFrame(&rxBuf[3]);
          }
        } else {
          Serial3.write(NAK);
        }
        rxState = WAIT_START;
      }
      break;
  }
}


void handleActuatorFrame(const uint8_t *data) {
  switch (data[0]) {
    case CONTROL_SERA:
      mode = (data[1] == STD_ON) ? MANUAL : AUTOMATED;
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
    case POMPA4:
      actuator.name = data[0];
      actuator.value = data[1];
      break;

    default:
      break;
  }

  memset(rxBuf, 0, RX_BUF_MAX);
}

void handleHarvestFrame(const uint8_t *data) {

  if (data[0] == 'P') {
    command = START;
  } else if (data[0] == 'O') {
    command = STOP;
    return;
  } else if (data[0] == 'R') {
    command = RUNNING;
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
    analogWrite(heatingPin_S2b, 35);
    digitalWrite(heatingPin_S2a, RELAY_ON);
    analogWrite(heatingPin_S1b, 35);
    digitalWrite(heatingPin_S1a, RELAY_ON);
    heatingLastValue = 35;
  } else {
    analogWrite(heatingPin_S2b, LOW);
    digitalWrite(heatingPin_S2a, RELAY_OFF);
    digitalWrite(heatingPin_S1a, RELAY_OFF);
    analogWrite(heatingPin_S1b, LOW);
    heatingLastValue = 0;
  }
}

void allowHumidifierToWork() {
  // Turn OFF conditions
  if (waterLvlHum >= HIGH_LVL_HUM || (allowHumToWork && (millis() - timeHumOn >= TIME_5_SECONDS))) {
    digitalWrite(pumpPin4, LOW);
    allowHumToWork = true;
    timeHumOn = 0;
    return;
  }

  // Turn ON condition
  if (waterVolume > VOLUM_SUFICIENT_APA && waterLvlHum <= LOW_LVL_HUM) {
    digitalWrite(pumpPin4, HIGH);
    timeHumOn = millis();
    allowHumToWork = false;
    // Serial.println("Pump ON");
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
    Serial3.write(buf, len);

    unsigned long t = millis();
    while (millis() - t < TIMEOUT_MS) {
      if (Serial3.available()) {
        uint8_t r = Serial3.read();
        if (r == ACK) {
          Serial.println("OK");
          return true;
        }
        if (r == NAK) {
          Serial.println("NOK");
          break;
        }
      }
    }
  }
  return false;
}

void shutdownAllActuators() {
  digitalWrite(pumpPin1, LOW);
  digitalWrite(pumpPin2, LOW);
  digitalWrite(pumpPin3, LOW);
  digitalWrite(pumpPin4, LOW);
  analogWrite(heatingPin_S1b, LOW);
  analogWrite(heatingPin_S2b, LOW);
  digitalWrite(heatingPin_S1a, RELAY_OFF);
  digitalWrite(heatingPin_S2a, RELAY_OFF);
  digitalWrite(humidifierPin, RELAY_OFF);
  analogWrite(ventilatorLeftPin, LOW);
  analogWrite(ventilatorRightPin, LOW);
  digitalWrite(growLightPin, RELAY_OFF);
  while (servoPos < SERVO_CLOSED) {
    servoPos += 5;
    windowServo.write(servoPos);
    delay(20);
  }
  requireWindow = false;
  FansState = FANS_OFF;
  IdleTimeWindow = IdleTimeFans = IdleTimeWindow = windowIsOpenedTime = 0;
}

void readAllSensors() {
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

  if (millis() - prevRainSensorRead >= RainSensorRead && mode == AUTOMATED) {  //in mod manual vreau sa fiu capabil sa deshid geamul chiar daca ploua:)
    prevRainSensorRead = millis();
    allowWindowToWork();
  }

  if ((millis() - prevWaterLevel >= WaterLevelRead) && (digitalRead(pumpPin1) == LOW && digitalRead(pumpPin2) == LOW && digitalRead(pumpPin3) == LOW)) {
    prevWaterLevel = millis();
    readWaterLevel();
  }
}

void updateCommunicationBuffers() {
  if (millis() - prevUpdateTxBufferActuators >= UpdateTxBufferActuators && (FirstReadSensors == FIRST_SENSORS_READ) && mode == AUTOMATED) {
    prevUpdateTxBufferActuators = millis();
    updateSerialBufferActuators();
  }
  if (millis() - prevUpdateTxBufferSensors >= UpdateTxBufferSensors && (FirstReadSensors == FIRST_SENSORS_READ)) {
    prevUpdateTxBufferSensors = millis();
    updateSerialBufferSensors();
  }
}
