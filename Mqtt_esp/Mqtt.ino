#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "arduino_base64.hpp"
#include <BearSSLHelpers.h>

/******************CONSTANTS START***************************/
#define SEND_FRAME(pkt) writeFrameWithRetry((uint8_t*)&(pkt), sizeof(pkt))
#define DEBUG STD_OFF
#define STD_ON HIGH
#define STD_OFF LOW
#define MAX_RETRY 3
#define TIMEOUT_MS 10
#define ACK 0x06
#define NAK 0x15
#define SENS_BUFF_ID 's'
#define ACT_BUFF_ID 'a'
#define VALID 0
#define FRAME_START 0xAA
#define ACTUATOR_FRAME_LEN 8
#define HARVEST_FRAME_LEN 20

#define BUFFER_SIZE 21

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
const char ACTUATOR_PKT_ID = 'A';
const char HARVEST_PKT_ID = 'H';
const uint16_t poly = 0x1021;
/******************CONSTANTS END************************/

/********VARIABILE PT FSM SERIAL INCEPUT***********/
#define RX_BUF_MAX 32
enum RxState { WAIT_START,
               WAIT_LEN,
               WAIT_PAYLOAD };
static RxState rxState = WAIT_START;
static uint8_t rxBuf[RX_BUF_MAX];
static uint8_t packetLength = 0;
static uint8_t iterator = 0;
/********VARIABILE PT FSM SERIAL SFARSIT***********/


/******************VARIABLES START***************************/

//Pachet serial de trimis ce contine informatii in legatura cu actuatorii
typedef struct
{
  uint8_t start;
  uint8_t len;
  uint8_t id;
  uint8_t actuator;
  uint8_t state;
  uint8_t dummy;  // adaugat pt padding
  uint16_t crc;
} ActuatorPacket;

//Pachet serial de trimis ce contine informatii in legatura cu statusul recoltei, implicit si informatii necesare
typedef struct
{
  uint8_t start;
  uint8_t len;
  uint8_t id;
  uint8_t data[15];
  uint16_t crc;
} HarvestPacket;

/******************VARIABLES END************************/

// === WiFi Credentials ===
const char* ssid = "DIGI-88Ju";
const char* password = "wF4D7edc";

// === MQTT Broker Config ===
const char* mqttServer = "5852f7b9b4c348afb716480914b3ea19.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "hunterbog";
const char* mqttPassword = "Ashford1875";
// const char* mqttServerCert =
// "-----BEGIN CERTIFICATE-----\n"
// "MIIFPDCCBCSgAwIBAgISBlvO6+BhAYoAodu3Tizj+vEDMA0GCSqGSIb3DQEBCwUA\n"
// "MDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQwwCgYDVQQD\n"
// "EwNSMTEwHhcNMjUwNDIyMjA1OTMwWhcNMjUwNzIxMjA1OTI5WjAfMR0wGwYDVQQD\n"
// "DBQqLnMxLmV1LmhpdmVtcS5jbG91ZDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCC\n"
// "AQoCggEBAKVuz2sMPmxx2w/f81/YAEKTbNZMJPk2+ooLFg5hxXvReF+AwIT4XvZ+\n"
// "MLhSKvFxmghJF+BB9WyhqrcJLGDCP4s6SOLWTYixEoTcaLUviqqn+06kYqDJ6E83\n"
// "NGsc7T42DlPnzqcZZjPRed9rt4CP3RgeZlWyYZgiD8FoJG9gie8ytihF/FkGZT8T\n"
// "N4Vkl2vQa3mfBWeeKrcuhcLPxqIWDz/30iYfLtEe5JYYScoCKTXcP9SUStjpR8pD\n"
// "vfOWdvasOAuBy7yBbx01/4lcQt50hfbhTR/K14/D4rNkuuvU7ktSQnoxVXC8YDwG\n"
// "zkny10DFt65mVYLNZcBQtOLHHOZGV30CAwEAAaOCAlwwggJYMA4GA1UdDwEB/wQE\n"
// "AwIFoDAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDAYDVR0TAQH/BAIw\n"
// "ADAdBgNVHQ4EFgQUgsEjDU35+EWJKBsFxJ0lM0PXMi4wHwYDVR0jBBgwFoAUxc9G\n"
// "pOr0w8B6bJXELbBeki8m47kwVwYIKwYBBQUHAQEESzBJMCIGCCsGAQUFBzABhhZo\n"
// "dHRwOi8vcjExLm8ubGVuY3Iub3JnMCMGCCsGAQUFBzAChhdodHRwOi8vcjExLmku\n"
// "bGVuY3Iub3JnLzAzBgNVHREELDAqghQqLnMxLmV1LmhpdmVtcS5jbG91ZIISczEu\n"
// "ZXUuaGl2ZW1xLmNsb3VkMBMGA1UdIAQMMAowCAYGZ4EMAQIBMC0GA1UdHwQmMCQw\n"
// "IqAgoB6GHGh0dHA6Ly9yMTEuYy5sZW5jci5vcmcvNC5jcmwwggEFBgorBgEEAdZ5\n"
// "AgQCBIH2BIHzAPEAdgAN4fIwK9MNwUBiEgnqVS78R3R8sdfpMO8OQh60fk6qNAAA\n"
// "AZZfgg0JAAAEAwBHMEUCIQCENUD4FWITFwnyxsOr4D54wR+LUgZyEjwMd+GwHiha\n"
// "agIgOdeXyofPYtzl2DajwNvR+6XbCikAbbQvZTZ4Eahu2coAdwDM+w9qhXEJZf6V\n"
// "m1PO6bJ8IumFXA2XjbapflTA/kwNsAAAAZZfghU/AAAEAwBIMEYCIQDu8/zVPYFl\n"
// "bmd1vt5Fqk0sXJLV+MEFhQH75Kn6jlvtFgIhAOA8DAE1QBWXxmYSyFXw9UvC4EvH\n"
// "4+VR1cA8merS5vl4MA0GCSqGSIb3DQEBCwUAA4IBAQBVET3hPDZX/protLVPy/vX\n"
// "4i41k5J3teGokrEMu/TdMN6i/W7555Vsgl1zXj5a1f+4FsQ2Nfh1sDMuz/Djzgxp\n"
// "M8HMifB5HJTX+slAuElLzlQFCxMVNn3+b4BgpxvwA3srrXGudF3cya0qztg5lNju\n"
// "y6zAjYfxMQA0uHtCSuxKk033uFkeBv1ui3XWC1JcISbsoF47RVBp/a5O3kBr+j18\n"
// "k5qL7dWcKWr2S9JctGCH4ezYNmAG9W6w/KoTHH3HJCWrTzziJutY48Rwt4gJcS1s\n"
// "OSV8OT5pGKVpVnKSSOz4ItIaqis6fdetTiba38lUyzjDNklYL72Ye4Ck+qvjyc33\n"
// "-----END CERTIFICATE-----\n";


// === MQTT Topics ===
const char* mqttTopicData = "myhome/esp8266/data";
const char* mqttTopicActuatorsAll = "myhome/esp8266/actuators/all";
const char* mqttTopicSubscribe = "myhome/esp8266/actuator/#";
const char* mqttTopicGreenhouseStatus = "myhome/greenhouse/status";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

unsigned long lastSensorSendTime = 0;
const unsigned long SEND_INTERVAL = 5000;

bool sendToServer = false;
unsigned char cnt_WiFiUnconnected;
unsigned char cnt_MqttUnconnected;

// === Setup WiFi ===
void connectWiFi() {
  WiFi.begin(ssid, password);
  //Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    cnt_WiFiUnconnected++;
    if (cnt_WiFiUnconnected == 20) {
      handleActuatorCommand("mod", "OFF");  // trimite comanda la ATmega pt a functiona automat
      cnt_WiFiUnconnected = 0;
    }
  }
}

// === Setup MQTT ===
void connectMQTT() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect("esp8266_client_001", mqttUser, mqttPassword)) {
#if (DEBUG == STD_ON)    
      Serial.println("Conectat la Mqtt!");
#endif
      mqttClient.subscribe(mqttTopicSubscribe);
      mqttClient.subscribe(mqttTopicGreenhouseStatus);
    } else {
#if (DEBUG == STD_ON)
      Serial.print(mqttClient.state());
      Serial.println("Incerc din nou");
#endif
      delay(1000);
      cnt_MqttUnconnected++;
      if (cnt_MqttUnconnected == 10) {
        handleActuatorCommand("mod", "OFF");  // trimite comanda la ATmega pt a functiona automat
        cnt_MqttUnconnected = 0;
      }
    }
  }
}

/*****************************METODA TRIMITERE INFORMATII DESPRE RECOLTA INCEPUT********************************************/
void handleGreenhouseStatus(const String& message) {
  HarvestPacket harvestPacket{};
  harvestPacket.start = FRAME_START;
  harvestPacket.len = HARVEST_FRAME_LEN;
  harvestPacket.id = HARVEST_PKT_ID;

  // Descompunere mesaj
  int firstComma = message.indexOf(',');
  int secondComma = message.indexOf(',', firstComma + 1);
  int thirdComma = message.indexOf(',', secondComma + 1);
  int fourthComma = message.indexOf(',', thirdComma + 1);
  int fifthComma = message.indexOf(',', fourthComma + 1);

  String hasCropStr = message.substring(0, firstComma);
  String tempStrDay = message.substring(firstComma + 1, secondComma);
  String humStrDay = message.substring(secondComma + 1, thirdComma);
  String tempStrNight = message.substring(thirdComma + 1, fourthComma);
  String humStrNight = message.substring(fourthComma + 1, fifthComma);
  String timeStamp = message.substring(fifthComma + 1);

  int dash1 = tempStrDay.indexOf('-');
  int dash2 = humStrDay.indexOf('-');
  int dash3 = tempStrNight.indexOf('-');
  int dash4 = humStrNight.indexOf('-');

  if (hasCropStr == "True" || hasCropStr == "Running") {
    if (hasCropStr == "True") {
      harvestPacket.data[0] = 'P';  // Pornit
    } else {
      harvestPacket.data[0] = 'R';  // Runnings
    }

    harvestPacket.data[1] = uint8_t(tempStrDay.substring(0, dash1).toInt());
    harvestPacket.data[2] = uint8_t(tempStrDay.substring(dash1 + 1).toInt());
    harvestPacket.data[3] = uint8_t(humStrDay.substring(0, dash2).toInt());
    harvestPacket.data[4] = uint8_t(humStrDay.substring(dash2 + 1).toInt());
    harvestPacket.data[5] = uint8_t(tempStrNight.substring(0, dash3).toInt());
    harvestPacket.data[6] = uint8_t(tempStrNight.substring(dash3 + 1).toInt());
    harvestPacket.data[7] = uint8_t(humStrNight.substring(0, dash4).toInt());
    harvestPacket.data[8] = uint8_t(humStrNight.substring(dash4 + 1).toInt());

    harvestPacket.data[9] = uint8_t(timeStamp.substring(2, 4).toInt());     // Year
    harvestPacket.data[10] = uint8_t(timeStamp.substring(5, 7).toInt());    // Month
    harvestPacket.data[11] = uint8_t(timeStamp.substring(8, 10).toInt());   // Day
    harvestPacket.data[12] = uint8_t(timeStamp.substring(11, 13).toInt());  // Hour
    harvestPacket.data[13] = uint8_t(timeStamp.substring(14, 16).toInt());  // Minute
    harvestPacket.data[14] = uint8_t(timeStamp.substring(17, 19).toInt());  // Second
  } else {
    harvestPacket.data[0] = 'O';  // Oprit
    memset(&harvestPacket.data[1], 0, sizeof(harvestPacket.data) - 1);
  }

  // Calcul CRC
  uint16_t crc = computeCRC((uint8_t*)&harvestPacket, HARVEST_FRAME_LEN - 2);
  harvestPacket.crc = (crc >> 8) | (crc << 8);
  SEND_FRAME(harvestPacket);

// Debug
#if (DEBUG == STD_ON)
  const uint8_t* raw = (const uint8_t*)&harvestPacket;
  for (size_t i = 0; i < sizeof(harvestPacket); ++i) {
    Serial.print(raw[i], DEC);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println(sizeof(harvestPacket));
  Serial.println();
  Serial.println(computeCRC((uint8_t*)&harvestPacket, HARVEST_FRAME_LEN));
#endif
}
/*****************************METODA TRIMITERE INFORMATII DESPRE RECOLTA SFARSIT********************************************/


/********************************METODA TRIMITERE DATE CONTROL CATRE ATMEGA INCEPUT******************************************/

void handleActuatorCommand(const String& actuator, const String& message) {
  uint16_t crc = 0;
  ActuatorPacket actuatorPacket{};

  actuatorPacket.start = FRAME_START;
  actuatorPacket.len = ACTUATOR_FRAME_LEN;
  actuatorPacket.id = ACTUATOR_PKT_ID;
  if (actuator == "mod") {
    actuatorPacket.actuator = CONTROL_SERA;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;  // STD_OFF sera lucreaza in mod automat, STD_ON sera lucreaza in mod manual - cum doreste utilizatorul
  } else if (actuator == "pompa1") {
    actuatorPacket.actuator = POMPA1;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  } else if (actuator == "pompa2") {
    actuatorPacket.actuator = POMPA2;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  } else if (actuator == "pompa3") {
    actuatorPacket.actuator = POMPA3;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  } else if (actuator == "geam") {
    actuatorPacket.actuator = GEAM;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  } else if (actuator == "umidificator") {
    actuatorPacket.actuator = UMIDIFICATOR;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  } else if (actuator == "incalzire1") {
    actuatorPacket.actuator = INCALZIRE1;
    actuatorPacket.state = uint8_t(message.toInt());
  } else if (actuator == "incalzire2") {
    actuatorPacket.actuator = INCALZIRE2;
    actuatorPacket.state = uint8_t(message.toInt());
  } else if (actuator == "ventilator1") {
    actuatorPacket.actuator = VENTILATOR1;
    actuatorPacket.state = uint8_t(message.toInt());
  } else if (actuator == "ventilator2") {
    actuatorPacket.actuator = VENTILATOR2;
    actuatorPacket.state = uint8_t(message.toInt());
  } else if (actuator == "lumina") {
    actuatorPacket.actuator = LUMINA;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  } else if (actuator == "pompa4") {
    actuatorPacket.actuator = POMPA4;
    actuatorPacket.state = (message == "ON") ? STD_ON : STD_OFF;
  }

  crc = computeCRC((uint8_t*)&actuatorPacket, ACTUATOR_FRAME_LEN - 2);
  actuatorPacket.crc = (crc >> 8) | (crc << 8);
  SEND_FRAME(actuatorPacket);
  //Cod de debug
#if (DEBUG == STD_ON)
  const uint8_t* raw = (const uint8_t*)&actuatorPacket;
  for (size_t i = 0; i < sizeof(actuatorPacket); ++i) {
    Serial.print(raw[i], HEX);
    Serial.print(' ');
  }
  Serial.println(sizeof(actuatorPacket));
  Serial.println();
  Serial.println(computeCRC((uint8_t*)&actuatorPacket, ACTUATOR_FRAME_LEN));
#endif
}
/********************************METODA TRIMITERE DATE CONTROL CATRE ATMEGA SFARSIT******************************************/

// === Handle MQTT Messages ===
void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (topicStr.startsWith("myhome/greenhouse/status")) {
    handleGreenhouseStatus(message);
  } else if (topicStr.startsWith("myhome/esp8266/actuator/")) {
    String actuator = topicStr.substring(String("myhome/esp8266/actuator/").length());
    handleActuatorCommand(actuator, message);
  }
}

// === CRC Calculation ===
uint16_t computeCRC(uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= (data[i] << 8);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ poly;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  connectWiFi();
  // static X509List cert(mqttServerCert);
  // espClient.setTrustAnchors(&cert);
  // espClient.setBufferSizes(2048, 2048);
  espClient.setInsecure();
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
}

bool writeFrameWithRetry(const uint8_t* buf, size_t len) {
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
      iterator = 0;
      rxState = WAIT_PAYLOAD;
      break;


    case WAIT_PAYLOAD:
      rxBuf[2 + iterator++] = octet;
      if (iterator == packetLength - 2) {
        if (computeCRC(rxBuf, packetLength) == VALID) {
          Serial.write(ACK);
          if (rxBuf[2] == ACT_BUFF_ID) {
            handleActuatorFrame(&rxBuf[3]);
          } else if (rxBuf[2] == SENS_BUFF_ID) {
            handleSensorFrame(&rxBuf[3]);
          }
        } else {
          Serial.write(NAK);
        }
        rxState = WAIT_START;
      }
      break;
  }
}

void handleSensorFrame(const uint8_t* data) {
  uint16_t soil1 = (data[0] << 8) | data[1];
  uint16_t soil2 = (data[2] << 8) | data[3];
  uint16_t soil3 = (data[4] << 8) | data[5];

  uint8_t soil1Percent = constrain(map(soil1, 1015, 150, 0, 100), 0, 100);
  uint8_t soil2Percent = constrain(map(soil2, 1015, 150, 0, 100), 0, 100);
  uint8_t soil3Percent = constrain(map(soil3, 1015, 150, 0, 100), 0, 100);

  uint8_t waterLevel = data[6];

  float waterVolume = float(data[7]) + float(data[8]) / 100.0f;

  int16_t tempRaw = (data[9] << 8) | data[10];
  int16_t humRaw = (data[11] << 8) | data[12];
  float dhtTemp = tempRaw / 100.0f;
  float dhtHumidity = humRaw / 100.0f;

  uint16_t lightLevel = (data[13] << 8) | data[14];

  String sensorPayload = "{";
  sensorPayload += "\"soil1Percent\":" + String(soil1) + ",";
  sensorPayload += "\"soil2Percent\":" + String(soil2) + ",";
  sensorPayload += "\"soil3Percent\":" + String(soil3) + ",";
  sensorPayload += "\"waterLevel\":" + String(waterLevel) + ",";
  sensorPayload += "\"waterVolume\":" + String(waterVolume, 2) + ",";
  sensorPayload += "\"temperature\":" + String(dhtTemp, 2) + ",";
  sensorPayload += "\"humidity\":" + String(dhtHumidity, 2) + ",";
  sensorPayload += "\"light\":" + String(lightLevel);
  sensorPayload += "}";

  mqttClient.publish(mqttTopicData, sensorPayload.c_str());
  memset(rxBuf, 0, RX_BUF_MAX);
}


void handleActuatorFrame(const uint8_t* data) {

  bool geam = (data[0] == 0 ? false : true);
  uint8_t pwmFanL = data[1];
  uint8_t pwmFanR = data[2];
  uint8_t pwmHeat1 = data[3];
  uint8_t pwmHeat2 = data[4];

  uint8_t digMask = data[5];
  bool pump1 = digMask & 0x01;
  bool pump2 = digMask & 0x02;
  bool pump3 = digMask & 0x04;
  bool pump4 = digMask & 0x08;
  bool humidifier = digMask & 0x10;
  bool growLight = digMask & 0x20;


  String actuatorPayload = "{";
  actuatorPayload += "\"geam\":\"" + String(geam ? "ON" : "OFF") + "\",";
  actuatorPayload += "\"ventilator1\":" + String(pwmFanL) + ",";
  actuatorPayload += "\"ventilator2\":" + String(pwmFanR) + ",";
  actuatorPayload += "\"incalzire1\":" + String(pwmHeat1) + ",";
  actuatorPayload += "\"incalzire2\":" + String(pwmHeat2) + ",";
  actuatorPayload += "\"pompa1\":\"" + String(pump1 ? "ON" : "OFF") + "\",";
  actuatorPayload += "\"pompa2\":\"" + String(pump2 ? "ON" : "OFF") + "\",";
  actuatorPayload += "\"pompa3\":\"" + String(pump3 ? "ON" : "OFF") + "\",";
  actuatorPayload += "\"pompa4\":\"" + String(pump4 ? "ON" : "OFF") + "\",";
  actuatorPayload += "\"umidificator\":\"" + String(humidifier ? "ON" : "OFF") + "\",";
  actuatorPayload += "\"lumina\":\"" + String(growLight ? "ON" : "OFF") + "\"";
  actuatorPayload += "}";

  mqttClient.publish(mqttTopicActuatorsAll, actuatorPayload.c_str());

  memset(rxBuf, 0, RX_BUF_MAX);
}

void loop() {
  // === Serial frame ===
  while (Serial.available() > 0) {
    uint8_t octet = Serial.read();
    uartByteReceived(octet);
  }

  // === WiFi reconnect ===
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // === MQTT reconnect ===
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  // === MQTT loop ===
  mqttClient.loop();
}