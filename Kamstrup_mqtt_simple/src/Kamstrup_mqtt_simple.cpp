#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "mbusparser.h"
#include "secrets.h"
#include "esp_log.h"
#include "mbedtls/gcm.h"
#include "LEDs.hpp"
#include "ButtonHandler.hpp"
#include "Logger.hpp"

void hexStr2bArr(uint8_t* dest, const char* source, int bytes_n);
void sendmsg(String topic, String payload);

#define IS_DNODE_NVE  false  // True if dNodeNVE. False if testing RX data on TXD pin (to be used in P1+NVE hybrid card)
#define IS_DNODE_OMNI  true  // True if dNodeNVE. False if testing RX data on TXD pin (to be used in P1+NVE hybrid card)

#define DEBUG_BEGIN Serial.begin(115200);
#define DEBUG_PRINT(x) Serial.print(x);sendmsg(String(mqtt_topic)+"/status",x);
#define DEBUG_PRINTLN(x) Serial.println(x);sendmsg(String(mqtt_topic)+"/status",x);

// Pins used for HAN port
#if IS_DNODE_NVE
  #define METER_RX 20//4
  #define METER_TX 10//5
//  #define METER_RX 20//11//4
//  #define METER_TX 21//10//5
  #define NO_OF_LEDS  1
#elif IS_DNODE_OMNI
  // Omni configuration
  #define METER_RX    38
  #define METER_TX    18
  #define PIN_LED     47     // GPIO9 where WS2812B is connected
  #define PIN_BUTTON  0     // GPIO9 where button is connected
  #define NO_OF_LEDS  3
  #define DISABLE_RX_PULLUP
#else
  // For testing hybrid board configuration with P1 on RXD and NVE on TXD
  #define METER_RX 21 //20//11//4
  #define METER_TX -1 //21//10//5
  #define NO_OF_LEDS  1
  #define INVERT_RX_LINE
#endif

const size_t headersize = 11;
const size_t footersize = 3;
uint8_t encryption_key[16];
uint8_t authentication_key[16];
uint8_t receiveBuffer[500];
uint8_t decryptedFrameBuffer[500];
VectorView decryptedFrame(decryptedFrameBuffer, 0);
MbusStreamParser streamParser(receiveBuffer, sizeof(receiveBuffer));
mbedtls_gcm_context m_ctx;

// wifi auto reconnect
unsigned long currentMillis;
unsigned long previousMillis;
unsigned long wifiReconnectInterval = 30000;


LEDs LED(NO_OF_LEDS, PIN_LED, PIN_BUTTON==PIN_LED);               // Initialize the LED strip with 1 LED on pin 9
ButtonHandler button(PIN_BUTTON, LOW);
TaskHandle_t ledButtonTaskHandle = nullptr;


WiFiClient espClient;
PubSubClient client(espClient);



// Button callback handler
void onButtonEvent(ClickType type) {
  switch(type) {
    case SINGLE_CLICK:
      Log.notice("Button: SINGLE_CLICK\n");
      LED.Blink(1, LEDs::ColorG, 250, 250);
      break;

    case DOUBLE_CLICK:
      Log.notice("Button: DOUBLE_CLICK\n");
      LED.Blink(2, LEDs::ColorG, 250, 250);
      break;

    case TRIPLE_CLICK:
      Log.notice("Button: TRIPLE_CLICK\n");
      LED.Blink(3, LEDs::ColorG, 250, 250);
      break;

    case LONG_PRESS_3S_RELEASED:
      Log.notice("Button: LONG_PRESS_3S_RELEASED\n");
      LED.Blink(3, LEDs::ColorR, 250, 250);
      break;

    case LONG_PRESS_5S_RELEASED:
      Log.notice("Button: LONG_PRESS_5S_RELEASED\n");
      LED.Blink(5, LEDs::ColorG, 250, 250);
      break;

    case LONG_PRESS_10S_RELEASED:
      Log.notice("Button: LONG_PRESS_10S_RELEASED\n");
      LED.Blink(10, LEDs::ColorB, 250, 250);
      break;

    case LONG_PRESS_3S_HELD:
      Log.notice("Button: LONG_PRESS_3S_HELD\n");
      LED.setColorNow(LED.ColorR);
      break;

    case LONG_PRESS_5S_HELD:
      Log.notice("Button: LONG_PRESS_5S_HELD\n");
      LED.setColorNow(LED.ColorG);
      break;

    case LONG_PRESS_10S_HELD:
      Log.notice("Button: LONG_PRESS_10S_HELD\n");
      LED.setColorNow(LED.ColorB);
      break;
      
    default:
      Log.notice("Button: UNKNOWN EVENT\n");
      break;
  }
}


// LED/Button handler task function
void ledButtonTask(void* pvParameters) {
    while (true) {
        LED.Handler();
        button.checkButtonClick();
        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 10ms
    }
}


void setup() {
  //DEBUG_BEGIN
  //DEBUG_PRINTLN("")
  Serial.begin(115200);
  
// Set debug level
#ifdef DEBUG
  // Allow debug output to serial if in debug mode
  Serial.setDebugOutput(true);
  esp_log_level_set("*", ESP_LOG_DEBUG); 
#elif defined(RELEASE)
  esp_log_level_set("*", ESP_LOG_NONE);
#else
  esp_log_level_set("*", ESP_LOG_INFO); // ESP_LOG_VERBOSE TODO: Restore this to ESP_LOG_INFO
#endif


//  Serial.setDebugOutput(true);
  Log.begin(USE_LOG_LEVEL, &Serial, true);
  Log.setShowLevel(true);

//   Log.setSuffix(printSuffix);   
  Log.setSuffix( [](Print *_logOutput, int level)  {
    _logOutput->print("\n");
  });



  esp_log_level_set("wifi", ESP_LOG_VERBOSE);

//  WiFi.printDiag(Serial);  // Prints current WiFi config


  Log.notice("Logging initialized");

  LED.Begin();

  LEDs::LEDColor color = static_cast<LEDs::LEDColor>(0x101010); // Low brightness white
  LED.setColorNow(color);                // Set initial color
  LED.setDefaultColorAfterBlinks(color); // Set default color after blinks
  LED.setDefaultColor2(color);           // Set default color 2 in blinking

  button.setCallback(onButtonEvent);  // Set the button event callback

  // Create LED/Button handler task
  xTaskCreate(
      ledButtonTask,         // Task function
      "LedButtonTask",       // Name
      2048,                  // Stack size
      nullptr,               // Parameters
      2,                     // Priority (higher than idle)
      &ledButtonTaskHandle   // Task handle
  );


  WiFi.disconnect(true, true); // erase old credentials
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
//  WiFi.setAutoReconnect(true);
//  WiFi.persistent(true);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
//  WiFi.printDiag(Serial); // Print diagnostic info

Serial.print("WiFi Status: ");
Serial.println(WiFi.status());
Serial.print("IP Address: ");
Serial.println(WiFi.localIP());
client.setSocketTimeout(15); // 15 seconds
  client.setServer(mqttServer, mqttPort);  
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect(mqttClientID, mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
  sendmsg(String(mqtt_topic) + "/status", String("Online"));
//  Serial.begin(2400, SERIAL_8N1);
//  Serial.swap();

  Serial1.begin(2400, SERIAL_8N1, METER_RX, METER_TX);
#ifndef DISABLE_RX_PULLUP
  pinMode(METER_RX, INPUT_PULLUP);
#endif
#ifdef INVERT_RX_LINE
  Serial1.setRxInvert(true);
#endif
  hexStr2bArr(encryption_key, conf_key, sizeof(encryption_key));
  hexStr2bArr(authentication_key, conf_authkey, sizeof(authentication_key));
  Serial.println("Setup completed");
//  digitalWrite(BUILTIN_LED, HIGH);
}


void reconnectWifi() {
  currentMillis = millis();
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= wifiReconnectInterval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}


void sendData(MeterData md) {
  if (md.activePowerPlusValid)
    sendmsg(String(mqtt_topic) + "/power/activePowerPlus", String(md.activePowerPlus));
  if (md.activePowerMinusValid)
    sendmsg(String(mqtt_topic) + "/power/activePowerMinus", String(md.activePowerMinus));
  if (md.activePowerPlusValidL1)
    sendmsg(String(mqtt_topic) + "/power/activePowerPlusL1", String(md.activePowerPlusL1));
  if (md.activePowerMinusValidL1)
    sendmsg(String(mqtt_topic) + "/power/activePowerMinusL1", String(md.activePowerMinusL1));
  if (md.activePowerPlusValidL2)
    sendmsg(String(mqtt_topic) + "/power/activePowerPlusL2", String(md.activePowerPlusL2));
  if (md.activePowerMinusValidL2)
    sendmsg(String(mqtt_topic) + "/power/activePowerMinusL2", String(md.activePowerMinusL2));
  if (md.activePowerPlusValidL3)
    sendmsg(String(mqtt_topic) + "/power/activePowerPlusL3", String(md.activePowerPlusL3));
  if (md.activePowerMinusValidL3)
    sendmsg(String(mqtt_topic) + "/power/activePowerMinusL3", String(md.activePowerMinusL3));
  if (md.reactivePowerPlusValid)
    sendmsg(String(mqtt_topic) + "/power/reactivePowerPlus", String(md.reactivePowerPlus));
  if (md.reactivePowerMinusValid)
    sendmsg(String(mqtt_topic) + "/power/reactivePowerMinus", String(md.reactivePowerMinus));

  if (md.powerFactorValidL1)
    sendmsg(String(mqtt_topic) + "/power/powerFactorL1", String(md.powerFactorL1));
  if (md.powerFactorValidL2)
    sendmsg(String(mqtt_topic) + "/power/powerFactorL2", String(md.powerFactorL2));
  if (md.powerFactorValidL3)
    sendmsg(String(mqtt_topic) + "/power/powerFactorL3", String(md.powerFactorL3));
  if (md.powerFactorTotalValid)
    sendmsg(String(mqtt_topic) + "/power/powerFactorTotal", String(md.powerFactorTotal));

  if (md.voltageL1Valid)
    sendmsg(String(mqtt_topic) + "/voltage/L1", String(md.voltageL1));
  if (md.voltageL2Valid)
    sendmsg(String(mqtt_topic) + "/voltage/L2", String(md.voltageL2));
  if (md.voltageL3Valid)
    sendmsg(String(mqtt_topic) + "/voltage/L3", String(md.voltageL3));

  if (md.centiAmpereL1Valid)
    sendmsg(String(mqtt_topic) + "/current/L1", String(md.centiAmpereL1 / 100.));
  if (md.centiAmpereL2Valid)
    sendmsg(String(mqtt_topic) + "/current/L2", String(md.centiAmpereL2 / 100.));
  if (md.centiAmpereL3Valid)
    sendmsg(String(mqtt_topic) + "/current/L3", String(md.centiAmpereL3 / 100.));

  if (md.activeImportWhValid)
    sendmsg(String(mqtt_topic) + "/energy/activeImportKWh", String(md.activeImportWh / 1000.));
  if (md.activeExportWhValid)
    sendmsg(String(mqtt_topic) + "/energy/activeExportKWh", String(md.activeExportWh / 1000.));
  if (md.activeImportWhValidL1)
    sendmsg(String(mqtt_topic) + "/energy/activeImportKWhL1", String(md.activeImportWhL1 / 1000.));
  if (md.activeExportWhValidL1)
    sendmsg(String(mqtt_topic) + "/energy/activeExportKWhL1", String(md.activeExportWhL1 / 1000.));
  if (md.activeImportWhValidL2)
    sendmsg(String(mqtt_topic) + "/energy/activeImportKWhL2", String(md.activeImportWhL2 / 1000.));
  if (md.activeExportWhValidL2)
    sendmsg(String(mqtt_topic) + "/energy/activeExportKWhL2", String(md.activeExportWhL2 / 1000.));
  if (md.activeImportWhValidL3)
    sendmsg(String(mqtt_topic) + "/energy/activeImportKWhL3", String(md.activeImportWhL3 / 1000.));
  if (md.activeExportWhValidL3)
    sendmsg(String(mqtt_topic) + "/energy/activeExportKWhL3", String(md.activeExportWhL3 / 1000.));

  if (md.reactiveImportWhValid)
    sendmsg(String(mqtt_topic) + "/energy/reactiveImportKWh", String(md.reactiveImportWh / 1000.));
  if (md.reactiveExportWhValid)
    sendmsg(String(mqtt_topic) + "/energy/reactiveExportKWh", String(md.reactiveExportWh / 1000.));
}

void printHex(const unsigned char* data, const size_t length) {
  for (int i = 0; i < length; i++) {
    Serial.printf("%02X", data[i]);
  }
}

void printHex(const VectorView& frame) {
  for (int i = 0; i < frame.size(); i++) {
    Serial.printf("%02X", frame[i]);
  }
}

bool decrypt(const VectorView& frame) {

  if (frame.size() < headersize + footersize + 12 + 18) {
    Serial.println("Invalid frame size.");
  }

  memcpy(decryptedFrameBuffer, &frame.front(), frame.size());

  if(conf_key[0] == '0' && conf_authkey[0] == '0') {
    Serial.println("No auth/encryption key set, skipping decryption.");
  }
  else {
    uint8_t system_title[8];
    memcpy(system_title, decryptedFrameBuffer + headersize + 2, 8);

    uint8_t initialization_vector[12];
    memcpy(initialization_vector, system_title, 8);
    memcpy(initialization_vector + 8, decryptedFrameBuffer + headersize + 14, 4);

    uint8_t additional_authenticated_data[17];
    memcpy(additional_authenticated_data, decryptedFrameBuffer + headersize + 13, 1);
    memcpy(additional_authenticated_data + 1, authentication_key, 16);

    uint8_t authentication_tag[12];
    memcpy(authentication_tag, decryptedFrameBuffer + headersize + frame.size() - headersize - footersize - 12, 12);

    uint8_t cipher_text[frame.size() - headersize - footersize - 18 - 12];
    memcpy(cipher_text, decryptedFrameBuffer + headersize + 18, frame.size() - headersize - footersize - 12 - 18);

    // Prepare the plaintext bufferif
    uint8_t plaintext[sizeof(cipher_text)];

    mbedtls_gcm_init(&m_ctx);
    int success = mbedtls_gcm_setkey(&m_ctx, MBEDTLS_CIPHER_ID_AES, encryption_key, sizeof(encryption_key) * 8);
    if (0 != success) {
      Serial.println("Setkey failed: " + String(success));
      return false;
    }
    success = mbedtls_gcm_auth_decrypt(&m_ctx, sizeof(cipher_text), initialization_vector, sizeof(initialization_vector),
                                      additional_authenticated_data, sizeof(additional_authenticated_data), authentication_tag, sizeof(authentication_tag),
                                      cipher_text, plaintext);
    if (0 != success) {
      Serial.println("authdecrypt failed: " + String(success));
      return false;
    }
    mbedtls_gcm_free(&m_ctx);

    //copy replace encrypted data with decrypted for mbusparser library. Checksum not updated. Hopefully not needed
    memcpy(decryptedFrameBuffer + headersize + 18, plaintext, sizeof(plaintext));
  }
  decryptedFrame = VectorView(decryptedFrameBuffer, frame.size());

  return true;
}

void hexStr2bArr(uint8_t* dest, const char* source, int bytes_n)
{
  uint8_t* dst = dest;
  uint8_t* end = dest + sizeof(bytes_n);
  unsigned int u;

  while (dest < end && sscanf(source, "%2x", &u) == 1)
  {
    *dst++ = u;
    source += 2;
  }
}


void sendmsg(String topic, String payload) {
  if (client.connected() && WiFi.status() == WL_CONNECTED) {
    //digitalWrite(BUILTIN_LED, LOW);
    // If we are connected to WiFi and MQTT, send. (From Niels Ørbæk)
    client.publish(topic.c_str(), payload.c_str());
    delay(10);
    //digitalWrite(BUILTIN_LED, HIGH);
  } else {
    // Otherwise, restart the chip, hoping that the issue resolved itself.
    delay(60*1000);
    ESP.restart();
  }
}


void loop() {
  while (Serial1.available() > 0) {
    //for(int i=0;i<sizeof(input);i++){
    int byte = Serial1.read();
    if(byte > ' ' && byte <= 0x7F)
      Serial.printf("%c", byte);
    else
      Serial.printf("[%02X]", byte);

    Serial.printf("%02X ", byte);

    if (streamParser.pushData(byte)) {//Serial1.read()
      //  if (streamParser.pushData(input[i])) {
      VectorView frame = streamParser.getFrame();
      if (streamParser.getContentType() == MbusStreamParser::COMPLETE_FRAME) {
        DEBUG_PRINTLN("\nFrame complete");
        if (!decrypt(frame))
        {
          DEBUG_PRINTLN("Decryption failed");
          return;
        }
        LED.Blink(1, LEDs::ColorB, 500);
        MeterData md = parseMbusFrame(decryptedFrame);
        sendData(md);
      }
    }
  }

  reconnectWifi();

  client.loop();

  static unsigned long lastTestMillis = 0;
  if (millis() - lastTestMillis >= 5000) {
    lastTestMillis = millis();
//    Serial1.println("ESP->METER");
//    Serial1.flush();
  }
}