/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * Contribution by a-lurker and Anticimex,
 * Contribution by Norbert Truchsess <norbert.truchsess@t-online.de>
 * Contribution by Ivo Pullens (ESP8266 support)
 *
 * DESCRIPTION
 * The EthernetGateway sends data received from sensors to the WiFi link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 * VERA CONFIGURATION:
 * Enter "ip-number:port" in the ip-field of the Arduino GW device. This will temporarily override any serial configuration for the Vera plugin.
 * E.g. If you want to use the defualt values in this sketch enter: 192.168.178.66:5003
 *
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch, only the LEDs that is defined is used.
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * nRF24L01+  ESP8266
 * VCC        VCC
 * CE         GPIO4
 * CSN/CS     GPIO15
 * SCK        GPIO14
 * MISO       GPIO12
 * MOSI       GPIO13
 * GND        GND
 *
 * Not all ESP8266 modules have all pins available on their external interface.
 * This code has been tested on an ESP-12 module.
 * The ESP8266 requires a certain pin configuration to download code, and another one to run code:
 * - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
 * - Connect GPIO15 via 10K pulldown resistor to GND
 * - Connect CH_PD via 10K resistor to VCC
 * - Connect GPIO2 via 10K resistor to VCC
 * - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
 *
  * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */


// Enable debug prints to serial monitor
#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_GATEWAY_ESP8266

#define MY_ESP8266_SSID "UPC8268667"
#define MY_ESP8266_PASSWORD "Natalia8Marek8"

// Enable UDP communication
//#define MY_USE_UDP

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
#define MY_ESP8266_HOSTNAME "ESP8266sensor-gateway"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
#define MY_IP_ADDRESS 192,168,0,230

// If using static ip you need to define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,0,1
#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 2

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE

// Enable Inclusion mode button on gateway
// #define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN  0

// Set blinking period
// #define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Led pins used if blinking feature is enabled above
//#define MY_DEFAULT_ERR_LED_PIN 2  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  2  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  2  // the PCB, on board LED

#if defined(MY_USE_UDP)
#include <WiFiUdp.h>
#endif

#include <ESP8266WiFi.h>
#include <SPI.h>
#include <MySensors.h>  
#include <Bounce2.h>
#include <Wire.h>
#include <MySensors.h>

#define RELAY_1  12  
#define RELAY_2  13  
#define RELAY_3  14 
#define RELAY_4  5 

#define NUMBER_OF_RELAYS 4

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define BUTTON1_PIN 0
#define BUTTON2_PIN 10
#define BUTTON3_PIN 4
#define BUTTON4_PIN 2

MyMessage msg1(1, V_LIGHT);
MyMessage msg2(2, V_LIGHT);
MyMessage msg3(3, V_LIGHT);
MyMessage msg4(4, V_LIGHT);

Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();
Bounce debouncer3 = Bounce();
Bounce debouncer4 = Bounce();


void setupDebouncer(Bounce& debouncer, byte buttonPin) {
  debouncer.attach(buttonPin);
  debouncer.interval(5);
}

void before() { /*
    pinMode(RELAY_1, OUTPUT);
    digitalWrite(RELAY_1, loadState(1)?RELAY_ON:RELAY_OFF);
    pinMode(RELAY_1, OUTPUT);   
    pinMode(RELAY_2, OUTPUT);  
    digitalWrite(RELAY_2, loadState(2)?RELAY_ON:RELAY_OFF);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);   
    digitalWrite(RELAY_3, loadState(3)?RELAY_ON:RELAY_OFF);
    pinMode(RELAY_3, OUTPUT); 
    pinMode(RELAY_4, OUTPUT);   
    digitalWrite(RELAY_4, loadState(4)?RELAY_ON:RELAY_OFF);  
    pinMode(RELAY_4, OUTPUT);*/
    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);
    pinMode(RELAY_4, OUTPUT);
    digitalWrite(RELAY_1, RELAY_OFF);
    digitalWrite(RELAY_2, RELAY_OFF);
    digitalWrite(RELAY_3, RELAY_OFF);
    digitalWrite(RELAY_4, RELAY_OFF);
}

void setup() {
  delay(1000);

  // expander1.begin(0x38);
  // expander2.begin(0x38);
  
  Serial1.begin(9600);
  
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);  

  setupDebouncer(debouncer1, BUTTON1_PIN);
  setupDebouncer(debouncer2, BUTTON2_PIN);
  setupDebouncer(debouncer3, BUTTON3_PIN);
  setupDebouncer(debouncer4, BUTTON4_PIN);

    digitalWrite(RELAY_1, loadState(1)?RELAY_ON:RELAY_OFF);
    digitalWrite(RELAY_2, loadState(2)?RELAY_ON:RELAY_OFF); 
    digitalWrite(RELAY_3, loadState(3)?RELAY_ON:RELAY_OFF); 
    digitalWrite(RELAY_4, loadState(4)?RELAY_ON:RELAY_OFF);  
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Relay ESP8266", "1.0");

  for (int sensor=1; sensor<=NUMBER_OF_RELAYS;sensor++) {
    // Register all sensors to gw (they will be created as child devices)
    present(sensor, S_LIGHT);  
  }
}


void loop() {
  checkSwitch(debouncer1, msg1, RELAY_1, 1);
  checkSwitch(debouncer2, msg2, RELAY_2, 2);
  checkSwitch(debouncer3, msg3, RELAY_3, 3);
  checkSwitch(debouncer4, msg4, RELAY_4, 4);  
}

void checkSwitch(Bounce& debouncer, MyMessage& myMessage, byte relayPin, byte stateId) {
  if (debouncer.update()) {
    // Get the update value.
    int value = debouncer.read();
    // Send in the new value.
    if (value == LOW){
       saveState(stateId, !loadState(stateId));

         digitalWrite(relayPin, loadState(stateId) ? RELAY_ON : RELAY_OFF);

       send(myMessage.set(loadState(stateId)));
    }
  }
}


void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_LIGHT) {
     // Change relay state
     if (message.sensor < 4) {
        digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
     } else if (message.sensor < 31) {
        digitalWrite(RELAY_4, message.getBool()?RELAY_ON:RELAY_OFF);
     } /*else {
        expander2.digitalWrite(message.sensor-31, message.getBool()?RELAY_ON:RELAY_OFF);
     }*/
     // Store state in eeprom
     saveState(message.sensor, message.getBool());
     // Write some debug info
     Serial.print("Incoming change for sensor:");
     Serial.print(message.sensor);
     Serial.print(", New status: ");
     Serial.println(message.getBool());
   } 
}
