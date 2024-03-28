#include <SPI.h>

#include <Arduino.h>
#include <PubSubClient.h>
#include "Ethernet.h"

#include <string.h>

#include <WiFi.h>

EthernetClient ethClient;
IPAddress serverIP(174,129,105,59);
void subscribeReceive(char* topic, byte* payload, unsigned int length); 
PubSubClient mqttClient(serverIP, 1883, subscribeReceive, ethClient);

String MQTTUserName = "user";
String MQTTPassword = "pass";

String deviceID;
String masterCommandString;
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAF, 0x19, 0x30 };  

bool MQTTConnectionEstablished = false;
bool ethernetConnected = false;
bool payloadRecieved = false;

uint32_t mqttHeartbeatStartTime = 0;

unsigned long long macToID(String mac);
void connectMQTTServer();
void CheckEthernetComms();
void PrintMQTTLog(String message);

// Definitions
#define ETHERNET_RESET_PIN	21
#define SCLK_PIN  			18 
#define MISO_PIN  			19
#define MOSI_PIN  			23
#define SS_PIN    			4

#define ETH_LED		22
#define MQTT_LED	17

#define DHCP_TIMEOUT_VALUE_MS   30000
#define MAX_BUFFER_SIZE         256   

#define HEART_BEAT_TIMER_MS     60000


unsigned long long macToID(String mac)
{
    const char *originalMacString = mac.c_str();
    String newMacString = "";
    unsigned long long id = 0;

    for(int i = 0; i < mac.length(); i++){
        if(originalMacString[i] != ':'){
            newMacString.concat(originalMacString[i]);
        }
    }

    id = strtoull(newMacString.c_str(), NULL, 16);

    return id;
}

void setup() {
	Serial.begin(115200);
	deviceID = String(macToID(WiFi.macAddress()));

	Serial.println("New Program running");
	Serial.println(String("Device ID: ") + deviceID);

	pinMode(ETHERNET_RESET_PIN, OUTPUT);
	pinMode(ETH_LED, OUTPUT);
	pinMode(MQTT_LED, OUTPUT);
	pinMode(SS_PIN, OUTPUT);

	digitalWrite(ETH_LED, HIGH);
	digitalWrite(MQTT_LED, HIGH);

	digitalWrite(ETHERNET_RESET_PIN, HIGH);
	delay(500);

	Ethernet.init(SS_PIN);
	Serial.println("Turning on Ethernet");
	if(Ethernet.begin(mac, DHCP_TIMEOUT_VALUE_MS)){
		Serial.println("Ethernet running");
		ethernetConnected = true;
		digitalWrite(ETH_LED, LOW);
	} else {
		Serial.println("Ethernet failed to configure");
	}

	if(Ethernet.hardwareStatus() == EthernetW5500){
		Serial.println("Device is W5500");
	} else if(Ethernet.hardwareStatus() == EthernetW5100S){
		Serial.println("Device is W5100S");
	} else {
		Serial.println("Hardware communication issue");
	}

	Serial.println("Connecting MQTT Server");
	if(ethernetConnected){
		connectMQTTServer();
	}

	if(MQTTConnectionEstablished){
		Serial.println("Confirm MQTT Connected");
		PrintMQTTLog("Connected to server");
		digitalWrite(MQTT_LED, LOW);
	}

	Serial.println("Running loop function");
}

void loop() {
  	if(MQTTConnectionEstablished){
		mqttClient.loop();
	}

	if(millis() - mqttHeartbeatStartTime > HEART_BEAT_TIMER_MS){
		if(mqttClient.connected()){
			PrintMQTTLog(String("MQTT heartbeat: ") + String(millis() - mqttHeartbeatStartTime));
		}
		Serial.println(String("MQTT heartbeat: ") + String(millis() - mqttHeartbeatStartTime));
		mqttHeartbeatStartTime = millis();
	}

	if(ethernetConnected){
		Ethernet.maintain();
		CheckEthernetComms();
	}
}

void connectMQTTServer()
{
    Serial.println(F("Connecting to MQTT Server..."));

  //  mqttClient = PubSubClient(serverIP, 1883, subscribeReceive, ethClient);
    if(mqttClient.connect(deviceID.c_str(), MQTTUserName.c_str(), MQTTPassword.c_str())){
        Serial.println(F("Connection established"));
        mqttClient.subscribe(String(String("/device/") + deviceID + String("/command")).c_str());
        mqttClient.subscribe(String(String("/device/") + deviceID + String("/config")).c_str());
        mqttClient.subscribe(String(String("/device/") + deviceID + String("/update")).c_str());
        Serial.println(String("Link Established with version: ") + String("TEST DEV BOARD"));
        MQTTConnectionEstablished = true; // TODO: Test the logic for this being true in the main loop
    } else {
        Serial.println(Ethernet.linkStatus());
        Serial.println(String("Error while connecting: ") + String(mqttClient.state()));
        Serial.println(F("Connection not established"));
    }
}

void CheckEthernetComms()
{
	if (payloadRecieved){
		char commandCString[UDP_TX_PACKET_MAX_SIZE];
		memset(commandCString, 0, UDP_TX_PACKET_MAX_SIZE * sizeof(char));
		memcpy(commandCString, masterCommandString.c_str(), masterCommandString.length());

		// Only do the test command here
		if(!strcmp(commandCString, "test")){
			PrintMQTTLog("Test Command Recieved");
			Serial.println("Check MQTT Log");
		} else {
			Serial.println("Invalid command");
			PrintMQTTLog("Command not supported");
		}

        payloadRecieved = false;
    }
}

void subscribeReceive(char* topic, byte* payload, unsigned int length)
{
	payloadRecieved = true;
    String bufferString;
    String topicString = String(topic);

    for(int i = 0; i < length; i++){
        bufferString.concat((char)payload[i]);
    }

    Serial.println(String("Payload String: ") + bufferString);
    Serial.println(String(topic));
    
    masterCommandString = bufferString;
}

void PrintMQTTLog(String message) // TODO: Rename function
{
    mqttClient.publish(String(String("/server/") + deviceID + String("/log")).c_str(), message.c_str());
}