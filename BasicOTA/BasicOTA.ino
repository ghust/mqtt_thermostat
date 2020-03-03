#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//nu thermostat
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <OpenTherm.h>

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 4;
const int outPin = 5;

//Data wire is connected to 14 pin on the OpenTherm Shield
#define ONE_WIRE_BUS 14

//All config is now in the secrets file. file is in same directory and looks like: #define SECRET_SSID "Stringvalue"
#include "arduino_secrets.h"

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASSWORD;

const char* mqtt_server = SECRET_MQTT_IP;
const int   mqtt_port = SECRET_MQTT_PORT;
const char* mqtt_user = SECRET_MQTT_USER;
const char* mqtt_password = SECRET_MQTT_PASSWD;

const char* mqtt_setpoint = "SD18/thermostat/opentherm/setpoint";
const char* mqtt_currtemp = "SD18/thermostat/opentherm/currtemp";

//antispam
int i = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
OpenTherm ot(inPin, outPin);
WiFiClient espClient;
PubSubClient client(espClient);
char buf[10];
char buffel[10];

float sp = 23, //set point
      pv = 0, //current temperature
      pv_last = 0, //prior temperature
      ierr = 0, //integral error
      dt = 0, //time between measurements
      op = 0; //PID controller output
unsigned long ts = 0, new_ts = 0; //timestamp
float currtemp = 20; //initial temp



void ICACHE_RAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

float getTemp() {
  return currtemp;
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt) {
  float Kc = 10.0; // K / %Heater
  float tauI = 50.0; // sec
  float tauD = 1.0;  // sec
  // PID coefficients
  float KP = Kc; //10
  float KI = Kc / tauI; // 0.2
  float KD = Kc * tauD; // 10
  // upper and lower bounds on heater level
  float ophi = 100;
  float oplo = 0;
  // calculate the error
  float error = sp - pv;
  // calculate the integral error
  ierr = ierr + KI * error * dt;
  // calculate the measurement derivative
  float dpv = (pv - pv_last) / dt;
  // calculate the PID output
  float P = KP * error; //proportional contribution
  float I = ierr; //integral contribution
  float D = -KD * dpv; //derivative contribution
  float op = P + I + D;
  // implement anti-reset windup
  if ((op < oplo) || (op > ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(ophi, op));
  }
  ierr = I;
  publishMQTT("SD18/thermostat/opentherm/sp_internal", String(sp));
  publishMQTT("SD18/thermostat/opentherm/debug", "sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
  return op;
}

void publishMQTT(String topic, String incoming) {
  Serial.println("MQTT: " + topic + ":" + incoming);
  char charBuf[incoming.length() + 1];
  incoming.toCharArray(charBuf, incoming.length() + 1);

  char topicBuf[topic.length() + 1];
  topic.toCharArray(topicBuf, topic.length() + 1);
  client.publish(topicBuf, charBuf);
}




void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    ESP.restart();
    
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  //thermostat
  //Init DS18B20 Sensor
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); //switch to async mode
  pv, pv_last = sensors.getTempCByIndex(0);
  ts = millis();

  //Init OpenTherm Controller
  ot.begin(handleInterrupt);

  //Init MQTT Client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);



}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("sub");
  //2 chans, mqtt_setpoint & mqtt_currtemp

  if (strcmp(topic, mqtt_setpoint) == 0) {
    //process setpoint
    String spstring = String();
    for (int i = 0; i < length; i++) {
      spstring += (char)payload[i];

    }
    sp = spstring.toFloat();

    Serial.println("Processed setpoint, it's now " + spstring);
  }

  if (strcmp(topic, mqtt_currtemp) == 0) {
    //process currtemp
    String ctstring = String();
    for (int i = 0; i < length; i++) {
      ctstring += (char)payload[i];
    }
    currtemp = ctstring.toFloat();
    Serial.println("Processed currtemp, it's now " + ctstring);
  }


}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect using a random clientid
    String clientId = "Thermostat-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected as " + clientId);
      // Once connected, publish an announcement...
      client.subscribe(mqtt_setpoint);
      client.subscribe(mqtt_currtemp);
      // ... and resubscribe
      client.subscribe(mqtt_setpoint);
      client.subscribe(mqtt_currtemp);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void loop() {
  ArduinoOTA.handle();

  new_ts = millis();
  i = i % 20; //antispam: Once every 20 times

  if (new_ts - ts > 1000) {
    //Set/Get Boiler Status
    bool enableCentralHeating = true;
    bool enableHotWater = true;
    bool enableCooling = false;
    unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
    if (responseStatus != OpenThermResponseStatus::SUCCESS) {
      Serial.println("Error: Invalid boiler response " + String(response, HEX));
    }

    pv = getTemp();
    dt = (new_ts - ts) / 1000.0;
    ts = new_ts;
    
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      op = pid(sp, pv, pv_last, ierr, dt);
      //Set Boiler Temperature
      ot.setBoilerTemperature(op);
    }
    pv_last = pv;

    if (i == 1) {
      float temperature = ot.getBoilerTemperature();
      publishMQTT("SD18/thermostat/opentherm/debug/bt", String(temperature));
    }
    i = i + 1;
  }

  //MQTT Loop
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
