#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <OpenTherm.h>

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 4;
const int outPin = 5;

//Data wire is connected to 14 pin on the OpenTherm Shield
#define ONE_WIRE_BUS 14
#include "arduino_secrets.h"

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASSWORD;
const char* mqtt_server = SECRET_MQTT_IP;
const int   mqtt_port = SECRET_MQTT_PORT;
const char* mqtt_user = SECRET_MQTT_USER;
const char* mqtt_password = SECRET_MQTT_PASSWD;
const char* mqtt_pubchan = "SD18/thermostat/opentherm/pv";
const char* mqtt_subchan = "SD18/thermostat/opentherm/sv";
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
  float KP = Kc;
  float KI = Kc / tauI;
  float KD = Kc * tauD;
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
  Serial.println("sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
  publishMQTT("SD18/thermostat/opentherm/debug", "sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
  return op;
}

void publishMQTT(String topic, String incoming) {

  char charBuf[incoming.length() + 1];
  incoming.toCharArray(charBuf, incoming.length() + 1);

  char topicBuf[topic.length() + 1];
  topic.toCharArray(topicBuf, topic.length() + 1);
  client.publish(topicBuf, charBuf);
}

void setup_wifi() {
  delay(10);
  //Connect to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(void) {
  Serial.begin(115200);
  setup_wifi();

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

void publish_temperature() {
  //String(sp).toCharArray(buf, 10);
  //client.publish(mqtt_pubchan, buf);
  //publishMQTT(String(sp));
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
    // Attempt to connect
    String clientId = "Thermostat-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected as "+clientId);
      // Once connected, publish an announcement...
      //client.subscribe(mqtt_subchan);
      client.subscribe(mqtt_setpoint);
      client.subscribe(mqtt_currtemp);
      publish_temperature();
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

void loop(void) {
  new_ts = millis();
  i = i % 20; //antispam: 1 om de 20x

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
      publishMQTT("SD18/thermostat/opentherm/debug/bt", "Boiler temperature is " + String(temperature) + " degrees C");
    }
    i = i + 1;
  }

  //MQTT Loop
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
