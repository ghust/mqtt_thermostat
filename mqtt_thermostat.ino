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
const char* mqtt_setpoint = "SD18/thermostat/opentherm/setpoint_br";
const char* mqtt_currtemp = "SD18/thermostat/opentherm/currtemp_br";
const char* mqtt_command = "SD18/thermostat/opentherm/cmd";


//antispam
int i = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
OpenTherm ot(inPin, outPin);
WiFiClient espClient;
PubSubClient client(espClient);
char buf[10];
char buffel[10];

float sp = 21, //set point
      pv = 0, //current temperature
      pv_last = 0, //prior temperature
      ierr = 0, //integral error
      dt = 0, //time between measurements
      op = 0; //PID controller output
unsigned long ts = 0, new_ts = 0; //timestamp
float currtemp = 20; //initial temp


float sp_br = 21, //set point
      pv_br = 0, //current temperature
      pv_last_br = 0, //prior temperature
      ierr_br = 0, //integral error
      dt_br = 0, //time between measurements
      op_br = 0; //PID controller output

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

  
  if (strcmp(topic, mqtt_setpoint_br) == 0) {
    //process setpoint
    String spstring = String();
    for (int i = 0; i < length; i++) {
      spstring += (char)payload[i];

    }
    sp_br = spstring.toFloat();

    Serial.println("Processed setpoint_br, it's now " + spstring);
  }

  if (strcmp(topic, mqtt_currtemp_br) == 0) {
    //process currtemp
    String ctstring = String();
    for (int i = 0; i < length; i++) {
      ctstring += (char)payload[i];
    }
    currtemp_br = ctstring.toFloat();
    Serial.println("Processed currtemp_br, it's now " + ctstring);
  }

  if (strcmp(topic, mqtt_command) == 0) {
    //process currtemp
    String cmdstring = String();
    for (int i = 0; i < length; i++) {
      cmdstring += (char)payload[i];
    }
    if (cmdstring == "reboot") {
      Serial.println("Rebooting...");
      ESP.restart();
    }
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

      // ... and resubscribe
      client.subscribe(mqtt_setpoint);
      client.subscribe(mqtt_currtemp);
      client.subscribe(mqtt_setpoint_br);
      client.subscribe(mqtt_currtemp_br)
      client.subscribe(mqtt_command);

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
  i = i % 60; //antispam: Once every minute

  if (new_ts - ts > 1000) { //every second
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
      op_br = pid(sp_br, pv_br, pv_last_br, ierr_br, dt);
      op = std::max(op,op_br);
      //Set Boiler Temperature
      ot.setBoilerTemperature(op);
    }
    pv_last = pv;

    if (i == 1) {
      float temperature = ot.getBoilerTemperature();
      publishMQTT("SD18/thermostat/opentherm/debug/BoilerTemperature", String(temperature));
    }
    
    if (i == 5) {

      unsigned int data = 0xFFFF;
      unsigned long request = ot.buildRequest(
                                OpenThermRequestType::READ,
                                OpenThermMessageID::CHPressure,
                                data);
      unsigned long response = ot.sendRequest(request);
      float parsedResponse = ot.getFloat(response);
      publishMQTT("SD18/thermostat/opentherm/debug/CHPressure", String(parsedResponse));
    }
    i = i + 1;
  }

  //MQTT Loop
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
