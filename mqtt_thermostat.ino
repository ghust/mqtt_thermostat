
#include <ArduinoJson.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>


#include <OpenTherm.h>
#include <LinkedList.h>

class Thermostat {
  public:
    String name;
    float pv, pv_last, sp, ierr, op;
    int last_updated, last_used;
};

LinkedList<Thermostat*> thermList = LinkedList<Thermostat*>();


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


const char* mqtt_profile = "SD18/thermostat/opentherm/profile";
const char* mqtt_command = "SD18/thermostat/opentherm/cmd";

void types(String a) {
  Serial.println("it's a String");
}
void types(int a) {
  Serial.println("it's an int");
}
void types(char* a) {
  Serial.println("it's a char*");
}
void types(float a) {
  Serial.println("it's a float");
}

//antispam
int i = 0;


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
      op = 0, //PID controller output
      globalierr = 0; // stashing
unsigned long ts = 0, new_ts = 0; //timestamp
float currtemp = 20; //initial temp
float currtemp_br = 20;


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

float pid(float sp, float pv, float pv_last, float ierr, float dt) {
  float Kc = 10.0; // K / %Heater
  float tauI = 200.0; // sec
  float tauD = 1.0;  // sec
  // PID coefficients
  float KP = Kc; //10
  float KI = Kc / tauI; // 0.05
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
  globalierr = I;
  //publishMQTT("SD18/thermostat/opentherm/therm_debug", "sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
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

void clearOldThermObjects() {
  int threshold = 1000 * 60 * 15; // 15 minutes
  int now = millis();
  Thermostat* therm;
  for (int i = 0; i < thermList.size(); i++) {
    therm = thermList.get(i);
    if (now - therm->last_updated > threshold) {
      thermList.remove(i);
      publishMQTT("SD18/thermostat/opentherm/clearOldTherm", "Removed Profile " + therm->name + ", it's last updated (" + therm->last_updated + " is more than 15 minutes ago than " + now );
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  setup_wifi();

  ts = millis();

  //Init OpenTherm Controller
  ot.begin(handleInterrupt);

  //Init MQTT Client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);


}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, mqtt_profile) == 0) {
    Serial.println("sub");
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    // use the JsonDocument as usual...
    String profile = doc["name"];
    float value = doc["value"];
    String key = doc["key"];
    // Test if parsing succeeds.
    Serial.println("Received key " + key + "  with value " + value + " for " + profile);

    Thermostat *t;
    bool found = false;
    for (int i = 0; i < thermList.size(); i++) {
      t = thermList.get(i);
      if (t->name == profile) {
        found = true;
        if (String(key) == String("pv")) {
          t->pv = float(value);
          t->last_updated = millis();
        }
        if (String(key) == String("sp")) {
          t->sp = float(value);
          t->last_updated = millis();
        }
      }
    }
    if (found == false) {
      Thermostat *r = new Thermostat();
      r->name = profile;
      if (String(key) == String("pv")) {
        r->pv = float(value);
        r->last_updated = millis();
      }
      if (String(key) == String("sp")) {
        r->sp = float(value);
        r->last_updated = millis();
      }
      thermList.add(r);
    }

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
      client.subscribe(mqtt_command);
      client.subscribe(mqtt_profile);

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
  float maxop = 0;
  new_ts = millis();
  i = i % 60; //antispam: Once every minute

  if (new_ts - ts > 1000) { //every second
    // Set / Get Boiler Status
    bool enableCentralHeating = true;
    bool enableHotWater = true;
    bool enableCooling = false;
    unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
    if (responseStatus != OpenThermResponseStatus::SUCCESS) {
      Serial.println("Error: Invalid boiler response " + String(response, HEX));
    }

    //every second, as per OpenTherm protocol:

    Thermostat *bla ;
    if (thermList.size() > 0) {
      // Serial.println("==================================");
    }
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      for (int i = 0; i < thermList.size(); i++) {
        bla = thermList.get(i);
        ierr = bla->ierr;
        new_ts = millis();

        dt = (new_ts - bla->last_used) / 1000.0;

        sp = bla-> sp;
        pv = bla -> pv;
        pv_last = bla->pv_last;
        ierr = bla->ierr;
        //Serial.println("Processing profile : " + String(bla->name));
        if (sp > 0 && pv > 0) {
          op = pid(bla->sp, bla->pv, bla->pv_last, bla->ierr, dt);
          maxop = max(op, maxop);

          // Serial.println("The maximum op is : " + String(maxop));
          // result vars stashen
          bla->op = op;
          bla->ierr = globalierr;
          bla-> last_used = millis();

          bla-> pv_last = pv;
          publishMQTT("SD18/thermostat/opentherm/debug", "Name=" + bla->name + " pv=" + bla->pv + " sp=" + bla->sp + " last_updated=" + bla->last_updated + " op=" + String(op) + " ierr=" + String(globalierr));
        } else {
          //Serial.println("Name: " + bla->name + "doesn't meet conditions, pv or sp is null");
        }
      }
      publishMQTT("SD18/thermostat/opentherm/maxop",String(maxop));
      //Set Boiler Temperature
      ot.setBoilerTemperature(maxop);
    }

    if (i == 5) { //every minute

      unsigned int data = 0xFFFF;
      unsigned long request = ot.buildRequest(
                                OpenThermRequestType::READ,
                                OpenThermMessageID::CHPressure,
                                data);
      unsigned long response = ot.sendRequest(request);
      float parsedResponse = ot.getFloat(response);
      publishMQTT("SD18/thermostat/opentherm/debug/CHPressure", String(parsedResponse));
    }

    if (i == 1) { //every minute
      float temperature = ot.getBoilerTemperature();
      publishMQTT("SD18/thermostat/opentherm/debug/BoilerTemperature", String(temperature));

      clearOldThermObjects();

    }
    i = i + 1;
  }

  //MQTT Loop
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
