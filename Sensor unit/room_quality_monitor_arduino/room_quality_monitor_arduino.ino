/* INCLUDES */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 

/* PINS */
const int power_led_pin = BUILTIN_LED;
const int status_led_pin = 2;

/* CONSTANTS */
const char* ssid = "linksys";
const char* password = "";
const char* mqtt_server = "192.168.1.111";

/* VARIABLES */
bool power_led_state = true;
bool status_led_state = false;

/* OBJECTS */
WiFiClient espClient;
PubSubClient client(espClient);
StaticJsonDocument<256> json_document;

void setup() {
  pinMode(power_led_pin, OUTPUT);
  pinMode(status_led_pin, OUTPUT);

  digitalWrite(power_led_pin, !true);
  digitalWrite(status_led_pin, !false);
  
  Serial.begin(9600);
  initWifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()){
    reconnect();
  }
  client.loop();

  const auto deser_err = deserializeJson(json_document, Serial);
  if (deser_err) {
    power_led_state = !power_led_state;
    status_led_state = !false;
    String errormsg = "{\"status\": \"error\", \"source\": \"esp\", \"msg\": \"Failed to deserialise, reason: " + String(deser_err.c_str()) + "\"}";
    publishData((char*) errormsg.c_str());
  } else {
    power_led_state = !true;
    status_led_state = !status_led_state;
    char json[1024];
    serializeJson(json_document, json);
    publishData(json);
  }

  digitalWrite(power_led_pin, power_led_state);
  digitalWrite(status_led_pin, status_led_state);
  delay(500);
}

void initWifi() {
  delay(10);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishData(char json[1024]) {
  client.publish("sensor", json);
  Serial.println(json);
}
