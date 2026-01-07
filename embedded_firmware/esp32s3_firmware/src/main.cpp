#include <WiFi.h>
#include <PubSubClient.h>

#define WIFI_SSID "iPhone"
#define WIFI_PASS "123456789"

#define MQTT_BROKER "192.168.1.10"
#define MQTT_PORT 1883

WiFiClient espClient;
PubSubClient mqttClient(espClient);

/* ================= MQTT CALLBACK ================= */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT topic: ");
  Serial.println(topic);

  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("MQTT msg: ");
  Serial.println(msg);

  if (msg == "LED_ON") {
    Serial.println("LED ON command received");
  }
}

/* ================= WIFI ================= */
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
}

/* ================= MQTT ================= */
void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting MQTT...");

    if (mqttClient.connect("esp32_s3_node_1")) {
      Serial.println("connected");

      mqttClient.subscribe("robot/esp32/cmd");
      mqttClient.publish("robot/esp32/status", "ONLINE");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  connectWiFi();
  

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  Serial.println("MQTT Initialized");
}

void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }

  mqttClient.loop();

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 5000) {
    mqttClient.publish("robot/esp32/heartbeat", "ALIVE");
    lastSend = millis();
  }
}
