#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//------ESP WLAN CONFIGURATION------
const char* WIFI_SSID = "iPhone"; 
const char* WIFI_PASSWORD = "123456789";
const char* MQTT_SERVER = "172.20.10.5";  //Linux System wlan ip  
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "pipes/sensors3"; //Topic for ESP1   ESP_CHANGE
const char* MQTT_CLIENT_ID = "ESP32_PipeSystem3";  //ESP_CHANGE

WiFiClient espClient;
PubSubClient mqttClient(espClient);

/* ================= PHYSICAL CONSTANTS ================= */

#define GRAVITY            9.81f    // m/s²
#define WATER_DENSITY      1000.0f  // kg/m³
#define WATER_VISCOSITY    0.001f   // Pa·s 
#define AIR_PRESSURE       101325.0f // Pa 
#define PIPE_ROUGHNESS     0.000015f // m 

/* ================= CONFIG ================= */
#define TOTAL_PIPES        250       //RAM limitation for ESP 32
#define UPDATE_INTERVAL    2000     
#define PHYSICS_DT         0.1f     

// Auto-scale physics substeps based on sensor count
#if TOTAL_PIPES > 1000
  #define PHYSICS_SUBSTEPS 2        // Reduce for large counts
#else
  #define PHYSICS_SUBSTEPS 10       // Full fidelity for small counts
#endif

#define FIRST_FAIL_MIN_MS  60000    // 1 min
#define NEXT_FAIL_MIN_MS   60000    
#define NEXT_FAIL_MAX_MS   120000   
#define MQTT_BATCH_SIZE    100     
#define MQTT_BATCH_DELAY   10       
#define MAX_SENSORS_PER_MESSAGE 100 
#define MQTT_BUFFER_SIZE   32768    




enum PipeType {
  FRESH_WATER = 0,
  SEWAGE = 1
};


struct Sensor {
  int sensorId;
  int pipeId;
  PipeType type;
  float pressure;      
  float level;        
  bool valveOpen;
  
  // Internal physics data 
  float _pressurePa;   // Pa 
  float diameter;      // m
  float length;        // m
  float roughness;     // m
  float elevation;     // m
  float crossSection;  // m²
  
  float flowRate;      // m³/s
  float velocity;      // m/s
  float temperature;   // °C
  
  float demandFlow;    // m³/s
  float supplyPressure;// Pa
  
  float leakCoeff;     // 0.0 = no leak, 1.0 = severe
  float blockageCoeff; // 0.0 = no blockage, 1.0 = blocked
  bool degrading;
  
  float reynoldsNumber;
  float frictionFactor;
  float headLoss;      // m
};

Sensor sensors[TOTAL_PIPES];
unsigned long lastUpdate = 0;
unsigned long nextDegradeTime = 0;
int remainingToDegrade = TOTAL_PIPES;
unsigned long lastMqttPublish = 0;
int mqttPublishIndex = 0;
int activeSensorCount = 0;



bool configReceived = false;
bool initializedFromConfig = false;
unsigned long mqttConnectTime = 0;
const unsigned long CONFIG_WAIT_TIME = 5 * 60 * 1000;  

// Static buffers to avoid heap fragmentation 
static char mqttBuffer[MQTT_BUFFER_SIZE];
static char topicBuffer[128];

// Static JSON document to avoid repeated heap allocation
static DynamicJsonDocument mqttDoc(MQTT_BUFFER_SIZE);

// Task handles for FreeRTOS
TaskHandle_t physicsTaskHandle = NULL;
SemaphoreHandle_t sensorMutex = NULL;


float frand(float minVal, float maxVal) {
  return minVal + (float)random(0, 10000) / 10000.0 * (maxVal - minVal);
}

float clampf(float val, float minVal, float maxVal) {
  return max(minVal, min(maxVal, val));
}

void publishSystemSummary();
void reconnectMQTT();

// Calculate Reynolds Number (determines laminar vs turbulent flow)
float calculateReynolds(float velocity, float diameter, float viscosity) {
  return (WATER_DENSITY * velocity * diameter) / viscosity;
}

// Calculate friction factor using Colebrook-White equation 
float calculateFrictionFactor(float reynolds, float diameter, float roughness) {
  if (reynolds < 1.0f) {
    return 0.0f;
  } 
  else if (reynolds < 2300.0f) {
    return 64.0f / reynolds;   // laminar
  }
  else if (reynolds < 4000.0f) {
    float laminar = 64.0f / 2300.0f;
    float turbulent = 0.02f;
    float t = (reynolds - 2300.0f) / 1700.0f;
    return laminar + t * (turbulent - laminar);
  } 
  else {
    float term1 = roughness / (3.7f * diameter);
    float term2 = 5.74f / pow(reynolds, 0.9f);
    return 0.25f / pow(log10(term1 + term2), 2.0f);
  }
}


// Calculate head loss using Darcy-Weisbach equation
float calculateHeadLoss(float frictionFactor, float length, float diameter, float velocity) {
  return frictionFactor * (length / diameter) * (velocity * velocity) / (2.0f * GRAVITY);
}

// Calculate pressure from head (height)
float headToPressure(float head) {
  return head * WATER_DENSITY * GRAVITY;
}

// Calculate head from pressure
float pressureToHead(float pressure) {
  return pressure / (WATER_DENSITY * GRAVITY);
}

// Calculate leak flow rate (orifice equation)
float calculateLeakFlow(float pressure, float leakCoeff, float diameter) {
  if (leakCoeff < 0.001f) return 0.0f;
  
  // Gauge pressure (above atmospheric)
  float gaugePressure = max(0.0f, pressure - AIR_PRESSURE);
  if (gaugePressure < 1000.0f) return 0.0f;
  
  float leakSizeRatio = leakCoeff * 0.1f; 
  float leakArea = leakSizeRatio * PI * diameter * diameter / 4.0f;
  
  // Convert pressure to head
  float head = gaugePressure / (WATER_DENSITY * GRAVITY);
  
  // Orifice equation with high discharge 
  float dischargeCoeff = 0.8f;
  float leakFlow = dischargeCoeff * leakArea * sqrt(2.0f * GRAVITY * head);
  
  return leakFlow;
}

// Calculate blockage effect on flow
float applyBlockage(float flow, float blockageCoeff) {
    float blockageFactor = exp(-blockageCoeff * 3.0f);
    return flow * blockageFactor;
}

/* ================= INITIALIZATION ================= */
void initSystem() {
  Serial.println("===========================================");
  Serial.println(" PIPE NETWORK SENSOR SIMULATION");
  Serial.println("===========================================");
  
  for (int i = 0; i < TOTAL_PIPES; i++) {
    sensors[i].sensorId = i;
    sensors[i].pipeId = i;
    sensors[i].type = (i < TOTAL_PIPES / 2) ? FRESH_WATER : SEWAGE;
    sensors[i].valveOpen = true;
    sensors[i].degrading = false;
    sensors[i].leakCoeff = 0.0f;
    sensors[i].blockageCoeff = 0.0f;
    
    if (sensors[i].type == FRESH_WATER) {
      // Fresh water pipes - each one DIFFERENT
      sensors[i].diameter = 0.05f + (i % 11) * 0.01f;
      sensors[i].length = 50.0f + (i % 20) * 25.0f;   
      sensors[i].roughness = 0.000015f + (i % 5) * 0.000001f;
      sensors[i].elevation = -5.0f + (i % 26) * 1.0f; 
      sensors[i].temperature = 10.0f + (i % 6) * 1.0f;
      
      // UNIQUE supply pressure and demand for each sensor
      sensors[i].supplyPressure = 300000.0f + (i % 21) * 10000.0f; 
      sensors[i].demandFlow = 0.001f + (i % 10) * 0.001f; 
      
      // UNIQUE initial pressure
      sensors[i]._pressurePa = sensors[i].supplyPressure * (0.6f + (i % 5) * 0.1f);
      sensors[i].pressure = sensors[i]._pressurePa / 100000.0f;
      sensors[i].flowRate = sensors[i].demandFlow;
      sensors[i].level = 70.0f + (i % 31) * 1.0f; 
      
    } else {
   
      sensors[i].diameter = 0.10f + (i % 21) * 0.01f; 
      sensors[i].length = 50.0f + (i % 20) * 25.0f;  
      sensors[i].roughness = 0.00006f + (i % 5) * 0.00001f;
      sensors[i].elevation = -20.0f + (i % 19) * 1.0f; 
      sensors[i].temperature = 15.0f + (i % 11) * 1.0f; 
      
      sensors[i].supplyPressure = AIR_PRESSURE + 5000.0f + (i % 16) * 1000.0f;
      sensors[i].demandFlow = 0.002f + (i % 14) * 0.001f; 
      
      sensors[i]._pressurePa = AIR_PRESSURE + 5000.0f + (i % 11) * 1000.0f;
      sensors[i].pressure = sensors[i]._pressurePa / 100000.0f;
      sensors[i].flowRate = sensors[i].demandFlow;
      sensors[i].level = 30.0f + (i % 71) * 1.0f; 
    }
    
    sensors[i].crossSection = PI * sensors[i].diameter * sensors[i].diameter / 4.0f;
    sensors[i].velocity = sensors[i].flowRate / sensors[i].crossSection;
    
    // Initialize Reynolds number
    float effectiveDiameter = sensors[i].diameter;
    if (sensors[i].type == SEWAGE) {
      float levelRatio = max(sensors[i].level / 100.0f, 0.01f);
      float filledArea = sensors[i].crossSection * levelRatio;
      float wettedPerimeter = PI * sensors[i].diameter * levelRatio;
      effectiveDiameter = 4.0f * filledArea / wettedPerimeter;
      effectiveDiameter = clampf(effectiveDiameter, 0.02f, sensors[i].diameter);
    }
    
    if (sensors[i].velocity < 0.00001f || effectiveDiameter < 0.00001f) {
      sensors[i].reynoldsNumber = 0.0f;
    } else {
      sensors[i].reynoldsNumber = calculateReynolds(sensors[i].velocity, effectiveDiameter, WATER_VISCOSITY);
    }
  }
  
  Serial.println("System initialized with unique sensor properties");
  Serial.println("===========================================");
}

/* ================= DEGRADATION ================= */
void scheduleNextDegradation() {

  unsigned long gap = random(15000, 45000);
  nextDegradeTime = millis() + gap;
  
  Serial.print("Next failure in ");
  Serial.print(gap / 1000);
  Serial.println(" seconds");
}

void activateRandomDegradation() {
  if (remainingToDegrade <= 0) return;
  
  int id;
  do {
    id = random(0, activeSensorCount);
  } while (sensors[id].degrading);
  
  sensors[id].degrading = true;
  remainingToDegrade--;
  
  int failureType = random(0, 100);
  
  Serial.print("DEGRADATION | SENSOR_ID=");
  Serial.print(id);
  Serial.print(" | PIPE_ID=");
  Serial.print(sensors[id].pipeId);
  Serial.print(" | TYPE=");
  
  if (sensors[id].type == FRESH_WATER) {
    if (failureType < 70) {

      Serial.print("FRESH-LEAK");
      sensors[id].leakCoeff = 0.01f;
      sensors[id].blockageCoeff = 0.0f;
    } else {

      Serial.print("FRESH-BLOCK");
      sensors[id].blockageCoeff = 0.01f;
      sensors[id].leakCoeff = 0.0f;
    }
  } else {
    if (failureType < 70) {

      Serial.print("SEWAGE-BLOCK");
      sensors[id].blockageCoeff = 0.01f;
      sensors[id].leakCoeff = 0.0f;
    } else {

      Serial.print("SEWAGE-LEAK");
      sensors[id].leakCoeff = 0.01f;
      sensors[id].blockageCoeff = 0.0f;
    }
  }
  
  Serial.println();
  scheduleNextDegradation();
}

/* ================= PHYSICS UPDATE ================= */
void updatePhysics(float dt) {
  for (int i = 0; i < activeSensorCount; i++) {
    Sensor &s = sensors[i];
    
    if (!s.valveOpen) {
      s.flowRate = 0.0f;
      s.velocity = 0.0f;
      continue;
    }
    
    // Calculate leak flow 
    float leakFlow = 0.0f;
    if (s.leakCoeff > 0.001f) {
      leakFlow = calculateLeakFlow(s._pressurePa, s.leakCoeff, s.diameter);
    }
    
    // Calculate target flow 
    float targetFlow = s.demandFlow;
    
    if (s.type == FRESH_WATER) {
    float pressureHead = pressureToHead(s._pressurePa);
    float pressureFactor = clampf(pressureHead / (30.0f + (i % 10)), 0.0f, 1.2f);
    targetFlow = s.demandFlow * pressureFactor;
    

    targetFlow = max(0.0f, targetFlow - leakFlow);
    

    targetFlow = applyBlockage(targetFlow, s.blockageCoeff);
}
    
    // Smooth flow changes with random variation
    float smoothing = 0.5f + frand(-0.1f, 0.1f);
    s.flowRate += (targetFlow - s.flowRate) * dt * smoothing;
    s.flowRate = max(0.0f, s.flowRate);
    
    // Calculate velocity 
    if (s.type == FRESH_WATER) {
      s.velocity = s.flowRate / max(s.crossSection, 0.0001f);
    } else {
      float levelRatio = max(s.level / 100.0f, 0.01f);
      float filledArea = s.crossSection * levelRatio;
      s.velocity = max(s.flowRate / filledArea, 0.0f);
    }
    
    // Reynolds number calculation
    float effectiveDiameter = s.diameter;
    if (s.type == SEWAGE) {
      float levelRatio = max(s.level / 100.0f, 0.01f);
      float filledArea = s.crossSection * levelRatio;
      float wettedPerimeter = PI * s.diameter * levelRatio;
      effectiveDiameter = 4.0f * filledArea / wettedPerimeter;
      effectiveDiameter = clampf(effectiveDiameter, 0.02f, s.diameter);
    }
    
    if (s.velocity < 0.001f || effectiveDiameter < 0.001f) {
      s.reynoldsNumber = 0.0f;
      s.frictionFactor = 0.0f;
      s.headLoss = 0.0f;
    } else {
      float localViscosity = WATER_VISCOSITY * (1.0f + frand(-0.05f, 0.05f));
      s.reynoldsNumber = calculateReynolds(s.velocity, effectiveDiameter, localViscosity);
      s.frictionFactor = calculateFrictionFactor(s.reynoldsNumber, effectiveDiameter, s.roughness);
      s.headLoss = calculateHeadLoss(s.frictionFactor, s.length, effectiveDiameter, s.velocity);
    }
    
    // Update pressure 
    if (s.type == FRESH_WATER) {
      float pressureLoss = headToPressure(s.headLoss);
      float elevationPressure = WATER_DENSITY * GRAVITY * s.elevation;
      s._pressurePa = s.supplyPressure - pressureLoss - elevationPressure;
     
      if (s.leakCoeff > 0.001f) {
        float leakPressureDrop = s.leakCoeff * s.leakCoeff * 150000.0f;
        s._pressurePa -= leakPressureDrop;
      }
      
      s._pressurePa = clampf(s._pressurePa, 50000.0f, 800000.0f);
      s.pressure = s._pressurePa / 100000.0f;
      
      // Level variations
      if (s.leakCoeff > 0.01f) {
        s.level -= s.leakCoeff * s.leakCoeff * 100.0f * dt;
}
      
      // Add random variations to level
      s.level += frand(-0.1f, 0.1f);
      s.level = clampf(s.level, 0.0f, 100.0f);
      
    } else {
      // Sewage: unique level changes
      float inflow = s.demandFlow * dt;
      float outflow = s.flowRate * dt;
      float volumeChange = (inflow - outflow);
      
      // Add random variation
      volumeChange += frand(-0.0001f, 0.0001f);
      
      float levelChange = (volumeChange / (s.crossSection * s.length)) * 100.0f;
      s.level += levelChange;
      s.level = clampf(s.level, 0.0f, 100.0f); 
      
      // Blockage causes level rise 
      if (s.blockageCoeff > 0.1f) {
        s.level += s.blockageCoeff * 0.3f * dt;
      }
      
      // Leak causes level drop in sewage
      if (s.leakCoeff > 0.01f) {
    // Sewage leaks are less dramatic since pipes aren't always full
    s.level -= s.leakCoeff * s.leakCoeff * 50.0f * dt;
}
      
      float fluidHeight = (s.level / 100.0f) * s.diameter;
      s._pressurePa = AIR_PRESSURE + WATER_DENSITY * GRAVITY * fluidHeight;
      s.pressure = s._pressurePa / 100000.0f;
    }
    
    // DEGRADATION EFFECTS 
    if (s.degrading) {
      //FRESH WATER PIPES
      if (s.type == FRESH_WATER) {
        if (s.leakCoeff > 0.0f) {
          s.leakCoeff += (0.002f + (i % 10) * 0.0005f) * dt;
          s.leakCoeff = min(1.0f, s.leakCoeff);
          s.roughness += frand(0.0000005f, 0.000002f) * dt;
        }
        
        if (s.type == FRESH_WATER && s.blockageCoeff > 0.01f) {
        float blockagePressureDrop = s.blockageCoeff * 300000.0f;  
        s._pressurePa -= blockagePressureDrop;
    
} else if (s.type == SEWAGE && s.blockageCoeff > 0.01f) {
    float blockagePressureDrop = s.blockageCoeff * 100000.0f;
    s._pressurePa -= blockagePressureDrop;
}
      } 
      //SEWAGE PIPES
      else {
        if (s.blockageCoeff > 0.0f) {
          s.blockageCoeff += (0.003f + (i % 10) * 0.0007f) * dt;
          s.blockageCoeff = min(0.95f, s.blockageCoeff);
          s.roughness += frand(0.000001f, 0.000003f) * dt;
        }
        if (s.leakCoeff > 0.0f) {
          s.leakCoeff += (0.001f + (i % 10) * 0.0002f) * dt;
          s.leakCoeff = min(0.6f, s.leakCoeff);  
          s.roughness += frand(0.0000005f, 0.000002f) * dt;
        }
      }
    }
    s.pressure += frand(-0.03f, 0.03f);
    s.level += frand(-0.3f, 0.3f);
    s.temperature += frand(-0.1f, 0.1f);
    s.pressure = max(0.1f, s.pressure);
    s.level = clampf(s.level, 0.0f, 100.0f);
    s.flowRate = max(0.0f, s.flowRate);
    
    if (i % 25 == 0) {
      yield();
    }
  }
}

/* ================= LOGGING ================= */
void printSensor(int i) {
  Sensor &s = sensors[i];
  
  Serial.print("SENSOR_ID=");
  Serial.print(s.sensorId);
  Serial.print(" | PIPE_ID=");
  Serial.print(s.pipeId);
  Serial.print(" | TYPE=");
  Serial.print(s.type == FRESH_WATER ? "FRESH" : "SEWAGE");
  Serial.print(" | PRESSURE=");
  Serial.print(s.pressure, 3);  
  Serial.print(" bar | LEVEL=");
  Serial.print(s.level, 2);    
  Serial.print("% | VALVE=");
  Serial.print(s.valveOpen ? "OPEN" : "CLOSED");
  
  if (s.degrading) {
    Serial.print(" | DEGRADING");
    
    if (s.leakCoeff > 0.01f) {
      Serial.print(" LEAK=");
      Serial.print(s.leakCoeff * 100.0f, 1);
      Serial.print("%");
      if (s.type == SEWAGE) {
        Serial.print("[SEW-LEAK]");
      }
    }
   
    if (s.blockageCoeff > 0.01f) {
      Serial.print(" BLOCK=");
      Serial.print(s.blockageCoeff * 100.0f, 1);
      Serial.print("%");
      if (s.type == FRESH_WATER) {
        Serial.print("[FRESH-BLOCK]");
      }
    }
  }
  
  Serial.println();
}

void verifySensorUniqueness() {
  Serial.println("========================================");
  Serial.println("VERIFYING SENSOR UNIQUENESS");
  Serial.println("========================================");
  
  for (int i = 0; i < min(10, activeSensorCount); i++) {
    Sensor &s = sensors[i];
    Serial.print("SENSOR ");
    Serial.print(i);
    Serial.print(": D=");
    Serial.print(s.diameter * 1000.0f, 0);
    Serial.print("mm, L=");
    Serial.print(s.length, 0);
    Serial.print("m, Elev=");
    Serial.print(s.elevation, 1);
    Serial.print("m, Supply=");
    Serial.print(s.supplyPressure / 100000.0f, 2);
    Serial.print("bar, Demand=");
    Serial.print(s.demandFlow * 1000.0f, 2);
    Serial.println(" L/s");
  }
  
  Serial.println("========================================");
}

/* ================= MQTT FUNCTIONS ================= */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Only handle init config
  if (strcmp(topic, "pipes/config/init3") != 0) { //ESP_CHANGE
    return;
  }

  Serial.println("===========================================");
  Serial.println("CONFIGURATION MESSAGE RECEIVED");
  Serial.println("===========================================");
  Serial.print("Payload size: ");
  Serial.print(length);
  Serial.println(" bytes");

  // Large JSON document
  DynamicJsonDocument doc(65536);
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  unsigned long timestamp = doc["timestamp"] | 0;
  float dt = doc["dt"] | 1.0f;

  Serial.print("Timestamp: ");
  Serial.println(timestamp);
  Serial.print("dt: ");
  Serial.println(dt);

  JsonArray sensorsArray = doc["sensors"];
  if (sensorsArray.isNull()) {
    Serial.println("No sensors array in config!");
    return;
  }

  int configuredCount = 0;


  for (JsonObject sensorObj : sensorsArray) {

    int sensorId = sensorObj["sensor_id"] | -1;
    if (sensorId < 0 || sensorId >= TOTAL_PIPES) continue;

    Sensor &s = sensors[sensorId];

    int pipeId = sensorObj["pipe_id"] | sensorId;
    const char* typeStr = sensorObj["type"] | "fresh";
    float pressureBar = sensorObj["pressure_bar"] | 0.0f;
    float levelPct = sensorObj["level_pct"] | 0.0f;
    int valve = sensorObj["valve"] | 1;


    s.sensorId = sensorId;
    s.pipeId   = pipeId;
    s.type     = (strcmp(typeStr, "fresh") == 0) ? FRESH_WATER : SEWAGE;

    s.valveOpen = (valve == 1);

    s.pressure     = pressureBar;
    s._pressurePa  = pressureBar * 100000.0f;

    s.supplyPressure = s._pressurePa;

    s.level = clampf(levelPct, 0.0f, 100.0f);
    if (s.type == FRESH_WATER) {
      s.diameter     = 0.10f;      
      s.length       = 100.0f;     
      s.roughness    = 0.000015f;  
      s.elevation    = 0.0f;
      s.temperature  = 15.0f;
      s.demandFlow   = 0.005f;   
    } else {
      s.diameter     = 0.25f; 
      s.length       = 120.0f;
      s.roughness    = 0.00006f;   
      s.elevation    = -2.0f;
      s.temperature  = 20.0f;
      s.demandFlow   = 0.007f;     
    }

    s.crossSection = PI * s.diameter * s.diameter / 4.0f;
    s.flowRate     = s.demandFlow;
    s.velocity     = 0.0f;

    s.leakCoeff     = 0.0f;
    s.blockageCoeff = 0.0f;
    s.degrading     = false;

    configuredCount++;
  }

  activeSensorCount = configuredCount;

  Serial.print("Active sensors locked to: ");
  Serial.println(activeSensorCount);
  Serial.print("✓ Total sensors configured: ");
  Serial.println(configuredCount);
  Serial.println("===========================================");

  configReceived = true;
  initializedFromConfig = true;
  nextDegradeTime = millis() + FIRST_FAIL_MIN_MS;
  remainingToDegrade = activeSensorCount;


}


void setupWiFi() {
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker...");
    
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected!");
      mqttConnectTime = millis();
      
      // Subscribe to config topic
      mqttClient.subscribe("pipes/config/init3"); //ESP_CHANGE
      Serial.println("Subscribed to pipes/config/init");
      Serial.println("Waiting for configuration message (50 seconds timeout)...");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void publishSensorData(int i) {
  Sensor &s = sensors[i];

  StaticJsonDocument<512> doc;
  
  doc["sensor_id"] = s.sensorId;
  doc["pipe_id"] = s.pipeId;
  doc["type"] = (s.type == FRESH_WATER) ? "fresh" : "sewage";
  doc["pressure_bar"] = round(s.pressure * 1000.0) / 1000.0;
  doc["level_pct"] = round(s.level * 1000.0) / 1000.0;
  doc["valve"] = s.valveOpen ? 1 : 0;
  doc["flow_rate_Ls"] = round(s.flowRate * 1000.0 * 100.0) / 100.0;
  doc["velocity_ms"] = round(s.velocity * 100.0) / 100.0;
  doc["temperature_C"] = round(s.temperature * 10.0) / 10.0;
  doc["reynolds"] = (int)s.reynoldsNumber;
  doc["friction_factor"] = round(s.frictionFactor * 10000.0) / 10000.0;
  doc["head_loss_m"] = round(s.headLoss * 100.0) / 100.0;
  doc["degrading"] = s.degrading ? 1 : 0;
  doc["leak_coeff_pct"] = round(s.leakCoeff * 100.0 * 10.0) / 10.0;
  doc["blockage_coeff_pct"] = round(s.blockageCoeff * 100.0 * 10.0) / 10.0;
  
  // Serialize to string
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  
  // Publish to individual sensor topic
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%d", MQTT_TOPIC, s.sensorId);
  mqttClient.publish(topic, jsonBuffer);
}

void publishAllSensors() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  
  // Calculate number of chunks needed
  int totalChunks = (activeSensorCount + MAX_SENSORS_PER_MESSAGE - 1) / MAX_SENSORS_PER_MESSAGE;
  
  for (int chunk = 0; chunk < totalChunks; chunk++) {
    mqttClient.loop(); // Prevent keepalive timeout
    
    int startIdx = chunk * MAX_SENSORS_PER_MESSAGE;
    if (startIdx >= activeSensorCount) {
    continue;
  }
    int endIdx = min(startIdx + MAX_SENSORS_PER_MESSAGE, activeSensorCount);
    
    // Clear and reuse static document (NO heap allocation!)
    mqttDoc.clear();
    
    mqttDoc["timestamp"] = millis() / 1000;
    mqttDoc["dt"] = 1.0;
    mqttDoc["chunk"] = chunk;
    mqttDoc["total_chunks"] = totalChunks;
    
    JsonArray sensorsArray = mqttDoc.createNestedArray("sensors");
    
    for (int i = startIdx; i < endIdx; i++) {
      Sensor &s = sensors[i];
      
      JsonObject sensorObj = sensorsArray.createNestedObject();
      sensorObj["sensor_id"] = s.sensorId;
      sensorObj["pipe_id"] = s.pipeId;
      sensorObj["type"] = (s.type == FRESH_WATER) ? "fresh" : "sewage";
      sensorObj["pressure_bar"] = round(s.pressure * 1000.0) / 1000.0;
      sensorObj["level_pct"] = round(s.level * 1000.0) / 1000.0;
      sensorObj["valve"] = s.valveOpen ? 1 : 0;
      sensorObj["flow_rate_Ls"] = round(s.flowRate * 1000.0 * 100.0) / 100.0;
      sensorObj["velocity_ms"] = round(s.velocity * 100.0) / 100.0;
      sensorObj["temperature_C"] = round(s.temperature * 10.0) / 10.0;
      sensorObj["reynolds"] = (int)s.reynoldsNumber;
      sensorObj["friction_factor"] = round(s.frictionFactor * 10000.0) / 10000.0;
      sensorObj["head_loss_m"] = round(s.headLoss * 100.0) / 100.0;
      sensorObj["degrading"] = s.degrading ? 1 : 0;
      sensorObj["leak_coeff_pct"] = round(s.leakCoeff * 100.0 * 10.0) / 10.0;
      sensorObj["blockage_coeff_pct"] = round(s.blockageCoeff * 100.0 * 10.0) / 10.0;
    }
    
    // Serialize directly to static buffer 
    size_t jsonSize = serializeJson(mqttDoc, mqttBuffer, sizeof(mqttBuffer));
    
    if (jsonSize > 0 && jsonSize < sizeof(mqttBuffer)) {
      if (totalChunks == 1) {
        snprintf(topicBuffer, sizeof(topicBuffer), "%s/all", MQTT_TOPIC);
      } else {
        snprintf(topicBuffer, sizeof(topicBuffer), "%s/all/chunk_%d", MQTT_TOPIC, chunk);
      }
      
      bool published = mqttClient.publish(topicBuffer, mqttBuffer);
      
      if (!published) {
        Serial.print("⚠ Publish failed for chunk ");
        Serial.println(chunk);
      }
    } else {
      Serial.print("⚠ JSON too large for chunk ");
      Serial.print(chunk);
      Serial.print(" (");
      Serial.print(jsonSize);
      Serial.println(" bytes)");
    }
    
    mqttClient.loop(); 
    
    if (chunk < totalChunks - 1) {
      delay(MQTT_BATCH_DELAY);
    }
  }
  
  publishSystemSummary();
}

void publishSystemSummary() {
  StaticJsonDocument<1024> doc;
  
  int freshCount = 0, sewageCount = 0;
  int degradingCount = 0, closedValves = 0;
  float avgPressureFresh = 0, avgLevelSewage = 0;
  float totalFlowFresh = 0, totalFlowSewage = 0;
  
  for (int i = 0; i < activeSensorCount; i++){
    if (sensors[i].type == FRESH_WATER) {
      freshCount++;
      avgPressureFresh += sensors[i].pressure;
      totalFlowFresh += sensors[i].flowRate * 1000.0;
    } else {
      sewageCount++;
      avgLevelSewage += sensors[i].level;
      totalFlowSewage += sensors[i].flowRate * 1000.0;
    }
    if (sensors[i].degrading) degradingCount++;
    if (!sensors[i].valveOpen) closedValves++;
  }
  
  avgPressureFresh /= (freshCount > 0 ? freshCount : 1);
  avgLevelSewage /= (sewageCount > 0 ? sewageCount : 1);
  
  doc["timestamp"] = millis() / 1000;
  doc["total_sensors"] = activeSensorCount;
  doc["fresh_water_count"] = freshCount;
  doc["sewage_count"] = sewageCount;
  doc["degrading_count"] = degradingCount;
  doc["closed_valves"] = closedValves;
  doc["avg_pressure_fresh_bar"] = round(avgPressureFresh * 1000.0) / 1000.0;
  doc["avg_level_sewage_pct"] = round(avgLevelSewage * 100.0) / 100.0;
  doc["total_flow_fresh_Ls"] = round(totalFlowFresh * 100.0) / 100.0;
  doc["total_flow_sewage_Ls"] = round(totalFlowSewage * 100.0) / 100.0;
  
  serializeJson(doc, mqttBuffer, sizeof(mqttBuffer));
  mqttClient.publish("pipes/system/summary3", mqttBuffer); //ESP_CHANGE
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(esp_random());
  
  Serial.println("===========================================");
  Serial.print("   INITIALIZING ");
  Serial.print(activeSensorCount);
  Serial.println(" SENSORS");
  Serial.println("===========================================");
  

  setupWiFi();
  

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage); 
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE); 
  mqttClient.setKeepAlive(60);    
  
  reconnectMQTT();
  
  Serial.println("===========================================");
  Serial.println("WAITING FOR CONFIGURATION FROM LINUX...");
  Serial.println("Topic: pipes/config/init");
  Serial.println("Timeout: 5 minutes");
  Serial.println("===========================================");
  

  unsigned long waitStart = millis();
  while (!configReceived && (millis() - waitStart < CONFIG_WAIT_TIME)) {
    mqttClient.loop();
    delay(100);
    
    if ((millis() - waitStart) % 10000 < 100) {
      int remaining = (CONFIG_WAIT_TIME - (millis() - waitStart)) / 1000;
      Serial.print("Waiting... ");
      Serial.print(remaining);
      Serial.println(" seconds remaining");
    }
  }
  
  if (configReceived) {
    Serial.println("===========================================");
    Serial.println("✓ CONFIGURATION RECEIVED FROM LINUX!");
    Serial.println("===========================================");
  } else {
    Serial.println("===========================================");
    Serial.println("TIMEOUT! Using default configuration");
    Serial.println("===========================================");
  
    initSystem();
  }
  verifySensorUniqueness();
  
  Serial.print("System ready - publishing ");
  Serial.print(TOTAL_PIPES);
  Serial.println(" sensors as JSON to MQTT every second");
}

void loop() {

  if (!initializedFromConfig) {
  mqttClient.loop();
  return;
}


  unsigned long now = millis();
  
  // Check for degradation event
  if (now >= nextDegradeTime) {
    activateRandomDegradation();
  }
  
  // Update at fixed interval
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;
    
    // Run physics simulation 
    for (int step = 0; step < PHYSICS_SUBSTEPS; step++) {
      updatePhysics(PHYSICS_DT);
    }
    
    // Publish all sensors to MQTT
    unsigned long mqttStart = millis();
    Serial.print("Publishing ");
    Serial.print(TOTAL_PIPES);
    Serial.print(" sensors to MQTT... ");
    publishAllSensors();
    unsigned long mqttTime = millis() - mqttStart;
    Serial.print("done in ");
    Serial.print(mqttTime);
    Serial.println(" ms");
    

    if (TOTAL_PIPES <= 100) {

      Serial.println("========================================");
      Serial.println("          ALL SENSOR READINGS          ");
      Serial.println("========================================");
      for (int i = 0; i < activeSensorCount; i++) {
        printSensor(i);
      }
      Serial.println("========================================");
    } else {
 
      Serial.println("========================================");
      Serial.println("        SAMPLE SENSOR READINGS         ");
      Serial.println("========================================");
      for(int i=0;i< activeSensorCount ;i++){
        printSensor(i);
      }
      Serial.println("========================================");
      int degrading = 0;
      for (int i = 0; i < activeSensorCount ; i++) {
        if (sensors[i].degrading) degrading++;
      }
      Serial.print("Total Sensors: ");
      Serial.print(activeSensorCount );
      Serial.print(" | Degrading: ");
      Serial.println(degrading);
      Serial.println("========================================");
    }
    Serial.println();
  }
} 