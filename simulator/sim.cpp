/*
 * SCADA-GRADE WATER & SEWAGE DIGITAL TWIN
 * ========================================
 * 
 * STRICTLY EMBEDDED-SYSTEM DRIVEN
 * ALL DATA FROM EXTERNAL SOURCES
 * NO INTERNAL SIMULATION LOGIC
 * 
 * COMPILE:
 * g++ -o water_scada water_scada_twin.cpp -lGL -lGLU -lglut -lm -O2 -std=c++11
 * 
 * RUN:
 * ./water_scada
 */

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <vector>
#include <map>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>

// ============================================================================
// ENGINEERING CONSTANTS
// ============================================================================

const int BUILDINGS_PER_CLUSTER = 10;
const float CITY_GRID_SPACING = 20.0f;
const float CLUSTER_SPACING = 60.0f;

const int FLOOR_OPTIONS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15};
const int NUM_FLOOR_OPTIONS = 11;
const float FLOOR_HEIGHT = 3.0f;
const float BUILDING_FOOTPRINT = 10.0f;

// Pipe diameters in meters
const float TRUNK_DIAMETER = 0.8f;
const float SECONDARY_DIAMETER = 0.4f;
const float RING_DIAMETER = 0.25f;
const float SERVICE_DIAMETER = 0.05f;

// ============================================================================
// UTILITY STRUCTURES
// ============================================================================

struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const { 
        float l = length(); 
        return l > 0.001f ? Vec3(x/l, y/l, z/l) : Vec3(0, 0, 0); 
    }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }
};

struct Color {
    float r, g, b;
    Color(float r = 1, float g = 1, float b = 1) : r(r), g(g), b(b) {}
};

// ============================================================================
// SENSOR & VALVE STRUCTURES (FIRST-CLASS OBJECTS)
// ============================================================================

enum SensorType {
    PRESSURE_SENSOR,
    FLOW_SENSOR,
    LEVEL_SENSOR,
    SEWAGE_LEVEL_SENSOR
};

struct Sensor {
    int id;
    std::string name;
    SensorType type;
    Vec3 position;
    float value;              // Current reading from embedded system
    float minValue, maxValue; // Range
    bool active;
    int connectedPipeID;
    
    Sensor(int id, std::string name, SensorType type, Vec3 pos)
        : id(id), name(name), type(type), position(pos),
          value(0), minValue(0), maxValue(100), active(true),
          connectedPipeID(-1) {}
    
    Color getColor() const {
        if (!active) return Color(0.3f, 0.3f, 0.3f);
        
        float ratio = (value - minValue) / (maxValue - minValue);
        ratio = std::max(0.0f, std::min(1.0f, ratio));
        
        if (type == PRESSURE_SENSOR) {
            return Color(0.2f, 0.5f + ratio * 0.5f, 1.0f - ratio * 0.3f);
        } else if (type == FLOW_SENSOR) {
            return Color(0.3f + ratio * 0.6f, 0.8f, 0.3f + ratio * 0.3f);
        } else if (type == LEVEL_SENSOR) {
            return Color(0.2f + ratio * 0.6f, 0.4f + ratio * 0.4f, 0.9f);
        } else {
            return Color(0.6f + ratio * 0.3f, 0.4f, 0.2f);
        }
    }
};

struct Valve {
    int id;
    std::string name;
    Vec3 position;
    float openPercentage;  // 0-100% from Python control
    bool isControlled;     // true = external control, false = manual
    int connectedPipeID;
    
    Valve(int id, std::string name, Vec3 pos)
        : id(id), name(name), position(pos),
          openPercentage(100.0f), isControlled(false),
          connectedPipeID(-1) {}
    
    Color getColor() const {
        if (openPercentage > 80.0f) {
            return Color(0.2f, 0.9f, 0.2f);  // Green = open
        } else if (openPercentage > 20.0f) {
            return Color(0.9f, 0.9f, 0.2f);  // Yellow = partial
        } else {
            return Color(0.9f, 0.2f, 0.2f);  // Red = closed
        }
    }
};

// ============================================================================
// PIPE MODEL
// ============================================================================

enum PipeType { 
    TRUNK_MAIN, SECONDARY_MAIN, RING_MAIN, SERVICE_PIPE,
    SEWAGE_LATERAL, SEWAGE_COLLECTOR, SEWAGE_INTERCEPTOR
};

struct Pipe {
    int id;
    Vec3 start, end;
    PipeType type;
    float diameter;
    float length;
    
    // Embedded system inputs
    float embeddedPressure;   // kPa
    float embeddedFlow;       // L/s
    bool embeddedLeakFlag;
    float embeddedLeakRate;   // L/s
    
    Pipe(int id, Vec3 s, Vec3 e, PipeType t, float diam)
        : id(id), start(s), end(e), type(t), diameter(diam),
          embeddedPressure(0), embeddedFlow(0),
          embeddedLeakFlag(false), embeddedLeakRate(0) {
        length = (end - start).length();
    }
    
    Color getColor() const {
        if (type >= SEWAGE_LATERAL) {
            // SEWAGE: BRIGHT BROWN/ORANGE for visibility
            float flowIntensity = std::min(1.0f, embeddedFlow / 30.0f);
            return Color(0.85f, 0.55f + flowIntensity * 0.25f, 0.15f);
        } else {
            // WATER: Blue gradient based on pressure
            float pressureRatio = std::min(1.0f, embeddedPressure / 500.0f);
            return Color(0.15f, 0.4f + pressureRatio * 0.5f, 0.75f + pressureRatio * 0.25f);
        }
    }
};

// ============================================================================
// BUILDING MODEL
// ============================================================================

struct Building {
    int id, clusterID;
    Vec3 position;
    int numFloors;
    float height;
    
    // Sensors and valves (POINT1_SUB)
    int inletValveID;
    int pressureSensorID;
    int flowSensorID;
    int sewageOutletSensorID;
    
    int servicePipeID;
    int sewerPipeID;
    
    Building(int id, Vec3 pos, int floors, int cluster)
        : id(id), clusterID(cluster), position(pos), numFloors(floors),
          height(floors * FLOOR_HEIGHT),
          inletValveID(-1), pressureSensorID(-1),
          flowSensorID(-1), sewageOutletSensorID(-1),
          servicePipeID(-1), sewerPipeID(-1) {}
    
    Color getBuildingColor() const {
        return Color(0.55f, 0.55f, 0.6f);
    }
};

// ============================================================================
// CLUSTER MODEL
// ============================================================================

struct Cluster {
    int id;
    Vec3 centerPos;
    std::vector<int> buildingIDs;
    std::vector<int> ringPipeIDs;
    std::vector<int> servicePipeIDs;
    std::vector<int> sewerPipeIDs;
    
    // Cluster-level sensors/valves (POINT1_MAIN)
    int clusterInletValveID;
    int clusterPressureSensorID;
    int clusterFlowSensorID;
    int clusterSewerSensorID;
    
    int secondaryMainID;
    int sewageCollectorID;
    
    bool highlighted;
    
    Cluster(int id, Vec3 center)
        : id(id), centerPos(center),
          clusterInletValveID(-1), clusterPressureSensorID(-1),
          clusterFlowSensorID(-1), clusterSewerSensorID(-1),
          secondaryMainID(-1), sewageCollectorID(-1),
          highlighted(false) {}
};

// ============================================================================
// CITY NETWORK
// ============================================================================

struct CityNetwork {
    std::vector<Building> buildings;
    std::vector<Cluster> clusters;
    std::vector<Pipe> pipes;
    std::vector<Sensor> sensors;
    std::vector<Valve> valves;
    
    Vec3 reservoirPos, stpPos;
    
    // Reservoir sensors
    int reservoirValveID;
    int reservoirPressureSensorID;
    int reservoirLevelSensorID;
    
    // STP sensors
    int stpInletLevelSensorID;
    int stpFlowSensorID;
    
    float simulationTime;
    bool embeddedDataConnected;
    
    CityNetwork()
        : reservoirPos(-100, 35, -100),
          stpPos(100, 0, 100),
          reservoirValveID(-1),
          reservoirPressureSensorID(-1),
          reservoirLevelSensorID(-1),
          stpInletLevelSensorID(-1),
          stpFlowSensorID(-1),
          simulationTime(0),
          embeddedDataConnected(false) {}
    
    void generateCity(int numBuildings);
    void generateWaterNetwork();
    void generateSewageNetwork();
    void createSensorsAndValves();
    void loadEmbeddedPressureData(const char* filename);
    void loadEmbeddedLeakData(const char* filename);
    void loadValveControlData(const char* filename);
    void updateSimulation(float dt);
};

// ============================================================================
// CITY GENERATION
// ============================================================================

void CityNetwork::generateCity(int numBuildings) {
    buildings.clear();
    clusters.clear();
    pipes.clear();
    sensors.clear();
    valves.clear();
    
    std::cout << "\n=== GENERATING SCADA-GRADE CITY ===\n";
    std::cout << "Target buildings: " << numBuildings << "\n";
    
    int numClusters = (numBuildings + BUILDINGS_PER_CLUSTER - 1) / BUILDINGS_PER_CLUSTER;
    int clustersPerRow = (int)ceil(sqrt(numClusters));
    
    // Create clusters
    for (int i = 0; i < numClusters; ++i) {
        int clusterRow = i / clustersPerRow;
        int clusterCol = i % clustersPerRow;
        
        Vec3 clusterCenter(
            clusterCol * CLUSTER_SPACING - (clustersPerRow - 1) * CLUSTER_SPACING * 0.5f,
            0,
            clusterRow * CLUSTER_SPACING - (clustersPerRow - 1) * CLUSTER_SPACING * 0.5f
        );
        
        clusters.push_back(Cluster(i, clusterCenter));
    }
    
    // Create buildings
    int buildingID = 0;
    for (int clusterIdx = 0; clusterIdx < numClusters && buildingID < numBuildings; ++clusterIdx) {
        Cluster& cluster = clusters[clusterIdx];
        int buildingsInCluster = std::min(BUILDINGS_PER_CLUSTER, numBuildings - buildingID);
        int cols = buildingsInCluster <= 5 ? buildingsInCluster : (buildingsInCluster <= 8 ? 4 : 5);
        int rows = (buildingsInCluster + cols - 1) / cols;
        
        for (int b = 0; b < buildingsInCluster; ++b) {
            int row = b / cols;
            int col = b % cols;
            
            Vec3 buildingPos = cluster.centerPos + Vec3(
                col * CITY_GRID_SPACING - (cols - 1) * CITY_GRID_SPACING * 0.5f,
                0,
                row * CITY_GRID_SPACING - (rows - 1) * CITY_GRID_SPACING * 0.5f
            );
            
            int floors = FLOOR_OPTIONS[rand() % NUM_FLOOR_OPTIONS];
            buildings.push_back(Building(buildingID, buildingPos, floors, clusterIdx));
            cluster.buildingIDs.push_back(buildingID);
            buildingID++;
        }
    }
    
    std::cout << "Created " << buildings.size() << " buildings in " << clusters.size() << " clusters\n";
    
    generateWaterNetwork();
    generateSewageNetwork();
    createSensorsAndValves();
    
    std::cout << "Total pipes: " << pipes.size() << "\n";
    std::cout << "Total sensors: " << sensors.size() << "\n";
    std::cout << "Total valves: " << valves.size() << "\n";
    std::cout << "=== CITY GENERATION COMPLETE ===\n\n";
}

void CityNetwork::generateWaterNetwork() {
    int pipeID = 0;
    
    Vec3 trunkStart = reservoirPos + Vec3(0, -30, 0);
    Vec3 trunkEnd(0, 0, 0);
    pipes.push_back(Pipe(pipeID++, trunkStart, trunkEnd, TRUNK_MAIN, TRUNK_DIAMETER));
    
    for (auto& cluster : clusters) {
        Vec3 secondaryEnd = cluster.centerPos + Vec3(0, 1, 0);
        cluster.secondaryMainID = pipeID;
        pipes.push_back(Pipe(pipeID++, trunkEnd, secondaryEnd, SECONDARY_MAIN, SECONDARY_DIAMETER));
    }
    
    for (auto& cluster : clusters) {
        if (cluster.buildingIDs.empty()) continue;
        
        Vec3 center = cluster.centerPos + Vec3(0, 0.5f, 0);
        float ringRadius = CITY_GRID_SPACING * 1.5f;
        
        Vec3 ringPoints[4] = {
            center + Vec3(-ringRadius, 0, -ringRadius),
            center + Vec3(ringRadius, 0, -ringRadius),
            center + Vec3(ringRadius, 0, ringRadius),
            center + Vec3(-ringRadius, 0, ringRadius)
        };
        
        for (int i = 0; i < 4; ++i) {
            int next = (i + 1) % 4;
            cluster.ringPipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(pipeID++, ringPoints[i], ringPoints[next], RING_MAIN, RING_DIAMETER));
        }
        
        pipes.push_back(Pipe(pipeID++, center, ringPoints[0], RING_MAIN, RING_DIAMETER));
    }
    
    for (auto& cluster : clusters) {
        for (int bid : cluster.buildingIDs) {
            Building& bldg = buildings[bid];
            
            float minDist = 1e9f;
            Vec3 nearestRingPoint;
            for (int ringPipeID : cluster.ringPipeIDs) {
                float d1 = (pipes[ringPipeID].start - bldg.position).length();
                float d2 = (pipes[ringPipeID].end - bldg.position).length();
                if (d1 < minDist) { minDist = d1; nearestRingPoint = pipes[ringPipeID].start; }
                if (d2 < minDist) { minDist = d2; nearestRingPoint = pipes[ringPipeID].end; }
            }
            
            Vec3 serviceEnd = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.25f, 0.5f, BUILDING_FOOTPRINT * 0.25f);
            bldg.servicePipeID = pipeID;
            cluster.servicePipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(pipeID++, nearestRingPoint, serviceEnd, SERVICE_PIPE, SERVICE_DIAMETER));
        }
    }
}

void CityNetwork::generateSewageNetwork() {
    int pipeID = pipes.size();
    
    for (auto& cluster : clusters) {
        Vec3 collectorPoint = cluster.centerPos + Vec3(0, -1.5f, CLUSTER_SPACING * 0.6f);
        
        for (int bid : cluster.buildingIDs) {
            Building& bldg = buildings[bid];
            Vec3 lateralStart = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.75f, 0.5f, BUILDING_FOOTPRINT * 0.75f);
            Vec3 lateralEnd = collectorPoint;
            lateralEnd.y = lateralStart.y - 1.0f;
            
            bldg.sewerPipeID = pipeID;
            cluster.sewerPipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(pipeID++, lateralStart, lateralEnd, SEWAGE_LATERAL, SERVICE_DIAMETER * 1.5f));
        }
        
        Vec3 collectorEnd = Vec3(collectorPoint.x, -2.0f, stpPos.z - 20.0f);
        cluster.sewageCollectorID = pipeID;
        pipes.push_back(Pipe(pipeID++, collectorPoint, collectorEnd, SEWAGE_COLLECTOR, SECONDARY_DIAMETER));
    }
    
    Vec3 interceptorStart(0, -2.5f, stpPos.z - 20.0f);
    Vec3 interceptorEnd = stpPos + Vec3(0, 1.0f, -10.0f);
    pipes.push_back(Pipe(pipeID++, interceptorStart, interceptorEnd, SEWAGE_INTERCEPTOR, TRUNK_DIAMETER));
}

void CityNetwork::createSensorsAndValves() {
    int sensorID = 0;
    int valveID = 0;
    
    // RESERVOIR sensors and valve
    reservoirValveID = valveID;
    valves.push_back(Valve(valveID++, "RESERVOIR_MAIN_VALVE", reservoirPos + Vec3(0, -28, 0)));
    
    reservoirPressureSensorID = sensorID;
    sensors.push_back(Sensor(sensorID++, "RES_PRESSURE", PRESSURE_SENSOR, reservoirPos + Vec3(3, -25, 0)));
    sensors.back().maxValue = 500;
    
    reservoirLevelSensorID = sensorID;
    sensors.push_back(Sensor(sensorID++, "RES_LEVEL", LEVEL_SENSOR, reservoirPos + Vec3(0, 15, 0)));
    sensors.back().maxValue = 100;
    
    // PER CLUSTER sensors/valves (POINT1_MAIN)
    for (auto& cluster : clusters) {
        Vec3 clusterValvePos = cluster.centerPos + Vec3(0, 2, 0);
        
        cluster.clusterInletValveID = valveID;
        valves.push_back(Valve(valveID++, "CLUSTER_" + std::to_string(cluster.id) + "_VALVE", clusterValvePos));
        valves.back().connectedPipeID = cluster.secondaryMainID;
        
        cluster.clusterPressureSensorID = sensorID;
        sensors.push_back(Sensor(sensorID++, "CL" + std::to_string(cluster.id) + "_PRESS", PRESSURE_SENSOR, clusterValvePos + Vec3(2, 0, 0)));
        sensors.back().maxValue = 400;
        
        cluster.clusterFlowSensorID = sensorID;
        sensors.push_back(Sensor(sensorID++, "CL" + std::to_string(cluster.id) + "_FLOW", FLOW_SENSOR, clusterValvePos + Vec3(-2, 0, 0)));
        sensors.back().maxValue = 100;
        
        cluster.clusterSewerSensorID = sensorID;
        Vec3 sewerSensorPos = cluster.centerPos + Vec3(0, -1, CLUSTER_SPACING * 0.6f);
        sensors.push_back(Sensor(sensorID++, "CL" + std::to_string(cluster.id) + "_SEWER", SEWAGE_LEVEL_SENSOR, sewerSensorPos));
        sensors.back().maxValue = 50;
    }
    
    // PER BUILDING sensors/valves (POINT1_SUB)
    for (auto& bldg : buildings) {
        Vec3 valvePos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.25f, 1.5f, BUILDING_FOOTPRINT * 0.25f);
        
        bldg.inletValveID = valveID;
        valves.push_back(Valve(valveID++, "BLD_" + std::to_string(bldg.id) + "_VALVE", valvePos));
        valves.back().connectedPipeID = bldg.servicePipeID;
        
        bldg.pressureSensorID = sensorID;
        sensors.push_back(Sensor(sensorID++, "B" + std::to_string(bldg.id) + "_PRESS", PRESSURE_SENSOR, valvePos + Vec3(0.5f, 0, 0)));
        sensors.back().maxValue = 300;
        
        bldg.flowSensorID = sensorID;
        sensors.push_back(Sensor(sensorID++, "B" + std::to_string(bldg.id) + "_FLOW", FLOW_SENSOR, valvePos + Vec3(-0.5f, 0, 0)));
        sensors.back().maxValue = 20;
        
        bldg.sewageOutletSensorID = sensorID;
        Vec3 sewerPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.75f, 0.8f, BUILDING_FOOTPRINT * 0.75f);
        sensors.push_back(Sensor(sensorID++, "B" + std::to_string(bldg.id) + "_SEW", SEWAGE_LEVEL_SENSOR, sewerPos));
        sensors.back().maxValue = 10;
    }
    
    // STP sensors
    stpInletLevelSensorID = sensorID;
    sensors.push_back(Sensor(sensorID++, "STP_INLET_LEVEL", LEVEL_SENSOR, stpPos + Vec3(0, 3, -10)));
    sensors.back().maxValue = 100;
    
    stpFlowSensorID = sensorID;
    sensors.push_back(Sensor(sensorID++, "STP_FLOW", FLOW_SENSOR, stpPos + Vec3(5, 2, -10)));
    sensors.back().maxValue = 200;
}

// ============================================================================
// EMBEDDED DATA LOADING (CSV/JSON placeholder)
// ============================================================================

void CityNetwork::loadEmbeddedPressureData(const char* filename) {
    std::cout << "Loading embedded pressure data from: " << filename << "\n";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "  WARNING: File not found, using defaults\n";
        embeddedDataConnected = false;
        return;
    }
    
    std::string line;
    std::getline(file, line); // Skip header
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        int sensorID;
        float value;
        
        std::getline(ss, item, ',');
        sensorID = std::stoi(item);
        std::getline(ss, item, ',');
        value = std::stof(item);
        
        if (sensorID < (int)sensors.size()) {
            sensors[sensorID].value = value;
        }
    }
    
    file.close();
    embeddedDataConnected = true;
    std::cout << "  Loaded pressure data successfully\n";
}

void CityNetwork::loadEmbeddedLeakData(const char* filename) {
    std::cout << "Loading embedded leak data from: " << filename << "\n";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "  WARNING: File not found, no leaks flagged\n";
        return;
    }
    
    std::string line;
    std::getline(file, line); // Skip header
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        int pipeID;
        bool hasLeak;
        float leakRate;
        
        std::getline(ss, item, ',');
        pipeID = std::stoi(item);
        std::getline(ss, item, ',');
        hasLeak = (std::stoi(item) == 1);
        std::getline(ss, item, ',');
        leakRate = std::stof(item);
        
        if (pipeID < (int)pipes.size()) {
            pipes[pipeID].embeddedLeakFlag = hasLeak;
            pipes[pipeID].embeddedLeakRate = leakRate;
            if (hasLeak) {
                std::cout << "  LEAK FLAGGED: Pipe " << pipeID << " (" << leakRate << " L/s)\n";
            }
        }
    }
    
    file.close();
}

void CityNetwork::loadValveControlData(const char* filename) {
    std::cout << "Loading valve control data from: " << filename << "\n";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "  WARNING: File not found, valves remain at current state\n";
        return;
    }
    
    std::string line;
    std::getline(file, line); // Skip header
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        int valveID;
        float openPercentage;
        
        std::getline(ss, item, ',');
        valveID = std::stoi(item);
        std::getline(ss, item, ',');
        openPercentage = std::stof(item);
        
        if (valveID < (int)valves.size()) {
            valves[valveID].openPercentage = openPercentage;
            valves[valveID].isControlled = true;
            std::cout << "  Valve " << valveID << " set to " << openPercentage << "%\n";
        }
    }
    
    file.close();
}

void CityNetwork::updateSimulation(float dt) {
    simulationTime += dt;
    
    // Apply valve states to pipe flow
    for (const auto& valve : valves) {
        if (valve.connectedPipeID >= 0 && valve.connectedPipeID < (int)pipes.size()) {
            float flowFactor = valve.openPercentage / 100.0f;
            pipes[valve.connectedPipeID].embeddedFlow *= flowFactor;
        }
    }
}

// ============================================================================
// RENDERING FUNCTIONS
// ============================================================================

CityNetwork city;
int windowWidth = 1600, windowHeight = 1000;
float camAngle = 45.0f, camElevation = 45.0f, camDistance = 200.0f;
int lastMouseX = 0, lastMouseY = 0;
bool mouseLeftDown = false;

bool showBuildings = true;
bool showWaterNetwork = true;
bool showSewageNetwork = true;
bool showSensors = true;
bool showValves = true;
bool showLeaks = true;
bool showFlowParticles = false;
bool showSensorLabels = false;
int highlightedCluster = -1;
int currentBuildingCount = 50;

void drawCylinder(Vec3 start, Vec3 end, float radius, Color color) {
    glColor3f(color.r, color.g, color.b);
    
    Vec3 dir = end - start;
    float length = dir.length();
    if (length < 0.001f) return;
    
    dir = dir.normalized();
    Vec3 up(0, 1, 0);
    if (fabs(dir.y) > 0.99f) up = Vec3(1, 0, 0);
    Vec3 right = dir.cross(up).normalized();
    up = right.cross(dir).normalized();
    
    glPushMatrix();
    glTranslatef(start.x, start.y, start.z);
    
    float matrix[16] = {
        right.x, right.y, right.z, 0,
        up.x, up.y, up.z, 0,
        dir.x, dir.y, dir.z, 0,
        0, 0, 0, 1
    };
    glMultMatrixf(matrix);
    
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, radius, radius, length, 12, 1);
    gluDeleteQuadric(quad);
    
    glPopMatrix();
}

void drawSphere(Vec3 pos, float radius, Color color) {
    glColor3f(color.r, color.g, color.b);
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glutSolidSphere(radius, 16, 16);
    glPopMatrix();
}

void drawBox(Vec3 pos, float w, float h, float d, Color color) {
    glColor3f(color.r, color.g, color.b);
    glPushMatrix();
    glTranslatef(pos.x + w/2, pos.y + h/2, pos.z + d/2);
    glScalef(w, h, d);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void drawSensor(const Sensor& sensor) {
    Color col = sensor.getColor();
    
    // Blinking effect if active
    float pulse = sensor.active ? (0.7f + 0.3f * sinf(city.simulationTime * 3.0f)) : 0.3f;
    drawSphere(sensor.position, 0.6f, Color(col.r * pulse, col.g * pulse, col.b * pulse));
    
    // Sensor base
    drawBox(Vec3(sensor.position.x - 0.3f, sensor.position.y - 0.8f, sensor.position.z - 0.3f),
            0.6f, 0.5f, 0.6f, Color(0.4f, 0.4f, 0.4f));
    
    // Label
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 1);
        glRasterPos3f(sensor.position.x, sensor.position.y + 1.2f, sensor.position.z);
        for (char c : sensor.name) {
            glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
        }
        glEnable(GL_LIGHTING);
    }
}

void drawValve(const Valve& valve) {
    Color col = valve.getColor();
    
    // Valve body - cube
    drawBox(Vec3(valve.position.x - 0.8f, valve.position.y - 0.8f, valve.position.z - 0.8f),
            1.6f, 1.6f, 1.6f, col);
    
    // Valve wheel
    glDisable(GL_LIGHTING);
    glColor3f(col.r * 0.7f, col.g * 0.7f, col.b * 0.7f);
    glPushMatrix();
    glTranslatef(valve.position.x, valve.position.y + 1.2f, valve.position.z);
    glRotatef(90, 1, 0, 0);
    glutSolidTorus(0.2, 0.5, 8, 16);
    glPopMatrix();
    glEnable(GL_LIGHTING);
    
    // Label
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 0);
        glRasterPos3f(valve.position.x, valve.position.y + 2.0f, valve.position.z);
        std::stringstream ss;
        ss << valve.name << " [" << (int)valve.openPercentage << "%]";
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
        }
        glEnable(GL_LIGHTING);
    }
}

void drawPipe(const Pipe& pipe) {
    Color color = pipe.getColor();
    float radius = pipe.diameter / 2.0f;
    
    // SEWAGE PIPES: Make them THICKER and BRIGHTER for visibility
    if (pipe.type >= SEWAGE_LATERAL) {
        radius *= 2.5f;  // Much thicker
        // SUPER BRIGHT ORANGE/BROWN
        float flowIntensity = std::min(1.0f, pipe.embeddedFlow / 30.0f);
        color = Color(0.95f, 0.65f + flowIntensity * 0.25f, 0.15f);
    }
    
    drawCylinder(pipe.start, pipe.end, radius, color);
    
    // Junction caps
    drawSphere(pipe.start, radius * 1.3f, color);
    drawSphere(pipe.end, radius * 1.3f, color);
    
    // LEAK VISUALIZATION - ONLY if embedded flag is set
    if (pipe.embeddedLeakFlag && showLeaks) {
        Vec3 mid = (pipe.start + pipe.end) * 0.5f;
        
        // Large red pulsing sphere
        float pulse = 0.8f + 0.4f * sinf(city.simulationTime * 5.0f);
        drawSphere(mid, 2.0f * pulse, Color(1, 0, 0));
        
        // Warning cone
        glDisable(GL_LIGHTING);
        glColor3f(1, 0.2f, 0);
        glPushMatrix();
        glTranslatef(mid.x, mid.y + 4, mid.z);
        glRotatef(-90, 1, 0, 0);
        glutSolidCone(1.5, 3.0, 8, 2);
        glPopMatrix();
        
        // Leak spray particles
        glPointSize(6.0f);
        glBegin(GL_POINTS);
        for (int i = 0; i < 40; i++) {
            float angle = i * M_PI * 2.0f / 40.0f + city.simulationTime * 2.0f;
            float dist = 2.5f + 1.5f * sinf(city.simulationTime * 3.0f + i);
            Vec3 particlePos = mid + Vec3(
                cos(angle) * dist, 
                sinf(angle * 3) * dist * 0.7f + 1.5f, 
                sin(angle) * dist
            );
            glColor3f(0.4f, 0.7f, 1.0f);
            glVertex3f(particlePos.x, particlePos.y, particlePos.z);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        
        // Leak label
        glDisable(GL_LIGHTING);
        glColor3f(1, 0, 0);
        glRasterPos3f(mid.x, mid.y + 6, mid.z);
        std::stringstream ss;
        ss << "LEAK: " << std::fixed << std::setprecision(1) << pipe.embeddedLeakRate << " L/s";
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
        glEnable(GL_LIGHTING);
    }
    
    // Flow particles
    if (showFlowParticles && pipe.embeddedFlow > 0.1f) {
        Vec3 dir = (pipe.end - pipe.start).normalized();
        float flowSpeed = pipe.embeddedFlow * 0.01f;
        
        glDisable(GL_LIGHTING);
        glPointSize(4.0f);
        
        if (pipe.type >= SEWAGE_LATERAL) {
            glColor3f(0.9f, 0.7f, 0.3f);
        } else {
            glColor3f(0.3f, 0.8f, 1.0f);
        }
        
        glBegin(GL_POINTS);
        for (int i = 0; i < 5; i++) {
            float t = fmodf(city.simulationTime * flowSpeed + i * 0.2f, 1.0f);
            Vec3 particlePos = pipe.start + (pipe.end - pipe.start) * t;
            glVertex3f(particlePos.x, particlePos.y, particlePos.z);
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
}

void drawBuilding(const Building& bldg) {
    Color color = bldg.getBuildingColor();
    drawBox(bldg.position, BUILDING_FOOTPRINT, bldg.height, BUILDING_FOOTPRINT, color);
    
    // Rooftop
    Color roofColor(0.4f, 0.4f, 0.45f);
    drawBox(Vec3(bldg.position.x, bldg.position.y + bldg.height, bldg.position.z),
            BUILDING_FOOTPRINT, 0.5f, BUILDING_FOOTPRINT, roofColor);
    
    // Water riser (blue)
    Vec3 riserPos = bldg.position + Vec3(2, 0, 2);
    drawCylinder(riserPos, riserPos + Vec3(0, bldg.height, 0), 0.12f, Color(0.4f, 0.6f, 0.9f));
    
    // Sewage drop (brown)
    Vec3 dropPos = bldg.position + Vec3(BUILDING_FOOTPRINT - 2, bldg.height, BUILDING_FOOTPRINT - 2);
    drawCylinder(dropPos, dropPos + Vec3(0, -bldg.height, 0), 0.15f, Color(0.85f, 0.6f, 0.2f));
}

void drawWaterReservoir(Vec3 pos, float waterLevel) {
    float tankRadius = 15.0f;
    float tankHeight = 25.0f;
    
    // Tank structure
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.3f, 0.5f, 0.7f, 0.3f);
    
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight/2, pos.z);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, tankRadius, tankRadius, tankHeight, 32, 1);
    gluDeleteQuadric(quad);
    glPopMatrix();
    glDisable(GL_BLEND);
    
    // Water surface
    float waterHeight = tankHeight * (waterLevel / 100.0f);
    glColor3f(0.2f, 0.4f, 0.8f);
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight/2 + waterHeight, pos.z);
    glRotatef(-90, 1, 0, 0);
    GLUquadric* disk = gluNewQuadric();
    gluDisk(disk, 0, tankRadius, 32, 1);
    gluDeleteQuadric(disk);
    glPopMatrix();
    
    // Pump house
    glColor3f(0.6f, 0.6f, 0.65f);
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight - 5, pos.z);
    glutSolidCube(12.0f);
    glPopMatrix();
}

void drawSewageTreatmentPlant(Vec3 pos) {
    glColor3f(0.4f, 0.3f, 0.2f);
    glPushMatrix();
    glTranslatef(pos.x - 20, pos.y + 2, pos.z);
    GLUquadric* quad1 = gluNewQuadric();
    gluCylinder(quad1, 12.0f, 12.0f, 4.0f, 32, 1);
    gluDeleteQuadric(quad1);
    glPopMatrix();
    
    glColor3f(0.35f, 0.4f, 0.3f);
    glPushMatrix();
    glTranslatef(pos.x + 10, pos.y + 2, pos.z - 10);
    glScalef(18, 4, 15);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    glColor3f(0.45f, 0.35f, 0.25f);
    glPushMatrix();
    glTranslatef(pos.x + 20, pos.y + 2, pos.z + 15);
    GLUquadric* quad2 = gluNewQuadric();
    gluCylinder(quad2, 10.0f, 10.0f, 4.0f, 32, 1);
    gluDeleteQuadric(quad2);
    glPopMatrix();
}

void drawText(float x, float y, const std::string& text, Color color = Color(0.9f, 0.9f, 0.9f)) {
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, windowWidth, 0, windowHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glColor3f(color.r, color.g, color.b);
    glRasterPos2f(x, y);
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
    }
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_LIGHTING);
}

void drawHUD() {
    std::stringstream ss;
    
    ss << "SCADA-GRADE WATER & SEWAGE DIGITAL TWIN";
    drawText(10, windowHeight - 25, ss.str());
    ss.str("");
    
    // Data source status
    if (city.embeddedDataConnected) {
        drawText(10, windowHeight - 50, "DATA SOURCE: [CONNECTED]", Color(0.2f, 0.9f, 0.2f));
    } else {
        drawText(10, windowHeight - 50, "DATA SOURCE: [DISCONNECTED - NO EMBEDDED DATA]", Color(0.9f, 0.2f, 0.2f));
    }
    
    ss << "Buildings: " << city.buildings.size() << " | Clusters: " << city.clusters.size() 
       << " | Pipes: " << city.pipes.size() << " | Sensors: " << city.sensors.size() 
       << " | Valves: " << city.valves.size();
    drawText(10, windowHeight - 75, ss.str());
    ss.str("");
    
    // Count active leaks from embedded data
    int leakCount = 0;
    std::string leakIDs = "";
    for (const auto& pipe : city.pipes) {
        if (pipe.embeddedLeakFlag) {
            leakCount++;
            if (!leakIDs.empty()) leakIDs += ", ";
            leakIDs += std::to_string(pipe.id);
        }
    }
    
    if (leakCount > 0) {
        ss << "ACTIVE LEAKS: " << leakCount << " [Pipe IDs: " << leakIDs << "]";
        drawText(10, windowHeight - 100, ss.str(), Color(1, 0, 0));
        ss.str("");
    }
    
    // Reservoir status
    if (city.reservoirLevelSensorID >= 0) {
        const Sensor& levelSensor = city.sensors[city.reservoirLevelSensorID];
        const Sensor& pressureSensor = city.sensors[city.reservoirPressureSensorID];
        const Valve& mainValve = city.valves[city.reservoirValveID];
        
        ss << "RESERVOIR: Level=" << std::fixed << std::setprecision(1) << levelSensor.value 
           << "% | Pressure=" << std::setprecision(0) << pressureSensor.value << " kPa | Main Valve=" 
           << std::setprecision(0) << mainValve.openPercentage << "%";
        drawText(10, windowHeight - 125, ss.str());
        ss.str("");
    }
    
    // Cluster info
    if (highlightedCluster >= 0 && highlightedCluster < (int)city.clusters.size()) {
        const Cluster& c = city.clusters[highlightedCluster];
        const Sensor& pressSensor = city.sensors[c.clusterPressureSensorID];
        const Sensor& flowSensor = city.sensors[c.clusterFlowSensorID];
        const Valve& valve = city.valves[c.clusterInletValveID];
        
        ss << "CLUSTER #" << c.id << ": Pressure=" << std::setprecision(0) << pressSensor.value 
           << " kPa | Flow=" << std::setprecision(1) << flowSensor.value 
           << " L/s | Valve=" << std::setprecision(0) << valve.openPercentage << "%";
        drawText(10, windowHeight - 150, ss.str(), Color(1, 1, 0));
        ss.str("");
    }
    
    // STP status
    if (city.stpInletLevelSensorID >= 0) {
        const Sensor& levelSensor = city.sensors[city.stpInletLevelSensorID];
        const Sensor& flowSensor = city.sensors[city.stpFlowSensorID];
        
        ss << "STP: Inlet Level=" << std::setprecision(1) << levelSensor.value 
           << "% | Flow=" << flowSensor.value << " L/s";
        drawText(10, windowHeight - 175, ss.str());
        ss.str("");
    }
    
    // Controls
    drawText(10, 200, "CONTROLS:");
    drawText(10, 180, "B: Buildings | W: Water | S: Sewage | V: Valves | N: Sensors | L: Leaks");
    drawText(10, 160, "F: Flow Particles | T: Sensor Labels | H: Highlight Cluster | R: Reset View");
    drawText(10, 140, "E: Load Embedded Pressure | K: Load Embedded Leaks | C: Load Valve Control");
    drawText(10, 120, "+/-: Buildings | Q: Quit | Mouse: Rotate/Zoom");
    drawText(10, 100, "");
    drawText(10, 80, "FILE FORMAT (embedded_pressure.csv): sensorID,value");
    drawText(10, 60, "FILE FORMAT (embedded_leaks.csv): pipeID,hasLeak,leakRate,timestamp");
    drawText(10, 40, "FILE FORMAT (valve_control.csv): valveID,openPercentage");
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    float camX = camDistance * cos(camElevation * M_PI / 180.0f) * sin(camAngle * M_PI / 180.0f);
    float camY = camDistance * sin(camElevation * M_PI / 180.0f);
    float camZ = camDistance * cos(camElevation * M_PI / 180.0f) * cos(camAngle * M_PI / 180.0f);
    
    gluLookAt(camX, camY, camZ, 0, 10, 0, 0, 1, 0);
    
    // Ground
    glDisable(GL_LIGHTING);
    glColor3f(0.15f, 0.15f, 0.15f);
    glBegin(GL_QUADS);
    glVertex3f(-300, 0, -300);
    glVertex3f(300, 0, -300);
    glVertex3f(300, 0, 300);
    glVertex3f(-300, 0, 300);
    glEnd();
    
    // Grid
    glColor3f(0.25f, 0.25f, 0.25f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int i = -300; i <= 300; i += 20) {
        glVertex3f(i, 0.1f, -300);
        glVertex3f(i, 0.1f, 300);
        glVertex3f(-300, 0.1f, i);
        glVertex3f(300, 0.1f, i);
    }
    glEnd();
    glEnable(GL_LIGHTING);
    
    // Facilities
    float resLevel = city.reservoirLevelSensorID >= 0 ? city.sensors[city.reservoirLevelSensorID].value : 75.0f;
    drawWaterReservoir(city.reservoirPos, resLevel);
    drawSewageTreatmentPlant(city.stpPos);
    
    // Cluster highlight
    if (highlightedCluster >= 0 && highlightedCluster < (int)city.clusters.size()) {
        const Cluster& c = city.clusters[highlightedCluster];
        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 0);
        glLineWidth(3.0f);
        float r = CLUSTER_SPACING * 0.6f;
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 32; i++) {
            float angle = i * M_PI * 2.0f / 32.0f;
            glVertex3f(c.centerPos.x + r * cos(angle), 5, c.centerPos.z + r * sin(angle));
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
    
    // Buildings
    if (showBuildings) {
        for (const auto& bldg : city.buildings) {
            if (highlightedCluster < 0 || bldg.clusterID == highlightedCluster) {
                drawBuilding(bldg);
            }
        }
    }
    
    // Pipes
    for (const auto& pipe : city.pipes) {
        bool isWater = pipe.type < SEWAGE_LATERAL;
        if ((isWater && showWaterNetwork) || (!isWater && showSewageNetwork)) {
            drawPipe(pipe);
        }
    }
    
    // Sensors
    if (showSensors) {
        for (const auto& sensor : city.sensors) {
            drawSensor(sensor);
        }
    }
    
    // Valves
    if (showValves) {
        for (const auto& valve : city.valves) {
            drawValve(valve);
        }
    }
    
    drawHUD();
    
    glutSwapBuffers();
}

void reshape(int w, int h) {
    windowWidth = w;
    windowHeight = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50.0, (double)w / h, 1.0, 1000.0);
    glMatrixMode(GL_MODELVIEW);
}

void idle() {
    city.updateSimulation(0.016f);
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 'b': case 'B':
            showBuildings = !showBuildings;
            break;
        case 'w': case 'W':
            showWaterNetwork = !showWaterNetwork;
            break;
        case 's': case 'S':
            showSewageNetwork = !showSewageNetwork;
            break;
        case 'v': case 'V':
            showValves = !showValves;
            break;
        case 'n': case 'N':
            showSensors = !showSensors;
            break;
        case 'l': case 'L':
            showLeaks = !showLeaks;
            break;
        case 'f': case 'F':
            showFlowParticles = !showFlowParticles;
            break;
        case 't': case 'T':
            showSensorLabels = !showSensorLabels;
            break;
        case 'h': case 'H':
            highlightedCluster++;
            if (highlightedCluster >= (int)city.clusters.size()) {
                highlightedCluster = -1;
            }
            break;
        case 'e': case 'E':
            city.loadEmbeddedPressureData("embedded_pressure.csv");
            break;
        case 'k': case 'K':
            city.loadEmbeddedLeakData("embedded_leaks.csv");
            break;
        case 'c': case 'C':
            city.loadValveControlData("valve_control.csv");
            break;
        case 'r': case 'R':
            camAngle = 45.0f;
            camElevation = 45.0f;
            camDistance = 200.0f;
            break;
        case '+': case '=':
            currentBuildingCount = std::min(1000, currentBuildingCount + 10);
            city.generateCity(currentBuildingCount);
            highlightedCluster = -1;
            break;
        case '-': case '_':
            currentBuildingCount = std::max(10, currentBuildingCount - 10);
            city.generateCity(currentBuildingCount);
            highlightedCluster = -1;
            break;
        case 'q': case 'Q': case 27:
            exit(0);
            break;
    }
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        mouseLeftDown = (state == GLUT_DOWN);
        lastMouseX = x;
        lastMouseY = y;
    } else if (button == 3) {
        camDistance = std::max(50.0f, camDistance - 10.0f);
    } else if (button == 4) {
        camDistance = std::min(500.0f, camDistance + 10.0f);
    }
}

void motion(int x, int y) {
    if (mouseLeftDown) {
        camAngle += (x - lastMouseX) * 0.5f;
        camElevation = std::max(5.0f, std::min(85.0f, camElevation - (y - lastMouseY) * 0.5f));
        lastMouseX = x;
        lastMouseY = y;
    }
}

void printInstructions() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   SCADA-GRADE WATER & SEWAGE DIGITAL TWIN                    ║\n";
    std::cout << "║   Embedded-System Driven Infrastructure Simulator            ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";
    std::cout << "STRICTLY EMBEDDED-DRIVEN:\n";
    std::cout << "  • NO internal simulation logic\n";
    std::cout << "  • ALL data from external files\n";
    std::cout << "  • Leaks appear ONLY when flagged by embedded system\n";
    std::cout << "  • Valve control from Python/external process\n";
    std::cout << "  • Behaves like real SCADA system\n\n";
    std::cout << "SENSOR & VALVE TOPOLOGY:\n";
    std::cout << "  • Per Building (POINT1_SUB): inlet valve, pressure/flow sensors\n";
    std::cout << "  • Per Cluster (POINT1_MAIN): cluster valve, pressure/flow sensors\n";
    std::cout << "  • Reservoir: main valve, pressure/level sensors\n";
    std::cout << "  • STP: inlet level, flow sensors\n\n";
    std::cout << "EMBEDDED DATA FILES:\n";
    std::cout << "  embedded_pressure.csv - sensorID,value\n";
    std::cout << "  embedded_leaks.csv - pipeID,hasLeak,leakRate,timestamp\n";
    std::cout << "  valve_control.csv - valveID,openPercentage\n\n";
    std::cout << "CONTROLS:\n";
    std::cout << "  B - Toggle buildings\n";
    std::cout << "  W - Toggle water network\n";
    std::cout << "  S - Toggle sewage network (BRIGHT ORANGE/BROWN)\n";
    std::cout << "  V - Toggle valves\n";
    std::cout << "  N - Toggle sensors\n";
    std::cout << "  L - Toggle leak visualization\n";
    std::cout << "  F - Toggle flow particles\n";
    std::cout << "  T - Toggle sensor/valve labels\n";
    std::cout << "  H - Cycle through cluster highlights\n";
    std::cout << "  E - Load embedded pressure data\n";
    std::cout << "  K - Load embedded leak data\n";
    std::cout << "  C - Load valve control data\n";
    std::cout << "  R - Reset camera view\n";
    std::cout << "  +/- - Increase/decrease building count\n";
    std::cout << "  Q - Quit\n";
    std::cout << "══════════════════════════════════════════════════════════════\n\n";
}


    int main(int argc, char** argv) {
    srand(time(nullptr));

    printInstructions();

    std::cout << "Generating city with " << currentBuildingCount << " buildings...\n";
    city.generateCity(currentBuildingCount);

    std::cout << "\nAttempting to load embedded data files...\n";
    city.loadEmbeddedPressureData("embedded_pressure.csv");
    city.loadEmbeddedLeakData("embedded_leaks.csv");
    city.loadValveControlData("valve_control.csv");

    // =========================
    // GLUT / OPENGL INITIALIZE
    // =========================
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("SCADA-Grade Water & Sewage Digital Twin");

    // =========================
    // OPENGL STATE
    // =========================
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // Lighting (industrial / neutral)
    GLfloat light_pos[] = { 200.0f, 300.0f, 200.0f, 1.0f };
    GLfloat light_amb[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat light_dif[] = { 0.9f, 0.9f, 0.9f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_dif);

    glClearColor(0.08f, 0.09f, 0.1f, 1.0f); // Dark SCADA background
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);

    // =========================
    // CALLBACKS
    // =========================
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    // =========================
    // START LOOP
    // =========================
    std::cout << "SCADA Digital Twin running...\n";
    std::cout << "Waiting for embedded data updates.\n\n";

    glutMainLoop();
    return 0;
}
