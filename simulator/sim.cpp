//g++ -o sim sim.cpp -lGL -lGLU -lglut -lm -O2 -std=c++11

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
#include <algorithm>
#include <cstdio>   
#include <sys/stat.h>


// ============================================================================
// ENGINEERING CONSTANTS
// ============================================================================

const int BUILDINGS_PER_CLUSTER = 10;
const float CITY_GRID_SPACING = 20.0f;
const float CLUSTER_SPACING = 80.0f;  // Increased spacing
int jsonExportCounter = 0;  // Add this global variable at the top
const float SYSTEM_JSON_UPDATE_INTERVAL = 0.1f; // Update every 0.1 seconds
float systemJsonUpdateTimer = 0.0f;

const int FLOOR_OPTIONS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15};
const int NUM_FLOOR_OPTIONS = 11;
const float FLOOR_HEIGHT = 3.0f;
const float BUILDING_FOOTPRINT = 10.0f;

// Pipe diameters in meters
const float TRUNK_DIAMETER = 0.8f;
const float SECONDARY_DIAMETER = 0.4f;
const float RING_DIAMETER = 0.25f;
const float SERVICE_DIAMETER = 0.05f;

// Default sensor values (ROS2 controllable)
const float DEFAULT_VALVE_STATE = 100.0f;  // ON (100%)
const float DEFAULT_PRESSURE = 15.0f;      // kPa
const float DEFAULT_WATER_LEVEL = 70.0f;   // %

// Water consumption parameters
const float WATER_CONSUMPTION_BASE = 0.0005f; // m³/s per building (base = 0.5 L/s)
const float DAILY_VARIATION = 0.3f; // ±30% daily variation
const float PARTICLE_SPEED = 2.0f; // m/s for water particles
const int PARTICLES_PER_PIPE = 20; // Number of particles per pipe
const float PARTICLE_SIZE = 0.15f; // Size of water particles

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
    float r, g, b, a;
    Color(float r = 1, float g = 1, float b = 1, float a = 1.0f) : r(r), g(g), b(b), a(a) {}
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

class CityNetwork; // Forward declaration

// ============================================================================
// WATER PARTICLE STRUCTURE
// ============================================================================

struct WaterParticle {
    Vec3 position;
    Vec3 direction;
    float speed;
    float life; // 0-1, 1 = full, 0 = dead
    Color color;
    int pipeID;
    
    WaterParticle(Vec3 pos, Vec3 dir, float spd, Color col, int pid)
        : position(pos), direction(dir.normalized()), speed(spd), life(1.0f), color(col), pipeID(pid) {}
    
    void update(float dt) {
        position = position + direction * (speed * dt);
        life -= dt * 0.2f; // Particles fade over time
        if (life < 0) life = 0;
    }
};

// ============================================================================
// SENSOR STRUCTURE (VALVE + PRESSURE + LEVEL combined)
// ============================================================================

enum SensorType {
    WATER_SENSOR,
    SEWAGE_SENSOR
};

struct Sensor {
    int id;                    // Unique sensor ID
    std::string name;
    SensorType type;           // WATER or SEWAGE
    Vec3 position;
    
    // ROS2 controllable values
    float valveState;          // 0-100% (DEFAULT: 100)
    float pressure;            // kPa (DEFAULT: 15)
    float waterLevel;          // % (DEFAULT: 70)
    
    bool active;
    int connectedPipeID;
    
    Sensor(int id, std::string name, SensorType type, Vec3 pos)
        : id(id), name(name), type(type), position(pos),
          valveState(DEFAULT_VALVE_STATE),
          pressure(DEFAULT_PRESSURE),
          waterLevel(DEFAULT_WATER_LEVEL),
          active(true), connectedPipeID(-1) {}
    
    Color getColor() const {
        if (!active) return Color(0.3f, 0.3f, 0.3f);
        
        // Different colors for WATER vs SEWAGE
        if (type == WATER_SENSOR) {
            // WATER: Blue/Cyan based on valve state
            if (valveState > 80.0f) return Color(0.2f, 0.7f, 1.0f);      // Bright blue = open
            else if (valveState > 20.0f) return Color(0.5f, 0.8f, 0.9f); // Light cyan = partial
            else return Color(0.3f, 0.5f, 0.7f);                          // Dark blue = closed
        } else {
            // SEWAGE: Orange/Brown based on valve state
            if (valveState > 80.0f) return Color(1.0f, 0.6f, 0.0f);      // Bright orange = open
            else if (valveState > 20.0f) return Color(0.9f, 0.7f, 0.3f); // Yellow-orange = partial
            else return Color(0.6f, 0.4f, 0.2f);                          // Brown = closed
        }
    }
    
    std::string getTypeString() const {
        return (type == WATER_SENSOR) ? "WATER" : "SEWAGE";
    }
    
    void updateFromROS2(float valve, float press, float level) {
        valveState = valve;
        pressure = press;
        waterLevel = level;
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
    int id;                    // Unique pipe ID
    Vec3 start, end;
    PipeType type;
    float diameter;
    float length;
    
    // Leak state (NO leak by default, controlled externally)
    bool hasLeak;
    float leakRate;            // L/s
    
    // Computed flow (from sensor states)
    float flowRate;
    
    // Water particles for visualization
    std::vector<WaterParticle> particles;
    
    Pipe(int id, Vec3 s, Vec3 e, PipeType t, float diam)
        : id(id), start(s), end(e), type(t), diameter(diam),
          hasLeak(false), leakRate(0.0f), flowRate(0.0f) {
        length = (end - start).length();
    }
    
    Color getColor() const {
        if (type >= SEWAGE_LATERAL) {
            // SEWAGE: SUPER BRIGHT ORANGE for maximum visibility
            return Color(1.0f, 0.6f, 0.0f);
        } else {
            // WATER: Cyan/blue
            float intensity = std::min(1.0f, flowRate / 50.0f);
            return Color(0.2f, 0.5f + intensity * 0.4f, 0.9f);
        }
    }
    
    void updateParticles(float dt, Color particleColor = Color(0.3f, 0.6f, 1.0f, 0.8f)) {
        // Update existing particles
        for (auto it = particles.begin(); it != particles.end();) {
            it->update(dt);
            if (it->life <= 0) {
                it = particles.erase(it);
            } else {
                ++it;
            }
        }
        
        // Add new particles if needed (only for water pipes with flow)
        if (type < SEWAGE_LATERAL && flowRate > 0.1f && particles.size() < PARTICLES_PER_PIPE) {
            // Add particles at start of pipe
            float t = static_cast<float>(rand()) / RAND_MAX;
            Vec3 spawnPos = start + (end - start) * t * 0.1f;
            Vec3 direction = (end - start).normalized();
            float speed = PARTICLE_SPEED * (0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX);
            
            // Vary particle color slightly
            Color particleCol = particleColor;
            particleCol.r *= (0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX);
            particleCol.g *= (0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX);
            
            particles.push_back(WaterParticle(spawnPos, direction, speed, particleCol, id));
        }
    }
    
    void drawParticles() const {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);
        
        for (const auto& particle : particles) {
            glColor4f(particle.color.r, particle.color.g, particle.color.b, particle.life * 0.7f);
            glPushMatrix();
            glTranslatef(particle.position.x, particle.position.y, particle.position.z);
            glutSolidSphere(PARTICLE_SIZE * (0.8f + 0.4f * particle.life), 8, 8);
            glPopMatrix();
        }
        
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
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
    
    int waterSensorID;         // Fresh water inlet sensor
    int sewageSensorID;        // Sewage outlet sensor
    int servicePipeID;
    int sewerPipeID;
    
    // Water consumption tracking
    float dailyWaterUsage;        // Random daily water usage factor
    float currentWaterFlow;       // Current water flow to building (m³/s)
    float totalWaterConsumed;     // Total water consumed (m³)
    Color waterColor;             // Color for water particles
    
    // Reference to city simulation time (will be set externally)
    static float* citySimulationTime;
    
    Building(int id, Vec3 pos, int floors, int cluster)
        : id(id), clusterID(cluster), position(pos), numFloors(floors),
          height(floors * FLOOR_HEIGHT),
          waterSensorID(-1), sewageSensorID(-1),
          servicePipeID(-1), sewerPipeID(-1),
          dailyWaterUsage(0), currentWaterFlow(0), totalWaterConsumed(0) {
        // Generate random daily water usage pattern
        dailyWaterUsage = 0.8f + (static_cast<float>(rand()) / RAND_MAX) * 0.4f; // 0.8-1.2
        
        // Assign random color for water particles
        waterColor = Color(
            (rand() % 100) / 200.0f + 0.5f,  // R: 0.5-1.0
            (rand() % 100) / 200.0f + 0.5f,  // G: 0.5-1.0
            (rand() % 100) / 200.0f + 0.8f,  // B: 0.8-1.0
            0.7f
        );
    }
    
    // Update building water consumption
    void updateWaterUsage(float dt) {
        if (!citySimulationTime) return;
        
        // Simulate daily variation (peaks in morning and evening)
        float timeOfDay = fmod(*citySimulationTime, 86400.0f) / 86400.0f; // 0-1 for day
        float dailyPattern = 0.7f + 0.6f * sinf(timeOfDay * 2 * M_PI - M_PI/2); // Peak at noon
        
        // Add some randomness
        float randomFactor = 0.9f + (static_cast<float>(rand()) / RAND_MAX) * 0.2f;
        
        // Calculate current flow
        currentWaterFlow = WATER_CONSUMPTION_BASE * dailyWaterUsage * dailyPattern * randomFactor;
        
        // Update total consumption
        totalWaterConsumed += currentWaterFlow * dt;
    }
    
    float getDailyConsumption() const {
        return currentWaterFlow * 86400.0f; // Convert to m³/day
    }
};

// Initialize static member
float* Building::citySimulationTime = nullptr;

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
    
    int waterSensorID;         // Cluster water inlet sensor
    int sewageSensorID;        // Cluster sewage outlet sensor
    int secondaryMainID;
    int sewageCollectorID;
    
    Cluster(int id, Vec3 center)
        : id(id), centerPos(center), waterSensorID(-1), sewageSensorID(-1),
          secondaryMainID(-1), sewageCollectorID(-1) {}
};

// ============================================================================
// CITY NETWORK
// ============================================================================

class CityNetwork {
public:
    std::vector<Building> buildings;
    std::vector<Cluster> clusters;
    std::vector<Pipe> pipes;
    std::vector<Sensor> sensors;
    std::vector<WaterParticle> reservoirParticles;
    
    Vec3 reservoirPos, stpPos;
    float cityExtent;          // For dynamic positioning
    
    int reservoirWaterSensorID;    // Main trunk water sensor
    int stpSewerSensorID;          // STP sewage inlet sensor
    
    float simulationTime;
    float reservoirVolume;      // Current water volume in reservoir (m³)
    float reservoirCapacity;    // Maximum reservoir capacity (m³)
    float reservoirLevel;       // Current water level (m)
    
    // Statistics
    float totalWaterSupplied;   // Total water supplied to city (m³)
    float totalWaterConsumed;   // Total water consumed by buildings (m³)
    float totalLeakWater;       // Total water lost to leaks (m³)
    
    CityNetwork() : cityExtent(100), reservoirWaterSensorID(-1), stpSewerSensorID(-1), 
                   simulationTime(0), reservoirVolume(0), reservoirCapacity(10000),
                   reservoirLevel(0), totalWaterSupplied(0), totalWaterConsumed(0),
                   totalLeakWater(0) {
        Building::citySimulationTime = &simulationTime;
    }
    
    void generateCity(int numBuildings);
    void updateDynamicPositions();
    void generateWaterNetwork();
    void generateSewageNetwork();
    void createSensors();
    void loadLeakData(const char* filename);
    void updateFromROS2(int sensorID, float valve, float pressure, float level);
    void updateSimulation(float dt);
    void generateInitSensorJSON(const char* filename);
    void exportSensorJSON(const char* filename);
    void exportLeakJSON(const char* filename);
    void exportSystemStateJSON(const char* filename);
    void importSensorJSON(const char* filename);
    
    // New methods for water tracking
    void updateBuildingWaterUsage(float dt);
    void updateReservoir(float dt);
    void generateReservoirParticles(float dt);
    void drawReservoirWithParticles() const;
    
private:
    void drawReservoirStructure(Vec3 pos, float waterLevel) const;
};

// ============================================================================
// CITY GENERATION
// ============================================================================

void CityNetwork::updateDynamicPositions() {
    // Calculate city extent
    float maxDist = 0;
    for (const auto& cluster : clusters) {
        float dist = sqrt(cluster.centerPos.x * cluster.centerPos.x + 
                         cluster.centerPos.z * cluster.centerPos.z);
        maxDist = std::max(maxDist, dist);
    }
    cityExtent = maxDist + CLUSTER_SPACING;
    
    // Position reservoir far from city
    reservoirPos = Vec3(-cityExtent - 80, 20, -cityExtent - 80);
    
    // Position STP far from city (opposite side)
    stpPos = Vec3(cityExtent + 80, 0, cityExtent + 80);
}

void CityNetwork::generateInitSensorJSON(const char* filename) {
    // ================= DELETE OLD FILE =================
    if (std::remove(filename) == 0) {
        std::cout << "Old init file deleted: " << filename << "\n";
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot create " << filename << "\n";
        return;
    }

    // ================= HEADER =================
    file << "{\n";
    file << "  \"generated_by\": \"water_ros2_scada_sim\",\n";
    file << "  \"schema\": \"init_sensor_v1\",\n";
    file << "  \"units\": {\n";
    file << "    \"pressure\": \"bar\",\n";
    file << "    \"level\": \"percent\"\n";
    file << "  },\n";
    file << "  \"timestamp\": " << time(nullptr) << ",\n";
    file << "  \"sensor_count\": " << sensors.size() << ",\n";
    file << "  \"sensors\": [\n";

    // ================= SENSOR LIST =================
    for (size_t i = 0; i < sensors.size(); ++i) {
        const Sensor& s = sensors[i];
        bool isFresh = (s.type == WATER_SENSOR);

        file << "    {\n";
        file << "      \"sensor_id\": " << s.id << ",\n";
        file << "      \"pipe_id\": " << s.connectedPipeID << ",\n";

        // pipe_type: 0 = fresh, 1 = sewage (matches ESP enum)
        file << "      \"pipe_type\": " << (isFresh ? 0 : 1) << ",\n";

        // Nominal baseline
        file << "      \"pressure_bar\": " << (isFresh ? 1.03f : 1.01f) << ",\n";
        file << "      \"level_pct\": " << (isFresh ? 90.0f : 70.0f) << ",\n";


        file << "      \"valve\": 1\n";

        file << "    }";
        if (i < sensors.size() - 1) file << ",";
        file << "\n";
    }

    // ================= FOOTER =================
    file << "  ]\n";
    file << "}\n";
    file.close();

    std::cout << "Init sensor snapshot regenerated: " << filename << "\n";
}

void CityNetwork::exportSensorJSON(const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot write " << filename << "\n";
        return;
    }

    file << "{\n";
    file << "  \"timestamp\": " << time(nullptr) << ",\n";
    file << "  \"sim_time\": " << std::fixed << std::setprecision(3) << simulationTime << ",\n";
    file << "  \"sensor_count\": " << sensors.size() << ",\n";
    file << "  \"sensors\": [\n";

    for (size_t i = 0; i < sensors.size(); ++i) {
        const Sensor& s = sensors[i];
        bool isFresh = (s.type == WATER_SENSOR);

        file << "    {\n";
        file << "      \"sensor_id\": " << s.id << ",\n";
        file << "      \"pipe_id\": " << s.connectedPipeID << ",\n";
        file << "      \"type\": " << (isFresh ? 1 : 0) << ",\n";
        file << "      \"pressure_bar\": " << std::setprecision(3) << (s.pressure / 100.0f) << ",\n";
        file << "      \"level_pct\": " << std::setprecision(1) << s.waterLevel << ",\n";
        file << "      \"valve\": " << (s.valveState > 50.0f ? 1 : 0) << "\n";
        file << "    }";
        if (i < sensors.size() - 1) file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";
    file.close();
}

void CityNetwork::importSensorJSON(const char* filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return; // No file = use defaults
    }

    std::string line, content;
    while (std::getline(file, line)) {
        content += line;
    }
    file.close();

    // Simple JSON parser (manual for no dependencies)
    size_t pos = 0;
    while ((pos = content.find("\"sensor_id\":", pos)) != std::string::npos) {
        pos += 12;
        int sid = std::stoi(content.substr(pos, content.find(",", pos) - pos));

        // Find pressure
        size_t p_pos = content.find("\"pressure_bar\":", pos);
        if (p_pos != std::string::npos) {
            p_pos += 15;
            float pressure_bar = std::stof(content.substr(p_pos, content.find(",", p_pos) - p_pos));
            
            // Find level
            size_t l_pos = content.find("\"level_pct\":", p_pos);
            float level = 70.0f;
            if (l_pos != std::string::npos) {
                l_pos += 12;
                level = std::stof(content.substr(l_pos, content.find(",", l_pos) - l_pos));
            }

            // Find valve
            size_t v_pos = content.find("\"valve\":", l_pos);
            float valve = 100.0f;
            if (v_pos != std::string::npos) {
                v_pos += 8;
                int valve_int = std::stoi(content.substr(v_pos, content.find("}", v_pos) - v_pos));
                valve = valve_int ? 100.0f : 0.0f;
            }

            // Update sensor
            if (sid >= 0 && sid < (int)sensors.size()) {
                sensors[sid].pressure = pressure_bar * 100.0f; // bar to kPa
                sensors[sid].waterLevel = level;
                sensors[sid].valveState = valve;
            }
        }
    }
}

// ============================================================================
// 3. LEAK.JSON - Leak detection data (WRITE by simulator)
// ============================================================================
void CityNetwork::exportLeakJSON(const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot write " << filename << "\n";
        return;
    }

    file << "{\n";
    file << "  \"timestamp\": " << time(nullptr) << ",\n";
    file << "  \"sim_time\": " << std::fixed << std::setprecision(3) << simulationTime << ",\n";
    file << "  \"total_leaks\": " << 0 << ",\n"; // Will count below
    file << "  \"leaks\": [\n";

    int leakCount = 0;
    bool first = true;
    
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            if (!first) file << ",\n";
            first = false;
            leakCount++;

            // Find controlling sensor
            int sensor_id = -1;
            int valve_state = 1;
            for (const auto& s : sensors) {
                if (s.connectedPipeID == pipe.id) {
                    sensor_id = s.id;
                    valve_state = (s.valveState > 50.0f) ? 1 : 0;
                    break;
                }
            }

            file << "    {\n";
            file << "      \"pipe_id\": " << pipe.id << ",\n";
            file << "      \"sensor_id\": " << sensor_id << ",\n";
            file << "      \"leak\": 1,\n";
            file << "      \"leak_rate_lps\": " << std::setprecision(2) << pipe.leakRate << ",\n";
            file << "      \"valve\": " << valve_state << "\n";
            file << "    }";
        }
    }

    file << "\n  ]\n";
    file << "}\n";
    file.close();

    // Update total_leaks count (rewrite file)
    if (leakCount > 0) {
        std::ifstream in(filename);
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();

        size_t pos = content.find("\"total_leaks\": 0");
        if (pos != std::string::npos) {
            content.replace(pos + 16, 1, std::to_string(leakCount));
            std::ofstream out(filename);
            out << content;
            out.close();
        }
    }
}

// ============================================================================
// 4. SYSTEM_STATE.JSON - Full physics state for ROS2/RL (WRITE)
// ============================================================================
void CityNetwork::exportSystemStateJSON(const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot write " << filename << "\n";
        return;
    }

    file << "{\n";
    file << "  \"header\": {\n";
    file << "    \"timestamp\": " << time(nullptr) << ",\n";
    file << "    \"sim_time_sec\": " << std::fixed << std::setprecision(3) << simulationTime << ",\n";
    file << "    \"sim_dt_sec\": 0.016,\n";
    file << "    \"step_count\": " << (int)(simulationTime / 0.016f) << "\n";
    file << "  },\n";

    // ================= GLOBAL STATE =================
    file << "  \"global_state\": {\n";

    file << "    \"reservoir_level_m\": "
         << std::setprecision(2)
         << reservoirLevel << ",\n";

    file << "    \"reservoir_volume_m3\": "
         << std::setprecision(2)
         << reservoirVolume << ",\n";

    // -------- Flow & Leak Aggregation --------
    float total_fresh_flow = 0.0f;
    float total_sewage_flow = 0.0f;
    float total_leak_flow  = 0.0f;
    int   numLeaks         = 0;
    std::vector<int> leakingPipeIDs;

    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) {
            total_fresh_flow += pipe.flowRate;
        } else {
            total_sewage_flow += pipe.flowRate;
        }

        if (pipe.hasLeak) {
            total_leak_flow += pipe.leakRate;
            numLeaks++;
            leakingPipeIDs.push_back(pipe.id);
        }
    }

    file << "    \"total_fresh_demand_m3s\": "
         << std::setprecision(4) << (total_fresh_flow / 1000.0f) << ",\n";

    file << "    \"total_sewage_flow_m3s\": "
         << (total_sewage_flow / 1000.0f) << ",\n";

    file << "    \"total_leak_flow_m3s\": "
         << (total_leak_flow / 1000.0f) << ",\n";

    file << "    \"active_leak_count\": " << numLeaks << ",\n";

    // -------- WHICH PIPE IS LEAKING --------
    file << "    \"leaking_pipe_ids\": [";
    for (size_t i = 0; i < leakingPipeIDs.size(); ++i) {
        file << leakingPipeIDs[i];
        if (i < leakingPipeIDs.size() - 1) file << ", ";
    }
    file << "],\n";

    // ================= PRESSURE STATS =================
    float min_p = 9999.0f, max_p = 0.0f, sum_p = 0.0f;
    int p_count = 0;

    for (const auto& s : sensors) {
        if (s.type == WATER_SENSOR) {
            min_p = std::min(min_p, s.pressure);
            max_p = std::max(max_p, s.pressure);
            sum_p += s.pressure;
            p_count++;
        }
    }

    file << "    \"avg_fresh_pressure_kpa\": "
         << std::setprecision(1) << (p_count ? sum_p / p_count : 0.0f) << ",\n";
    file << "    \"min_fresh_pressure_kpa\": " << min_p << ",\n";
    file << "    \"max_fresh_pressure_kpa\": " << max_p << ",\n";

    // ================= SEWAGE STATS =================
    float max_sewage_level = 0.0f, sum_sewage = 0.0f;
    int s_count = 0;

    for (const auto& s : sensors) {
        if (s.type == SEWAGE_SENSOR) {
            max_sewage_level = std::max(max_sewage_level, s.waterLevel);
            sum_sewage += s.waterLevel;
            s_count++;
        }
    }

    file << "    \"avg_sewage_level_pct\": "
         << (s_count ? sum_sewage / s_count : 0.0f) << ",\n";
    file << "    \"max_sewage_level_pct\": " << max_sewage_level << ",\n";

    // ================= PERFORMANCE METRICS =================
    float demand_satisfaction =
        (total_fresh_flow > 0.0f)
            ? (total_fresh_flow - total_leak_flow) / total_fresh_flow
            : 1.0f;

    file << "    \"demand_satisfaction\": "
         << std::setprecision(3) << demand_satisfaction << ",\n";

    file << "    \"hydraulic_efficiency\": "
         << (1.0f - total_leak_flow / (total_fresh_flow + 0.1f)) << ",\n";

    file << "    \"non_revenue_water_ratio\": "
         << (total_leak_flow / (total_fresh_flow + 0.1f)) << "\n";

    file << "  },\n";
    
    // ================= BUILDING WATER CONSUMPTION =================
    file << "  \"building_water_usage\": {\n";
    file << "    \"total_buildings\": " << buildings.size() << ",\n";
    file << "    \"total_consumption_m3\": " << std::setprecision(2) << totalWaterConsumed << ",\n";
    file << "    \"current_demand_m3s\": " << std::setprecision(4) << (total_fresh_flow / 1000.0f) << ",\n";
    file << "    \"buildings\": [\n";
    
    // Show water usage for all buildings (not just sample)
    for (size_t i = 0; i < buildings.size(); ++i) {
        const auto& b = buildings[i];
        file << "      {\n";
        file << "        \"id\": " << b.id << ",\n";
        file << "        \"cluster\": " << b.clusterID << ",\n";
        file << "        \"floors\": " << b.numFloors << ",\n";
        file << "        \"current_flow_m3s\": " << std::setprecision(6) << b.currentWaterFlow << ",\n";
        file << "        \"daily_consumption_m3\": " << std::setprecision(2) << b.getDailyConsumption() << ",\n";
        file << "        \"total_consumed_m3\": " << std::setprecision(2) << b.totalWaterConsumed << "\n";
        file << "      }";
        if (i < buildings.size() - 1) file << ",";
        file << "\n";
    }
    
    file << "    ]\n";
    file << "  },\n";
    
    // ================= PIPE STATE FOR RL =================
    file << "  \"pipe_state\": {\n";
    file << "    \"total_pipes\": " << pipes.size() << ",\n";
    file << "    \"water_pipes\": " << [&]() {
        int count = 0;
        for (const auto& pipe : pipes) {
            if (pipe.type < SEWAGE_LATERAL) count++;
        }
        return count;
    }() << ",\n";
    file << "    \"sewage_pipes\": " << [&]() {
        int count = 0;
        for (const auto& pipe : pipes) {
            if (pipe.type >= SEWAGE_LATERAL) count++;
        }
        return count;
    }() << ",\n";
    file << "    \"pipes\": [\n";
    
    // Include all pipes for RL agent
    for (size_t i = 0; i < pipes.size(); ++i) {
        const auto& p = pipes[i];
        file << "      {\n";
        file << "        \"id\": " << p.id << ",\n";
        file << "        \"type\": " << (p.type < SEWAGE_LATERAL ? "\"water\"" : "\"sewage\"") << ",\n";
        file << "        \"diameter_m\": " << std::setprecision(3) << p.diameter << ",\n";
        file << "        \"length_m\": " << std::setprecision(1) << p.length << ",\n";
        file << "        \"flow_m3s\": " << std::setprecision(4) << (p.flowRate / 1000.0f) << ",\n";
        file << "        \"has_leak\": " << (p.hasLeak ? "true" : "false") << ",\n";
        file << "        \"leak_rate_m3s\": " << std::setprecision(4) << (p.leakRate / 1000.0f) << ",\n";
        
        // Find connected sensor
        int sensor_id = -1;
        float valve_state = 0.0f;
        for (const auto& s : sensors) {
            if (s.connectedPipeID == p.id) {
                sensor_id = s.id;
                valve_state = s.valveState / 100.0f;
                break;
            }
        }
        
        file << "        \"sensor_id\": " << sensor_id << ",\n";
        file << "        \"valve_state\": " << std::setprecision(2) << valve_state << "\n";
        file << "      }";
        if (i < pipes.size() - 1) file << ",";
        file << "\n";
    }
    
    file << "    ]\n";
    file << "  }\n";
    
    file << "}\n";
    file.close();
}



void CityNetwork::generateCity(int numBuildings) {
    buildings.clear();
    clusters.clear();
    pipes.clear();
    sensors.clear();
    reservoirParticles.clear();
    
    std::cout << "\n=== GENERATING ROS2 SCADA CITY ===\n";
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
    
    updateDynamicPositions();
    
    // Initialize reservoir
    reservoirVolume = reservoirCapacity * 0.7f; // Start at 70% capacity
    reservoirLevel = (reservoirVolume / reservoirCapacity) * 30.0f; // 30m max height
    
    std::cout << "Created " << buildings.size() << " buildings in " << clusters.size() << " clusters\n";
    std::cout << "City extent: " << cityExtent << "m\n";
    std::cout << "Reservoir at: (" << reservoirPos.x << ", " << reservoirPos.z << ")\n";
    std::cout << "STP at: (" << stpPos.x << ", " << stpPos.z << ")\n";
    std::cout << "Reservoir capacity: " << reservoirCapacity << " m³\n";
    std::cout << "Initial reservoir volume: " << reservoirVolume << " m³\n";
    
    generateWaterNetwork();
    generateSewageNetwork();
    createSensors();
    
    std::cout << "Total pipes: " << pipes.size() << "\n";
    std::cout << "Total sensors: " << sensors.size() << "\n";
    std::cout << "=== CITY GENERATION COMPLETE ===\n\n";
}

void CityNetwork::generateWaterNetwork() {
    int pipeID = 0;
    
    // Main trunk from reservoir
    Vec3 trunkStart = reservoirPos + Vec3(0, -10, 0);
    Vec3 trunkEnd(0, 0, 0);
    pipes.push_back(Pipe(pipeID++, trunkStart, trunkEnd, TRUNK_MAIN, TRUNK_DIAMETER));
    
    // Secondary mains to clusters
    for (auto& cluster : clusters) {
        Vec3 secondaryEnd = cluster.centerPos + Vec3(0, 1, 0);
        cluster.secondaryMainID = pipeID;
        pipes.push_back(Pipe(pipeID++, trunkEnd, secondaryEnd, SECONDARY_MAIN, SECONDARY_DIAMETER));
    }
    
    // Ring mains within clusters
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
    
    // Service pipes to buildings
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
    int sewagePipeCount = 0;
    
    // Building laterals and cluster collectors
    for (auto& cluster : clusters) {
        Vec3 collectorPoint = cluster.centerPos + Vec3(0, -2.5f, CLUSTER_SPACING * 0.7f);
        
        for (int bid : cluster.buildingIDs) {
            Building& bldg = buildings[bid];
            Vec3 lateralStart = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.75f, 1.0f, BUILDING_FOOTPRINT * 0.75f);
            Vec3 lateralEnd = collectorPoint;
            lateralEnd.y = -2.0f; // Underground
            
            bldg.sewerPipeID = pipeID;
            cluster.sewerPipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(
    pipeID++,
    lateralStart,
    lateralEnd,
    SEWAGE_LATERAL,
   SERVICE_DIAMETER * 1.5f
));

        }
        
        Vec3 collectorEnd = Vec3(collectorPoint.x, -3.5f, stpPos.z - 30.0f);
        cluster.sewageCollectorID = pipeID;
        pipes.push_back(Pipe(
    pipeID++,
    collectorPoint,
    collectorEnd,
    SEWAGE_COLLECTOR,
    SECONDARY_DIAMETER * 1.2f
));

    }
    
    // Main interceptor to STP - DEEP UNDERGROUND
    Vec3 interceptorStart(0, -4.0f, stpPos.z - 30.0f);
    Vec3 interceptorEnd = stpPos + Vec3(0, 0.5f, -15.0f);
    pipes.push_back(Pipe(
    pipeID++,
    interceptorStart,
    interceptorEnd,
    SEWAGE_INTERCEPTOR,
    TRUNK_DIAMETER
));

    
    std::cout << "  Sewage pipes: pipe_s0 to pipe_s" << (sewagePipeCount-1) << "\n";
}

void CityNetwork::createSensors() {
    int sensorID = 0;
    int waterSensorCount = 0;
    int sewageSensorCount = 0;
    
    // ===== WATER SIDE SENSORS =====
    
    // RESERVOIR WATER SENSOR (main trunk)
    reservoirWaterSensorID = sensorID;
    Vec3 resSensorPos = reservoirPos + Vec3(0, -5, 0);
    sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, resSensorPos));
    sensors.back().connectedPipeID = 0; // Trunk main
    std::cout << "  Created " << sensors.back().name << " (Reservoir Water) ID: " << reservoirWaterSensorID << "\n";
    
    // CLUSTER WATER SENSORS
    for (auto& cluster : clusters) {
        cluster.waterSensorID = sensorID;
        Vec3 clusterWaterPos = cluster.centerPos + Vec3(-3, 2.5, 0);
        sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, clusterWaterPos));
        sensors.back().connectedPipeID = cluster.secondaryMainID;
    }
    std::cout << "  Created " << clusters.size() << " Cluster Water Sensors (sensor_f1 to sensor_f" << (waterSensorCount-1) << ")\n";
    
    // BUILDING WATER SENSORS
    for (auto& bldg : buildings) {
        bldg.waterSensorID = sensorID;
        Vec3 bldgWaterPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.15f, 1.8f, BUILDING_FOOTPRINT * 0.25f);
        sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, bldgWaterPos));
        sensors.back().connectedPipeID = bldg.servicePipeID;
    }
    std::cout << "  Created " << buildings.size() << " Building Water Sensors\n";
    std::cout << "  Total Water Sensors: sensor_f0 to sensor_f" << (waterSensorCount-1) << "\n";
    
    // ===== SEWAGE SIDE SENSORS =====
    
    // BUILDING SEWAGE SENSORS
    for (auto& bldg : buildings) {
        bldg.sewageSensorID = sensorID;
        Vec3 bldgSewerPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.85f, 1.8f, BUILDING_FOOTPRINT * 0.75f);
        sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, bldgSewerPos));
        sensors.back().connectedPipeID = bldg.sewerPipeID;
    }
    std::cout << "  Created " << buildings.size() << " Building Sewage Sensors\n";
    
    // CLUSTER SEWAGE SENSORS
    for (auto& cluster : clusters) {
        cluster.sewageSensorID = sensorID;
        Vec3 clusterSewerPos = cluster.centerPos + Vec3(3, -0.5, CLUSTER_SPACING * 0.6f);
        sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, clusterSewerPos));
        sensors.back().connectedPipeID = cluster.sewageCollectorID;
    }
    std::cout << "  Created " << clusters.size() << " Cluster Sewage Sensors\n";
    
    // STP SEWAGE SENSOR
    stpSewerSensorID = sensorID;
    Vec3 stpSensorPos = stpPos + Vec3(0, 3, -15);
    sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, stpSensorPos));
    std::cout << "  Created " << sensors.back().name << " (STP Sewage) ID: " << stpSewerSensorID << "\n";
    std::cout << "  Total Sewage Sensors: sensor_s0 to sensor_s" << (sewageSensorCount-1) << "\n";
    
    std::cout << "\n=== SENSOR SUMMARY ===\n";
    std::cout << "Total Water Sensors: " << waterSensorCount << " (sensor_f0 to sensor_f" << (waterSensorCount-1) << ")\n";
    std::cout << "Total Sewage Sensors: " << sewageSensorCount << " (sensor_s0 to sensor_s" << (sewageSensorCount-1) << ")\n";
    std::cout << "Grand Total Sensors: " << sensors.size() << "\n";
}

void CityNetwork::loadLeakData(const char* filename) {
    std::cout << "Loading leak data from: " << filename << "\n";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "  No leak file found - NO LEAKS (default)\n";
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
            pipes[pipeID].hasLeak = hasLeak;
            pipes[pipeID].leakRate = leakRate;
            if (hasLeak) {
                std::cout << "  LEAK: Pipe " << pipeID << " = " << leakRate << " L/s\n";
            }
        }
    }
    file.close();
}

void CityNetwork::updateFromROS2(int sensorID, float valve, float pressure, float level) {
    if (sensorID >= 0 && sensorID < (int)sensors.size()) {
        sensors[sensorID].updateFromROS2(valve, pressure, level);
    }
}

void CityNetwork::updateBuildingWaterUsage(float dt) {
    totalWaterConsumed = 0;
    
    for (auto& building : buildings) {
        building.updateWaterUsage(dt);
        totalWaterConsumed += building.currentWaterFlow * dt;
    }
}

void CityNetwork::updateReservoir(float dt) {
    // Calculate total water demand from buildings
    float totalDemand = 0;
    for (const auto& building : buildings) {
        totalDemand += building.currentWaterFlow;
    }
    
    // Calculate total leaks
    float totalLeaks = 0;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeaks += pipe.leakRate / 1000.0f; // Convert L/s to m³/s
        }
    }
    
    // Update reservoir volume
    float outflow = (totalDemand + totalLeaks) * dt;
    reservoirVolume -= outflow;
    
    // Keep reservoir within bounds
    if (reservoirVolume < 0) reservoirVolume = 0;
    if (reservoirVolume > reservoirCapacity) reservoirVolume = reservoirCapacity;
    
    // Update reservoir level (0-30 meters)
    reservoirLevel = (reservoirVolume / reservoirCapacity) * 30.0f;
    
    // Update statistics
    totalWaterSupplied += outflow;
    totalLeakWater += totalLeaks * dt;
    
    // Update reservoir sensor
    if (reservoirWaterSensorID >= 0 && reservoirWaterSensorID < (int)sensors.size()) {
        sensors[reservoirWaterSensorID].waterLevel = (reservoirVolume / reservoirCapacity) * 100.0f;
    }
}

void CityNetwork::generateReservoirParticles(float dt) {
    // Update existing particles
    for (auto it = reservoirParticles.begin(); it != reservoirParticles.end();) {
        it->update(dt);
        if (it->life <= 0) {
            it = reservoirParticles.erase(it);
        } else {
            ++it;
        }
    }
    
    // Add new particles if reservoir has water
    if (reservoirVolume > 0 && reservoirParticles.size() < 100) {
        // Generate particles near the outlet
        float radius = 10.0f;
        float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
        float dist = static_cast<float>(rand()) / RAND_MAX * radius;
        
        Vec3 spawnPos = reservoirPos + Vec3(
            cos(angle) * dist,
            -5 + static_cast<float>(rand()) / RAND_MAX * 5, // Random height in reservoir
            sin(angle) * dist
        );
        
        // Particle flows toward outlet
        Vec3 direction = (reservoirPos + Vec3(0, -10, 0) - spawnPos).normalized();
        direction = direction + Vec3(
            (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.2f,
            (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.1f,
            (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.2f
        ).normalized();
        
        float speed = 0.5f + static_cast<float>(rand()) / RAND_MAX * 1.0f;
        
        Color particleColor(0.2f, 0.5f, 0.9f, 0.8f);
        
        reservoirParticles.push_back(WaterParticle(spawnPos, direction, speed, particleColor, -1));
    }
}

void CityNetwork::updateSimulation(float dt) {
    simulationTime += dt;
    
    // Update building water usage
    updateBuildingWaterUsage(dt);
    
    // Update reservoir
    updateReservoir(dt);
    
    // Generate reservoir particles
    generateReservoirParticles(dt);
    
    // Compute flow rates based on sensor states and building demand
    float totalBuildingDemand = 0;
    for (const auto& building : buildings) {
        totalBuildingDemand += building.currentWaterFlow;
    }
    
    // Distribute flow through pipes
    for (auto& pipe : pipes) {
        // Find controlling sensor
        float valveFactor = 1.0f;
        for (const auto& sensor : sensors) {
            if (sensor.connectedPipeID == pipe.id) {
                valveFactor = sensor.valveState / 100.0f;
                break;
            }
        }
        
        // Simplified flow calculation based on pipe type and demand
        if (pipe.type < SEWAGE_LATERAL) {
            // Water pipes: flow based on downstream demand
            if (pipe.type == TRUNK_MAIN) {
                pipe.flowRate = totalBuildingDemand * 1000.0f * valveFactor; // Convert to L/s
            } else if (pipe.type == SERVICE_PIPE) {
                // Service pipes get flow from their building
                for (const auto& building : buildings) {
                    if (building.servicePipeID == pipe.id) {
                        pipe.flowRate = building.currentWaterFlow * 1000.0f * valveFactor;
                        break;
                    }
                }
            } else {
                // Other water pipes get proportional flow
                pipe.flowRate = 10.0f * valveFactor;
            }
            
            // Update particles for this pipe with building-specific color
            Color particleColor(0.3f, 0.6f, 1.0f, 0.8f);
            for (const auto& building : buildings) {
                if (building.servicePipeID == pipe.id) {
                    particleColor = building.waterColor;
                    break;
                }
            }
            pipe.updateParticles(dt, particleColor);
            
        } else {
            // Sewage pipes: flow based on building outflow
            pipe.flowRate = 5.0f; // Base sewage flow
        }
    }
}

void CityNetwork::drawReservoirStructure(Vec3 pos, float waterLevel) const {
    float tankRadius = 25.0f;
    float tankHeight = 30.0f;
    float wallThickness = 1.0f;
    
    // Draw reservoir structure (concrete)
    glColor3f(0.7f, 0.7f, 0.7f);
    
    // Draw outer cylinder
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight/2, pos.z);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, tankRadius, tankRadius, tankHeight, 64, 8);
    gluDeleteQuadric(quad);
    glPopMatrix();
    
    // Draw top cap
    glPushMatrix();
    glTranslatef(pos.x, pos.y + tankHeight/2, pos.z);
    glRotatef(-90, 1, 0, 0);
    GLUquadric* disk = gluNewQuadric();
    gluDisk(disk, 0, tankRadius, 64, 8);
    gluDeleteQuadric(disk);
    glPopMatrix();
    
    // Draw water surface
    float waterHeight = waterLevel;
    if (waterHeight > 0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.1f, 0.3f, 0.8f, 0.7f);
        
        glPushMatrix();
        glTranslatef(pos.x, pos.y - tankHeight/2 + waterHeight, pos.z);
        glRotatef(-90, 1, 0, 0);
        GLUquadric* waterDisk = gluNewQuadric();
        gluDisk(waterDisk, 0, tankRadius, 64, 8);
        gluDeleteQuadric(waterDisk);
        glPopMatrix();
        
        // Draw water volume
        glColor4f(0.15f, 0.4f, 0.9f, 0.5f);
        glPushMatrix();
        glTranslatef(pos.x, pos.y - tankHeight/2 + waterHeight/2, pos.z);
        gluCylinder(quad, tankRadius - wallThickness, tankRadius - wallThickness, waterHeight, 64, 8);
        glPopMatrix();
        
        glDisable(GL_BLEND);
    }
    
    // Draw reservoir base/platform
    glColor3f(0.5f, 0.5f, 0.5f);
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight - 5, pos.z);
    glScalef(40.0f, 10.0f, 40.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Draw outlet structure
    glColor3f(0.4f, 0.4f, 0.4f);
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight - 2, pos.z);
    glRotatef(-90, 0, 1, 0);
    GLUquadric* outlet = gluNewQuadric();
    gluCylinder(outlet, 1.5f, 1.5f, 15.0f, 16, 4);
    gluDeleteQuadric(outlet);
    glPopMatrix();
}

void CityNetwork::drawReservoirWithParticles() const {
    // Draw reservoir structure
    drawReservoirStructure(reservoirPos, reservoirLevel);
    
    // Draw reservoir particles
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    
    for (const auto& particle : reservoirParticles) {
        glColor4f(particle.color.r, particle.color.g, particle.color.b, particle.life * 0.6f);
        glPushMatrix();
        glTranslatef(particle.position.x, particle.position.y, particle.position.z);
        glutSolidSphere(PARTICLE_SIZE * (0.7f + 0.3f * particle.life), 6, 6);
        glPopMatrix();
    }
    
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

// ============================================================================
// RENDERING
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
bool showLeaks = true;
bool showSensorLabels = false;
bool showGround = true;  // Toggle ground visibility
bool transparentBuildings = false;  // Toggle building transparency
bool showWaterParticles = true;     // Toggle water particles in pipes
bool showReservoirParticles = true; // Toggle reservoir particles
int highlightedCluster = -1;
int currentBuildingCount = 200;

void drawCylinder(Vec3 start, Vec3 end, float radius, Color color) {
    glColor3f(color.r, color.g, color.b);
    
    // Enhanced material properties
    GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat mat_shininess[] = { 50.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    
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
    gluQuadricNormals(quad, GLU_SMOOTH);
    gluQuadricTexture(quad, GL_TRUE);
    gluCylinder(quad, radius, radius, length, 64, 8);
    gluDeleteQuadric(quad);
    
    glPopMatrix();
}

void drawSphere(Vec3 pos, float radius, Color color) {
    glColor3f(color.r, color.g, color.b);
    
    // Enhanced material
    GLfloat mat_specular[] = { 0.6f, 0.6f, 0.6f, 1.0f };
    GLfloat mat_shininess[] = { 60.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glutSolidSphere(radius, 48, 48);
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
    
    // Enhanced pulsing effect
    float pulse = 0.75f + 0.35f * sinf(city.simulationTime * 3.0f);
    
    // Outer glow for active sensors
    if (sensor.active) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f(col.r, col.g, col.b, 0.3f * pulse);
        
        float glowSize = 2.5f;
        glPushMatrix();
        glTranslatef(sensor.position.x, sensor.position.y, sensor.position.z);
        glutSolidSphere(glowSize, 16, 16);
        glPopMatrix();
        glDisable(GL_BLEND);
    }
    
    // Enhanced material for sensor body
    GLfloat mat_specular[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat mat_shininess[] = { 80.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    
    // Sensor body - larger and more prominent
    float sensorSize = 2.2f;
    glColor3f(col.r * pulse, col.g * pulse, col.b * pulse);
    glPushMatrix();
    glTranslatef(sensor.position.x, sensor.position.y, sensor.position.z);
    glutSolidCube(sensorSize);
    glPopMatrix();
    
    // Valve wheel on top - more prominent
    glDisable(GL_LIGHTING);
    glColor3f(col.r * 0.7f, col.g * 0.7f, col.b * 0.7f);
    glPushMatrix();
    glTranslatef(sensor.position.x, sensor.position.y + 1.8f, sensor.position.z);
    glRotatef(90, 1, 0, 0);
    glutSolidTorus(0.4, 0.9, 10, 20);
    glPopMatrix();
    glEnable(GL_LIGHTING);
    
    // ID Label with enhanced visibility
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        
        // Background color based on type - brighter
        if (sensor.type == WATER_SENSOR) {
            glColor3f(0.3f, 0.85f, 1.0f);  // Bright cyan for water
        } else {
            glColor3f(1.0f, 0.7f, 0.2f);  // Bright orange for sewage
        }
        
        glRasterPos3f(sensor.position.x, sensor.position.y + 3.5f, sensor.position.z);
        std::stringstream ss;
        ss << sensor.name << " [" << sensor.getTypeString() << "]";
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
        }
        
        // Detailed info below - brighter
        glRasterPos3f(sensor.position.x, sensor.position.y + 2.5f, sensor.position.z);
        ss.str("");
        ss << "V:" << (int)sensor.valveState << "% P:" << (int)sensor.pressure << "kPa L:" << (int)sensor.waterLevel << "%";
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        
        glEnable(GL_LIGHTING);
    }
}

void drawPipe(const Pipe& pipe) {
    Color color = pipe.getColor();
    float radius = pipe.diameter / 2.0f;
    
    // VISUAL DISTINCTION - Different rendering styles
    if (pipe.type >= SEWAGE_LATERAL) {
        // SEWAGE: Brown color with enhanced contrast
        Color sewerColor(0.65f, 0.38f, 0.12f);
        drawCylinder(pipe.start, pipe.end, radius, sewerColor);
        drawSphere(pipe.start, radius * 1.4f, sewerColor);
        drawSphere(pipe.end, radius * 1.4f, sewerColor);
        
        // Add BRIGHT ORANGE stripes for maximum visual distinction
        Vec3 dir = (pipe.end - pipe.start).normalized();
        float len = pipe.length;
        int numStripes = std::max(3, (int)(len / 3.0f));
        
        glDisable(GL_LIGHTING);
        for (int i = 0; i < numStripes; i++) {
            float t = (float)i / numStripes;
            Vec3 stripePos = pipe.start + (pipe.end - pipe.start) * t;
            glColor3f(1.0f, 0.6f, 0.1f); // Bright orange stripe
            glPushMatrix();
            glTranslatef(stripePos.x, stripePos.y, stripePos.z);
            glutSolidSphere(radius * 1.3f, 12, 12);
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
        
    } else {
        // WATER: Enhanced cyan/blue with shimmer
        Color waterColor(0.25f, 0.65f, 0.95f);
        drawCylinder(pipe.start, pipe.end, radius, waterColor);
        
        // Add subtle glow effect
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f(0.4f, 0.8f, 1.0f, 0.3f);
        drawCylinder(pipe.start, pipe.end, radius * 1.1f, Color(0.4f, 0.8f, 1.0f));
        glDisable(GL_BLEND);
        
        drawSphere(pipe.start, radius * 1.3f, waterColor);
        drawSphere(pipe.end, radius * 1.3f, waterColor);
        
        // Draw water particles in pipe if enabled
        if (showWaterParticles && pipe.type < SEWAGE_LATERAL && pipe.flowRate > 0.1f) {
            pipe.drawParticles();
        }
    }
    
    // LEAK visualization with enhanced effects
    if (pipe.hasLeak && showLeaks) {
        Vec3 mid = (pipe.start + pipe.end) * 0.5f;
        
        float pulse = 0.8f + 0.4f * sinf(city.simulationTime * 5.0f);
        
        // Outer glow
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f(1.0f, 0.0f, 0.0f, 0.4f);
        drawSphere(mid, 3.5f * pulse, Color(1, 0, 0));
        glDisable(GL_BLEND);
        
        // Inner bright core
        drawSphere(mid, 2.0f * pulse, Color(1, 0, 0));
        
        glDisable(GL_LIGHTING);
        // Warning cone with enhanced color
        glColor3f(1, 0.3f, 0);
        glPushMatrix();
        glTranslatef(mid.x, mid.y + 5, mid.z);
        glRotatef(-90, 1, 0, 0);
        glutSolidCone(2.5, 5.0, 12, 2);
        glPopMatrix();
        
        // Enhanced particle spray
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        for (int i = 0; i < 60; i++) {
            float angle = i * M_PI * 2.0f / 60.0f + city.simulationTime * 2.0f;
            float dist = 3.5f + 2.0f * sinf(city.simulationTime * 3.0f + i);
            Vec3 p = mid + Vec3(cos(angle) * dist, sinf(angle * 3) * dist + 2, sin(angle) * dist);
            
            // Alternating blue/white particles
            if (i % 2 == 0) {
                glColor3f(0.5f, 0.8f, 1.0f);
            } else {
                glColor3f(1.0f, 1.0f, 1.0f);
            }
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
        
        // Enhanced label with background
        glColor3f(1, 0, 0);
        glRasterPos3f(mid.x, mid.y + 8, mid.z);
        std::stringstream ss;
        ss << "⚠ LEAK: " << pipe.id << " (" << std::fixed << std::setprecision(1) << pipe.leakRate << " L/s)";
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
        glEnable(GL_LIGHTING);
    }
    
    // Enhanced pipe ID label
    if (showSensorLabels) {
        Vec3 mid = (pipe.start + pipe.end) * 0.5f;
        glDisable(GL_LIGHTING);
        
        if (pipe.type >= SEWAGE_LATERAL) {
            glColor3f(1.0f, 0.7f, 0.3f);  // Bright orange for sewage
        } else {
            glColor3f(0.4f, 0.9f, 1.0f);  // Bright cyan for water
        }
        
        glRasterPos3f(mid.x, mid.y + 2.0f, mid.z);
        for (char c : std:: to_string(pipe.id)) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        glEnable(GL_LIGHTING);
    }
}

void drawBuilding(const Building& bldg) {
    Color baseColor(0.55f, 0.55f, 0.6f);
    Color roofColor(0.4f, 0.4f, 0.45f);
    
    // Enable transparency if toggled
    if (transparentBuildings) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(baseColor.r, baseColor.g, baseColor.b, 0.25f);  // 25% opacity
    } else {
        glColor3f(baseColor.r, baseColor.g, baseColor.b);
    }
    
    // Main building body
    glPushMatrix();
    glTranslatef(bldg.position.x + BUILDING_FOOTPRINT/2, bldg.position.y + bldg.height/2, bldg.position.z + BUILDING_FOOTPRINT/2);
    glScalef(BUILDING_FOOTPRINT, bldg.height, BUILDING_FOOTPRINT);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Rooftop
    if (transparentBuildings) {
        glColor4f(roofColor.r, roofColor.g, roofColor.b, 0.25f);
    } else {
        glColor3f(roofColor.r, roofColor.g, roofColor.b);
    }
    
    glPushMatrix();
    glTranslatef(bldg.position.x + BUILDING_FOOTPRINT/2, bldg.position.y + bldg.height + 0.25f, bldg.position.z + BUILDING_FOOTPRINT/2);
    glScalef(BUILDING_FOOTPRINT, 0.5f, BUILDING_FOOTPRINT);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    if (transparentBuildings) {
        glDisable(GL_BLEND);
    }
    
    // Water riser (blue) - always opaque
    Vec3 riserPos = bldg.position + Vec3(2, 0, 2);
    drawCylinder(riserPos, riserPos + Vec3(0, bldg.height, 0), 0.12f, Color(0.4f, 0.6f, 0.9f));
    
    // Sewage drop (ULTRA BRIGHT ORANGE, thicker) - always opaque
    Vec3 dropPos = bldg.position + Vec3(BUILDING_FOOTPRINT - 2, bldg.height, BUILDING_FOOTPRINT - 2);
    drawCylinder(dropPos, dropPos + Vec3(0, -bldg.height - 1.5f, 0), 0.25f, Color(1.0f, 0.5f, 0.0f));
    
    // Display water consumption info if labels are shown
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        glColor3f(0.2f, 0.8f, 0.2f);
        
        // Show current flow rate
        std::stringstream ss;
        ss << "Bldg " << bldg.id << ": " << std::fixed << std::setprecision(3) 
           << (bldg.currentWaterFlow * 1000.0f) << " L/s";
        
        glRasterPos3f(bldg.position.x + BUILDING_FOOTPRINT/2, 
                      bldg.position.y + bldg.height + 3.0f, 
                      bldg.position.z + BUILDING_FOOTPRINT/2);
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        glEnable(GL_LIGHTING);
    }
}

void drawSewageTreatmentPlant(Vec3 pos) {
    // Primary clarifier
    glColor3f(0.4f, 0.3f, 0.2f);
    glPushMatrix();
    glTranslatef(pos.x - 20, pos.y + 2, pos.z);
    GLUquadric* quad1 = gluNewQuadric();
    gluCylinder(quad1, 12.0f, 12.0f, 4.0f, 32, 1);
    gluDeleteQuadric(quad1);
    glPopMatrix();
    
    // Aeration basin
    glColor3f(0.35f, 0.4f, 0.3f);
    glPushMatrix();
    glTranslatef(pos.x + 10, pos.y + 2, pos.z - 10);
    glScalef(18, 4, 15);
    glutSolidCube(1.0f);
    glPopMatrix();
    
    // Secondary clarifier
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
    
    ss << "ROS2-INTEGRATED SCADA WATER & SEWAGE DIGITAL TWIN - WATER CONSUMPTION SIMULATION";
    drawText(10, windowHeight - 25, ss.str());
    ss.str("");
    
    ss << "Buildings: " << city.buildings.size() << " | Clusters: " << city.clusters.size() 
       << " | Pipes: " << city.pipes.size() << " | Sensors: " << city.sensors.size();
    drawText(10, windowHeight - 50, ss.str());
    ss.str("");
    
    drawText(10, windowHeight - 75, "ROS2 CONTROL ACTIVE - Sensor values updated via ROS2 topics", Color(0.2f, 0.9f, 0.2f));
    
    // Count active leaks
    int leakCount = 0;
    std::string leakIDs = "";
    for (const auto& pipe : city.pipes) {
        if (pipe.hasLeak) {
            leakCount++;
            if (!leakIDs.empty()) leakIDs += ", ";
            leakIDs += std::to_string(pipe.id);
        }
    }
    
    if (leakCount > 0) {
        ss << "ACTIVE LEAKS: " << leakCount << " [Pipe IDs: " << leakIDs << "]";
        drawText(10, windowHeight - 100, ss.str(), Color(1, 0, 0));
        ss.str("");
    } else {
        drawText(10, windowHeight - 100, "NO LEAKS DETECTED (Default state)", Color(0.2f, 0.9f, 0.2f));
    }
    
    // Reservoir status
    ss << "RESERVOIR: Level=" << std::fixed << std::setprecision(2) << city.reservoirLevel << "m (" 
       << std::setprecision(1) << (city.reservoirVolume / city.reservoirCapacity * 100.0f) << "%) | "
       << "Volume=" << std::setprecision(0) << city.reservoirVolume << "/" << city.reservoirCapacity << " m³";
    drawText(10, windowHeight - 125, ss.str());
    ss.str("");
    
    // Total water statistics
    float totalDemand = 0;
    for (const auto& bldg : city.buildings) {
        totalDemand += bldg.currentWaterFlow;
    }
    
    ss << "CITY WATER: Demand=" << std::fixed << std::setprecision(3) << (totalDemand * 1000.0f) << " L/s | "
       << "Consumed=" << std::setprecision(1) << city.totalWaterConsumed << " m³ | "
       << "Leaked=" << std::setprecision(1) << city.totalLeakWater << " m³";
    drawText(10, windowHeight - 150, ss.str());
    ss.str("");
    
    // System state JSON update info
    ss << "SYSTEM STATE JSON: Updating every " << SYSTEM_JSON_UPDATE_INTERVAL << "s | "
       << "Next in: " << std::setprecision(1) << (SYSTEM_JSON_UPDATE_INTERVAL - systemJsonUpdateTimer) << "s";
    drawText(10, windowHeight - 175, ss.str(), Color(0.8f, 0.8f, 0.2f));
    ss.str("");
    
    // Cluster info (show both water and sewage sensors)
    if (highlightedCluster >= 0 && highlightedCluster < (int)city.clusters.size()) {
        const Cluster& c = city.clusters[highlightedCluster];
        const Sensor& waterSensor = city.sensors[c.waterSensorID];
        const Sensor& sewerSensor = city.sensors[c.sewageSensorID];
        
        // Calculate cluster water demand
        float clusterDemand = 0;
        for (int bid : c.buildingIDs) {
            clusterDemand += city.buildings[bid].currentWaterFlow;
        }
        
        ss << "CLUSTER #" << c.id << " (" << c.buildingIDs.size() << " buildings): Water=" 
           << std::fixed << std::setprecision(2) << (clusterDemand * 1000.0f) << " L/s";
        drawText(10, windowHeight - 200, ss.str(), Color(0.5f, 0.8f, 1.0f));
        ss.str("");
        
        ss << "  WATER [ID:" << waterSensor.id << "] V:" << std::setprecision(0) 
           << waterSensor.valveState << "% P:" << waterSensor.pressure << " kPa";
        drawText(30, windowHeight - 225, ss.str(), Color(0.5f, 0.8f, 1.0f));
        ss.str("");
        
        ss << "  SEWER [ID:" << sewerSensor.id << "] V:" << std::setprecision(0) 
           << sewerSensor.valveState << "% L:" << sewerSensor.waterLevel << "%";
        drawText(30, windowHeight - 250, ss.str(), Color(1.0f, 0.6f, 0.0f));
        ss.str("");
    }
    
    // STP status
    if (city.stpSewerSensorID >= 0) {
        const Sensor& sensor = city.sensors[city.stpSewerSensorID];
        ss << "STP SEWER SENSOR [ID:" << sensor.id << "] Level=" << std::setprecision(1) << sensor.waterLevel << "%";
        drawText(10, windowHeight - 275, ss.str());
        ss.str("");
    }
    
    // Building water consumption sample (show first 5 buildings)
    if (city.buildings.size() > 0) {
        drawText(10, windowHeight - 300, "BUILDING WATER CONSUMPTION (sample):", Color(0.2f, 0.9f, 0.2f));
        
        int sampleCount = std::min(5, (int)city.buildings.size());
        for (int i = 0; i < sampleCount; i++) {
            const Building& b = city.buildings[i];
            ss << "  B" << b.id << ": " << std::fixed << std::setprecision(1) 
               << (b.currentWaterFlow * 1000.0f) << " L/s (" 
               << std::setprecision(0) << b.getDailyConsumption() << " m³/day)";
            drawText(30, windowHeight - 325 - (i * 25), ss.str());
            ss.str("");
        }
    }
    
    // DEFAULT VALUES reminder
    drawText(10, 360, "DEFAULT SENSOR VALUES:", Color(0.9f, 0.9f, 0.2f));
    drawText(10, 340, "CONTROLS:");
    drawText(10, 320, "B: Buildings | W: Water (CYAN/BLUE) | S: Sewage (BROWN w/STRIPES)");
    drawText(10, 300, "N: Sensors | L: Leaks | T: Labels | X: Transparent Buildings | G: Ground");
    drawText(10, 280, "P: Water Particles | R: Reset | +/-: Buildings | Q: Quit");
    drawText(10, 260, "H: Highlight | 1-9: Test sensors | Mouse: FULL 360° ROTATION");
    drawText(10, 240, "Mouse Wheel: Zoom | K: Load leak data");
    
    if (transparentBuildings) {
        drawText(10, 220, ">>> BUILDINGS 75% TRANSPARENT <<<", Color(0.2f, 1.0f, 0.2f));
    }
    
    if (showWaterParticles) {
        drawText(10, 200, ">>> WATER PARTICLES ENABLED <<<", Color(0.2f, 0.8f, 1.0f));
    }
    
    drawText(10, 180, "PIPE IDs:");
    drawText(10, 160, "  Water:  pipe_w0, pipe_w1, pipe_w2, ... (CYAN smooth)", Color(0.3f, 0.7f, 1.0f));
    drawText(10, 140, "  Sewage: pipe_s0, pipe_s1, pipe_s2, ... (BROWN striped)", Color(0.8f, 0.5f, 0.2f));
    drawText(10, 120, "");
    
    // SYSTEM STATE JSON UPDATES - FIXED SECTION
    ss << "SYSTEM STATE JSON UPDATES:";
    drawText(10, 100, ss.str());
    ss.str("");
    
    ss << "  • Updates every " << SYSTEM_JSON_UPDATE_INTERVAL << " seconds";
    drawText(10, 80, ss.str());
    ss.str("");
    
    drawText(10, 60, "  • Includes all buildings water consumption");
    drawText(10, 40, "  • Includes all pipe states for RL agent");
    drawText(10, 20, "  • Real-time system monitoring");
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    // Dynamic camera distance based on city size
    float autoCamDistance = std::max(200.0f, city.cityExtent * 2.5f);
    float actualDistance = camDistance == 200.0f ? autoCamDistance : camDistance;
    
    float camX = actualDistance * cos(camElevation * M_PI / 180.0f) * sin(camAngle * M_PI / 180.0f);
    float camY = actualDistance * sin(camElevation * M_PI / 180.0f);
    float camZ = actualDistance * cos(camElevation * M_PI / 180.0f) * cos(camAngle * M_PI / 180.0f);
    
    gluLookAt(camX, camY, camZ, 0, 10, 0, 0, 1, 0);
    
    // Ground - MAKE IT TRANSPARENT so we can see underground sewage
    if (showGround) {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.12f, 0.12f, 0.12f, 0.4f);  // Semi-transparent ground
        float groundSize = city.cityExtent * 2.0f;
        glBegin(GL_QUADS);
        glVertex3f(-groundSize, 0, -groundSize);
        glVertex3f(groundSize, 0, -groundSize);
        glVertex3f(groundSize, 0, groundSize);
        glVertex3f(-groundSize, 0, groundSize);
        glEnd();
        glDisable(GL_BLEND);
        
        // Grid
        glColor3f(0.25f, 0.25f, 0.25f);
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (float i = -groundSize; i <= groundSize; i += 20) {
            glVertex3f(i, 0.1f, -groundSize);
            glVertex3f(i, 0.1f, groundSize);
            glVertex3f(-groundSize, 0.1f, i);
            glVertex3f(groundSize, 0.1f, i);
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
    
    // Draw reservoir with enhanced visualization
    city.drawReservoirWithParticles();
    
    // Draw STP
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
    
    drawHUD();
    
    glutSwapBuffers();
}

void reshape(int w, int h) {
    windowWidth = w;
    windowHeight = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(50.0, (double)w / h, 1.0, 5000.0);
    glMatrixMode(GL_MODELVIEW);
}

void idle() {
    float dt = 0.016f; // 60 FPS
    city.updateSimulation(dt);
    
    // Update system JSON timer
    systemJsonUpdateTimer += dt;
    
    // Export system_state.json every 0.1 seconds
    if (systemJsonUpdateTimer >= SYSTEM_JSON_UPDATE_INTERVAL) {
        city.exportSystemStateJSON("system_state.json");
        systemJsonUpdateTimer = 0.0f;
    }
    
    // Export other JSON files every 60 frames (~1 second at 60fps)
    jsonExportCounter++;
    if (jsonExportCounter >= 60) {
        city.exportSensorJSON("sensor.json");         // Real-time sensor state
        city.exportLeakJSON("leak.json");             // Leak detection
        jsonExportCounter = 0;
    }
    
    // Also try to read sensor.json if it was modified externally
    static time_t lastModTime = 0;
    struct stat fileStat;
    if (stat("sensor.json", &fileStat) == 0) {
        if (fileStat.st_mtime > lastModTime) {
            city.importSensorJSON("sensor.json");
            lastModTime = fileStat.st_mtime;
        }
    }
    
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
        case 'n': case 'N':
            showSensors = !showSensors;
            break;
        case 'l': case 'L':
            showLeaks = !showLeaks;
            break;
        case 't': case 'T':
            showSensorLabels = !showSensorLabels;
            break;
        case 'p': case 'P':
            showWaterParticles = !showWaterParticles;
            std::cout << "Water particles: " << (showWaterParticles ? "ON" : "OFF") << "\n";
            break;
        case 'g': case 'G':
            showGround = !showGround;
            std::cout << "Ground visibility: " << (showGround ? "ON (transparent)" : "OFF") << "\n";
            break;
        case 'x': case 'X':
            transparentBuildings = !transparentBuildings;
            std::cout << "Buildings transparency: " << (transparentBuildings ? "ON (25% opacity)" : "OFF (solid)") << "\n";
            break;
        case 'h': case 'H':
            highlightedCluster++;
            if (highlightedCluster >= (int)city.clusters.size()) {
                highlightedCluster = -1;
            }
            break;
        case 'k': case 'K':
            city.loadLeakData("leaks.csv");
            break;
        case 'r': case 'R':
            camAngle = 45.0f;
            camElevation = 45.0f;
            camDistance = 200.0f;
            break;
        case '+': case '=':
            currentBuildingCount = std::min(100000, currentBuildingCount + 10);
            city.generateCity(currentBuildingCount);
            highlightedCluster = -1;
            break;
        case '-': case '_':
            currentBuildingCount = std::max(10, currentBuildingCount - 10);
            city.generateCity(currentBuildingCount);
            highlightedCluster = -1;
            break;
        case '1': case '2': case '3': case '4': case '5': 
        case '6': case '7': case '8': case '9': {
            // Test: Update sensor 0-8 valve states
            int sensorID = key - '1';
            if (sensorID < (int)city.sensors.size()) {
                float newValve = (city.sensors[sensorID].valveState > 50) ? 20.0f : 100.0f;
                city.updateFromROS2(sensorID, newValve, DEFAULT_PRESSURE, DEFAULT_WATER_LEVEL);
                std::cout << "TEST: Toggled Sensor " << sensorID << " valve to " << newValve << "%\n";
            }
            break;
        }
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
        camDistance = std::max(50.0f, camDistance - 20.0f);
    } else if (button == 4) {
        camDistance = std::min(2000.0f, camDistance + 20.0f);
    }
}

void motion(int x, int y) {
    if (mouseLeftDown) {
        camAngle += (x - lastMouseX) * 0.5f;
        // ALLOW FULL 360 DEGREE VERTICAL ROTATION - no limits!
        camElevation -= (y - lastMouseY) * 0.5f;
        // Wrap around for full rotation
        if (camElevation > 180.0f) camElevation -= 360.0f;
        if (camElevation < -180.0f) camElevation += 360.0f;
        lastMouseX = x;
        lastMouseY = y;
    }
}

void printInstructions() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   ROS2-INTEGRATED SCADA WATER & SEWAGE DIGITAL TWIN          ║\n";
    std::cout << "║   WITH WATER CONSUMPTION SIMULATION                          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "NEW FEATURES ADDED:\n";
    std::cout << "  1. Water Consumption Tracking per Building\n";
    std::cout << "  2. Enhanced Reservoir Graphics (realistic 3D model)\n";
    std::cout << "  3. Water Particles in Pipes for visual flow simulation\n";
    std::cout << "  4. Building-specific water usage patterns\n";
    std::cout << "  5. System State JSON updates every " << SYSTEM_JSON_UPDATE_INTERVAL << " seconds\n\n";
    
    std::cout << "SYSTEM STATE JSON FOR RL AGENT:\n";
    std::cout << "  • Updates every " << SYSTEM_JSON_UPDATE_INTERVAL << " seconds (live simulation)\n";
    std::cout << "  • Includes ALL building water consumption data\n";
    std::cout << "  • Includes ALL pipe states (flow, leaks, valve positions)\n";
    std::cout << "  • Comprehensive state for reinforcement learning\n";
    std::cout << "  • Real-time system monitoring\n\n";
    
    std::cout << "WATER CONSUMPTION MODEL:\n";
    std::cout << "  • Each building has random daily water usage pattern\n";
    std::cout << "  • Base consumption: " << (WATER_CONSUMPTION_BASE * 1000.0f) << " L/s per building\n";
    std::cout << "  • Daily variation: ±" << (DAILY_VARIATION * 100.0f) << "%\n";
    std::cout << "  • Peaks during morning and evening hours\n\n";
    
    std::cout << "RESERVOIR SIMULATION:\n";
    std::cout << "  • Capacity: " << city.reservoirCapacity << " m³\n";
    std::cout << "  • Realistic 3D model with water surface\n";
    std::cout << "  • Water particles show outflow\n";
    std::cout << "  • Level decreases based on city demand\n\n";
    
    std::cout << "PARTICLE SYSTEM:\n";
    std::cout << "  • Water particles flow through pipes\n";
    std::cout << "  • Each building has unique water color\n";
    std::cout << "  • Particles in reservoir show water movement\n";
    std::cout << "  • Toggle with 'P' key\n\n";
    
    std::cout << "DATA OUTPUT:\n";
    std::cout << "  • system_state.json: Updates every " << SYSTEM_JSON_UPDATE_INTERVAL << "s (RL agent input)\n";
    std::cout << "  • Building water consumption data\n";
    std::cout << "  • Real-time flow rates per building\n";
    std::cout << "  • Total city water demand\n";
    std::cout << "  • Reservoir volume tracking\n";
    std::cout << "  • All pipe states (220+ pipes)\n\n";
    
    std::cout << "ROS2 INTEGRATION:\n";
    std::cout << "  • All sensors controlled via ROS2 topics\n";
    std::cout << "  • Unique sensor IDs for all control points\n";
    std::cout << "  • Real-time valve/pressure/level updates\n\n";
    
    std::cout << "SENSOR ARCHITECTURE:\n";
    std::cout << "  • Each sensor = Valve + Pressure Sensor + Level Sensor\n";
    std::cout << "  • WATER SIDE:\n";
    std::cout << "      - Reservoir: 1 water sensor\n";
    std::cout << "      - Per Cluster: 1 water sensor\n";
    std::cout << "      - Per Building: 1 water sensor\n";
    std::cout << "  • SEWAGE SIDE:\n";
    std::cout << "      - Per Building: 1 sewage sensor\n";
    std::cout << "      - Per Cluster: 1 sewage sensor\n";
    std::cout << "      - STP: 1 sewage sensor\n\n";
    
    std::cout << "DEFAULT VALUES:\n";
    std::cout << "  • Valve: 100% (ON)\n";
    std::cout << "  • Pressure: 15 kPa\n";
    std::cout << "  • Water Level: 70%\n";
    std::cout << "  • NO LEAKS (leak logic external)\n\n";
    
    std::cout << "UNIQUE IDs:\n";
    std::cout << "  • All pipes have unique IDs (0 to N)\n";
    std::cout << "  • All sensors have unique IDs (0 to M)\n";
    std::cout << "  • Leak file references pipe IDs\n\n";
    
    std::cout << "SEWAGE VISIBILITY:\n";
    std::cout << "  • Sewage pipes: 3x thicker than water\n";
    std::cout << "  • Bright orange color (1.0, 0.55, 0.0)\n";
    std::cout << "  • Visible sewage drops in buildings\n\n";
    
    std::cout << "DYNAMIC SCALING:\n";
    std::cout << "  • Reservoir/STP auto-position based on city size\n";
    std::cout << "  • Camera auto-adjusts for 10-100000 buildings\n";
    std::cout << "  • NO overlap with buildings\n\n";
    
    std::cout << "LEAK DATA FILE (leaks.csv):\n";
    std::cout << "  Format: pipeID,hasLeak,leakRate\n";
    std::cout << "  Example:\n";
    std::cout << "    13,1,15.723\n";
    std::cout << "    35,1,20.1\n\n";
    
    std::cout << "ROS2 TOPICS (Python interface):\n";
    std::cout << "  Publish: /water_scada/sensor_update\n";
    std::cout << "    Message: {sensor_id, valve, pressure, level}\n";
    std::cout << "  Subscribe: /water_scada/sensor_state\n";
    std::cout << "    Message: {sensor_id, valve, pressure, level}\n\n";
    
    std::cout << "CONTROLS:\n";
    std::cout << "  B - Toggle buildings\n";
    std::cout << "  W - Toggle water network\n";
    std::cout << "  S - Toggle sewage network (BRIGHT ORANGE UNDERGROUND)\n";
    std::cout << "  N - Toggle sensors\n";
    std::cout << "  L - Toggle leaks\n";
    std::cout << "  T - Toggle sensor labels (shows water consumption)\n";
    std::cout << "  P - Toggle water particles in pipes\n";
    std::cout << "  X - Toggle building transparency (SEE THROUGH!)\n";
    std::cout << "  G - Toggle ground visibility\n";
    std::cout << "  H - Highlight cluster\n";
    std::cout << "  K - Load leak data file\n";
    std::cout << "  1-9 - Test toggle sensors 0-8 valves\n";
    std::cout << "  +/- - Add/remove buildings (10 to 100000)\n";
    std::cout << "  R - Reset camera\n";
    std::cout << "  Mouse Drag - Rotate camera\n";
    std::cout << "  Mouse Wheel - Zoom\n";
    std::cout << "  Q - Quit\n";
    std::cout << "══════════════════════════════════════════════════════════════\n\n";
    
    std::cout << "VISUALIZATION TIPS:\n";
    std::cout << "  1. Press 'P' to see water particles flowing through pipes\n";
    std::cout << "  2. Press 'T' to see building water consumption rates\n";
    std::cout << "  3. Rotate camera to look from above-down angle\n";
    std::cout << "  4. Zoom in on reservoir to see water particles\n";
    std::cout << "  5. Check system_state.json for RL agent input data\n\n";
    
    std::cout << "SENSOR NAMING:\n";
    std::cout << "  Water sensors:  sensor_f0, sensor_f1, sensor_f2, ...\n";
    std::cout << "  Sewage sensors: sensor_s0, sensor_s1, sensor_s2, ...\n\n";
    
    // In printInstructions() function, find this section:
std::cout << "SYSTEM STATE JSON UPDATES:\n";
std::cout << "  • Updates every " << SYSTEM_JSON_UPDATE_INTERVAL << " seconds\n";
std::cout << "  • Includes ALL building water consumption data\n";
std::cout << "  • Includes ALL pipe states (flow, leaks, valve positions)\n";
std::cout << "  • Comprehensive state for reinforcement learning\n";
std::cout << "  • Real-time system monitoring\n\n";
}

int main(int argc, char** argv) {
    srand(time(nullptr));
    
    printInstructions();
    
    std::cout << "Generating city with " << currentBuildingCount << " buildings...\n";
    city.generateCity(currentBuildingCount);
    city.generateInitSensorJSON("init_sensor.json");
    
    std::cout << "\nAll sensors initialized with DEFAULT values:\n";
    std::cout << "  Valve: 100% (ON) | Pressure: 15 kPa | Level: 70%\n";
    std::cout << "  NO LEAKS (default state)\n\n";
    
    std::cout << "Water consumption simulation ACTIVE:\n";
    std::cout << "  Each building has unique water usage pattern\n";
    std::cout << "  Reservoir tracks total city water demand\n";
    std::cout << "  Water particles visualize flow in pipes\n\n";
    
    std::cout << "System State JSON updates:\n";
    std::cout << "  File: system_state.json\n";
    std::cout << "  Update interval: " << SYSTEM_JSON_UPDATE_INTERVAL << " seconds\n";
    std::cout << "  Contains data for " << city.buildings.size() << " buildings and " << city.pipes.size() << " pipes\n\n";
    
    std::cout << "Attempting to load leak data...\n";
    city.loadLeakData("leaks.csv");
    
    std::cout << "\n=== SENSOR IDs ===\n";
    std::cout << "WATER SIDE:\n";
    std::cout << "  Reservoir water sensor: " << city.reservoirWaterSensorID << "\n";
    for (int i = 0; i < std::min(3, (int)city.clusters.size()); i++) {
        std::cout << "  Cluster " << i << " water sensor: " << city.clusters[i].waterSensorID << "\n";
    }
    if (city.clusters.size() > 3) std::cout << "  ... (+" << (city.clusters.size() - 3) << " more clusters)\n";
    if (!city.buildings.empty()) {
        std::cout << "  Building 0 water sensor: " << city.buildings[0].waterSensorID << "\n";
    }
    
    std::cout << "\nSEWAGE SIDE:\n";
    if (!city.buildings.empty()) {
        std::cout << "  Building 0 sewage sensor: " << city.buildings[0].sewageSensorID << "\n";
    }
    for (int i = 0; i < std::min(3, (int)city.clusters.size()); i++) {
        std::cout << "  Cluster " << i << " sewage sensor: " << city.clusters[i].sewageSensorID << "\n";
    }
    if (city.clusters.size() > 3) std::cout << "  ... (+" << (city.clusters.size() - 3) << " more clusters)\n";
    std::cout << "  STP sewage sensor: " << city.stpSewerSensorID << "\n";
    
    std::cout << "\nTotal sensors: " << city.sensors.size() << "\n";
    std::cout << "  Water sensors: " << (1 + city.clusters.size() + city.buildings.size()) << "\n";
    std::cout << "  Sewage sensors: " << (city.buildings.size() + city.clusters.size() + 1) << "\n\n";
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("ROS2 SCADA Water & Sewage Digital Twin - Water Consumption Simulation");
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);  // Add second light
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    // CRITICAL: Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Main light - brighter and more dramatic
    GLfloat lightPos[] = {200.0f, 400.0f, 200.0f, 1.0f};
    GLfloat lightAmb[] = {0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat lightDif[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightSpec[] = {1.0f, 1.0f, 1.0f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);
    
    // Second fill light from below (for underground visibility)
    GLfloat lightPos2[] = {-100.0f, -200.0f, 100.0f, 1.0f};
    GLfloat lightAmb2[] = {0.3f, 0.3f, 0.35f, 1.0f};
    GLfloat lightDif2[] = {0.6f, 0.6f, 0.7f, 1.0f};
    
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos2);
    glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmb2);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDif2);
    
    glClearColor(0.05f, 0.06f, 0.08f, 1.0f);  // Darker background for better contrast
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);
    glEnable(GL_RESCALE_NORMAL);
    
    // Enhanced depth and alpha testing
    glDepthMask(GL_TRUE);
    glAlphaFunc(GL_GREATER, 0.1f);
    glEnable(GL_ALPHA_TEST);
    
    // Anti-aliasing hints
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    
    std::cout << "ROS2 SCADA Digital Twin with Water Consumption Simulation running...\n";
    std::cout << "System State JSON updating every " << SYSTEM_JSON_UPDATE_INTERVAL << " seconds\n";
    std::cout << "Ready for ROS2 sensor updates and external leak data.\n\n";
    std::cout << "Press 'P' to toggle water particles visualization\n";
    std::cout << "Press 'T' to see building water consumption rates\n";
    std::cout << "Check system_state.json for RL agent input data\n\n";
    
    glutMainLoop();
    return 0;
}