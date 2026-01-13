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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <csignal>

// ============================================================================
// UDP BROADCAST SETTINGS
// ============================================================================

const int UDP_BROADCAST_PORT = 8888;
const float UDP_BROADCAST_INTERVAL = 0.1f; // 10 Hz
float udpBroadcastTimer = 0.0f;
int udpSocket = -1;
struct sockaddr_in broadcastAddr;
std::mutex udpMutex;
std::queue<std::string> udpMessageQueue;
std::atomic<bool> udpRunning(false);
std::thread* udpThread = nullptr;
std::mutex render_mutex;
std::atomic<bool> simulation_running(true);
std::mutex city_mutex;
std::atomic<bool> simulation_active{true};

// ============================================================================
// ROS2 HUMBLE INTEGRATION - IMPROVED VERSION
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <json/json.h>

// ROS2 globals
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros2_executor = nullptr;
std::shared_ptr<std::thread> ros2_thread = nullptr;
std::mutex sensor_mutex;
bool ros2_initialized = false;
bool ros2_running = false;
std::atomic<bool> g_shutdown_requested{false};

// ============================================================================
// ENGINEERING CONSTANTS
// ============================================================================

const int BUILDINGS_PER_CLUSTER = 10;
const float CITY_GRID_SPACING = 20.0f;
const float CLUSTER_SPACING = 80.0f;
int jsonExportCounter = 0;
const float SYSTEM_JSON_UPDATE_INTERVAL = 0.1f;
float systemJsonUpdateTimer = 0.0f;

const int FLOOR_OPTIONS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15};
const int NUM_FLOOR_OPTIONS = 11;
const float FLOOR_HEIGHT = 3.0f;
const float BUILDING_FOOTPRINT = 10.0f;

const float TRUNK_DIAMETER = 0.8f;
const float SECONDARY_DIAMETER = 0.4f;
const float RING_DIAMETER = 0.25f;
const float SERVICE_DIAMETER = 0.05f;

const float DEFAULT_VALVE_STATE = 100.0f;
const float DEFAULT_PRESSURE = 150.0f; // kPa (1.5 bar)
const float DEFAULT_WATER_LEVEL = 70.0f;

const float WATER_CONSUMPTION_BASE = 0.001f; // m³/s per building
const float DAILY_VARIATION = 0.3f;
const float PARTICLE_SPEED = 2.0f;
const int PARTICLES_PER_PIPE = 20;
const float PARTICLE_SIZE = 0.15f;

// ============================================================================
// PHYSICS CONSTANTS
// ============================================================================

const float GRAVITY = 9.81f; // m/s²
const float WATER_DENSITY = 1000.0f; // kg/m³
const float PIPE_ROUGHNESS = 0.000015f; // Smooth concrete
const float DYNAMIC_VISCOSITY = 0.001002f; // Pa·s at 20°C

// ============================================================================
// DAY-NIGHT CYCLE
// ============================================================================

struct Color {
    float r, g, b, a;
    Color(float r = 1, float g = 1, float b = 1, float a = 1.0f) : r(r), g(g), b(b), a(a) {}
};

struct DayNightCycle {
    float timeOfDay; // 0-24 hours
    float sunAngle;
    float ambientLight;
    float sunIntensity;
    Color skyColor;
    Color sunColor;
    Color moonColor;
    
    DayNightCycle() : timeOfDay(8.0f), sunAngle(0), ambientLight(0.8f), 
                     sunIntensity(1.0f), skyColor(0.05f, 0.06f, 0.08f), 
                     sunColor(1.0f, 0.9f, 0.7f), moonColor(0.8f, 0.8f, 0.9f) {}
    
    void update(float dt) {
        timeOfDay += dt / 3600.0f; // Convert dt to hours
        if (timeOfDay >= 24.0f) timeOfDay -= 24.0f;
        
        // Calculate lighting based on time
        float dawn = 5.0f, sunrise = 6.0f, sunset = 18.0f, dusk = 19.0f;
        
        if (timeOfDay >= dawn && timeOfDay <= sunrise) {
            float t = (timeOfDay - dawn) / (sunrise - dawn);
            ambientLight = 0.2f + 0.6f * t;
            sunIntensity = 0.3f + 0.7f * t;
            sunColor = Color(1.0f, 0.6f + 0.4f * t, 0.3f + 0.4f * t);
        }
        else if (timeOfDay >= sunrise && timeOfDay <= sunset) {
            ambientLight = 0.8f;
            sunIntensity = 1.0f;
            sunColor = Color(1.0f, 0.9f, 0.7f);
        }
        else if (timeOfDay >= sunset && timeOfDay <= dusk) {
            float t = (timeOfDay - sunset) / (dusk - sunset);
            ambientLight = 0.8f - 0.6f * t;
            sunIntensity = 1.0f - 0.7f * t;
            sunColor = Color(1.0f, 0.9f - 0.3f * t, 0.7f - 0.4f * t);
        }
        else {
            ambientLight = 0.2f;
            sunIntensity = 0.3f;
        }
    }

    int getDayOfWeek() const {
        // Calculate day of week based on simulation time
        // Each day = 24 hours, Monday=0, Sunday=6
        int days = static_cast<int>(timeOfDay / 24.0f);
        return days % 7;
    }
    
    float getWaterDemandFactor() const {
        // Residential water usage pattern
        float peakMorning = 7.0f;
        float peakEvening = 19.0f;
        float baseDemand = 0.3f;
        
        float morningFactor = exp(-pow(timeOfDay - peakMorning, 2) / 2.0f);
        float eveningFactor = exp(-pow(timeOfDay - peakEvening, 2) / 2.0f);
        float dayFactor = (timeOfDay >= 6.0f && timeOfDay <= 22.0f) ? 0.2f : 0.0f;
        
        return baseDemand + 0.5f * morningFactor + 0.6f * eveningFactor + dayFactor;
    }
    
    bool isWeekend() const {
        // Simple simulation - assume every 7th day is weekend
        int dayNumber = static_cast<int>(timeOfDay / 24.0f) % 7;
        return (dayNumber == 0 || dayNumber == 6); // Saturday or Sunday
    }
    
    std::string getDayType() const {
        return isWeekend() ? "weekend" : "weekday";
    }
    
    bool isDay() const {
        return (timeOfDay >= 6.0f && timeOfDay <= 18.0f);
    }
};

DayNightCycle dayNight;

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
    float dot(const Vec3& v) const {
        return x*v.x + y*v.y + z*v.z;
    }
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

class CityNetwork;
class ROS2SensorSubscriber;
struct Pipe;

// ============================================================================
// WATER PARTICLE STRUCTURE
// ============================================================================

struct WaterParticle {
    // Two different constructors for different use cases
    // Constructor for pipe particles
    WaterParticle(float t0, float spd, Color col, int pid)
        : t(t0), speed(spd), life(1.0f), color(col), pipeID(pid),
          position(0,0,0), direction(0,0,0) {
        // Random radial position within pipe
        radial_r = static_cast<float>(rand()) / RAND_MAX * 0.8f; // Keep away from walls
        radial_theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
    }
    
    // Constructor for reservoir particles
    WaterParticle(Vec3 pos, Vec3 dir, float spd, Color col, int pid)
        : position(pos), direction(dir), speed(spd), life(1.0f), color(col), pipeID(pid),
          t(0), radial_r(0), radial_theta(0) {}
    
    // Pipe particles
    float t;           // 0→1 along pipe length
    float radial_r;    // Radial distance (0→1)
    float radial_theta; // Radial angle
    
    // Reservoir particles
    Vec3 position;
    Vec3 direction;
    
    // Common
    float speed;       // Speed
    float life;
    Color color;
    int pipeID;
    
    void update(float dt) {
        if (pipeID >= 0) {
            // Pipe particle update
            t += speed * dt;  // Move along pipe
            life -= dt * 0.3f; // Shorter life for faster particles
            if (life < 0) life = 0;
            
            // Add some turbulence
            radial_theta += (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.5f * dt;
        } else {
            // Reservoir particle update
            position = position + direction * (speed * dt);
            life -= dt * 0.2f;
            if (life < 0) life = 0;
        }
    }
    
    Vec3 getWorldPosition(const Pipe& pipe) const;
};

// ============================================================================
// ZONE MANAGEMENT
// ============================================================================

struct Zone {
    int id;
    std::vector<int> buildingIDs;
    std::vector<int> pipeIDs;
    std::vector<int> sensorIDs;
    Vec3 center;
    float avgPressure;
    float minPressure;
    float maxPressure;
    float totalFlow;
    float pressureVariance;
    float flowToPressureRatio;
    std::vector<float> historicalFlow;
    bool leakFlag;
    float leakSeverity;
    bool overflowFlag;
    bool pressureViolation;
    
    Zone(int id, Vec3 center) : id(id), center(center), 
        avgPressure(DEFAULT_PRESSURE), minPressure(DEFAULT_PRESSURE), 
        maxPressure(DEFAULT_PRESSURE), totalFlow(0), pressureVariance(0),
        flowToPressureRatio(0), leakFlag(false), leakSeverity(0),
        overflowFlag(false), pressureViolation(false) {}
    
    void updateStatistics(float currentFlow, float currentPressure) {
        // Update historical flow (keep last 100 readings)
        historicalFlow.push_back(currentFlow);
        if (historicalFlow.size() > 100) {
            historicalFlow.erase(historicalFlow.begin());
        }
        
        // Update statistics
        totalFlow = currentFlow;
        avgPressure = (avgPressure * 0.9f) + (currentPressure * 0.1f);
        
        if (currentPressure < minPressure) minPressure = currentPressure;
        if (currentPressure > maxPressure) maxPressure = currentPressure;
        
        // Calculate variance (simplified)
        pressureVariance = fabs(currentPressure - avgPressure);
        
        // Calculate flow to pressure ratio
        if (currentPressure > 0.1f) {
            flowToPressureRatio = currentFlow / currentPressure;
        }
        
        // Check for violations
        pressureViolation = (currentPressure < 100.0f || currentPressure > 300.0f);
    }
};

// ============================================================================
// SENSOR STRUCTURE
// ============================================================================

enum SensorType {
    WATER_SENSOR,
    SEWAGE_SENSOR
};

struct Sensor {
    int id;
    std::string name;
    SensorType type;
    Vec3 position;
    int zoneID;
    
    float valveState;
    float pressure;
    float waterLevel;
    
    bool active;
    int connectedPipeID;
    
    // Valve control
    time_t lastActionTime;
    int recentActionCount;
    float targetValveState;
    bool pressureFromROS = false;
    
    // Last update source tracking
    int last_esp_id;
    time_t last_update_time;
    
    float cumulativeVolume_m3 = 0.0f;
    float hourlyVolume_m3[24] = {0};
    float dailyVolume_m3 = 0.0f;
    time_t lastVolumeUpdate = 0;
    
    Sensor(int id, std::string name, SensorType type, Vec3 pos, int zoneID = -1)
        : id(id), name(name), type(type), position(pos), zoneID(zoneID),
          valveState(DEFAULT_VALVE_STATE),
          pressure(DEFAULT_PRESSURE),
          waterLevel(DEFAULT_WATER_LEVEL),
          active(true), connectedPipeID(-1),
          last_esp_id(-1), last_update_time(0),
          lastActionTime(time(nullptr)), recentActionCount(0),
          targetValveState(DEFAULT_VALVE_STATE) {}
    
    Color getColor() const {
        if (!active) return Color(0.3f, 0.3f, 0.3f);
        
        if (type == WATER_SENSOR) {
            if (valveState > 80.0f) return Color(0.2f, 0.7f, 1.0f);
            else if (valveState > 20.0f) return Color(0.5f, 0.8f, 0.9f);
            else return Color(0.3f, 0.5f, 0.7f);
        } else {
            if (valveState > 80.0f) return Color(1.0f, 0.6f, 0.0f);
            else if (valveState > 20.0f) return Color(0.9f, 0.7f, 0.3f);
            else return Color(0.6f, 0.4f, 0.2f);
        }
    }
    
    std::string getTypeString() const {
        return (type == WATER_SENSOR) ? "WATER" : "SEWAGE";
    }
    
    void updateFromROS2(float valve, float press, float level, int esp_id) {
        // Removed lock here - locking will be done at CityNetwork level
        valveState = valve;
        pressure = press;
        waterLevel = level;
        last_esp_id = esp_id;
        last_update_time = time(nullptr);
        pressureFromROS = true;
    }
    
    bool isStale() const {
        return (time(nullptr) - last_update_time) > 5; // 5 seconds stale
    }

    bool isROSFresh() const {
        return !isStale();
    }

    void updateVolume(float flow_m3s, float dt) {
        float volume = flow_m3s * dt;
        cumulativeVolume_m3 += volume;
        
        // Update hourly tracking
        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);
        int currentHour = timeinfo->tm_hour;
        
        if (lastVolumeUpdate == 0) {
            lastVolumeUpdate = now;
        } else if (now - lastVolumeUpdate >= 3600) {
            // Reset hourly tracking (simplified)
            hourlyVolume_m3[(currentHour + 23) % 24] = 0;
            lastVolumeUpdate = now;
        }
        
        hourlyVolume_m3[currentHour] += volume;
    }
    
    float getHourlyUsage(int hour) const {
        if (hour >= 0 && hour < 24) {
            return hourlyVolume_m3[hour];
        }
        return 0.0f;
    }
    
    void setValve(float target, bool force = false) {
        time_t now = time(nullptr);
        if (!force && (now - lastActionTime) < 1) { // 1 second hysteresis
            recentActionCount++;
            if (recentActionCount > 10) { // Prevent chattering
                return;
            }
        } else {
            recentActionCount = 0;
        }
        
        targetValveState = std::max(0.0f, std::min(100.0f, target));
        lastActionTime = now;
    }
    
    void updateValve(float dt) {
        // Smooth valve movement (10% per second)
        float valveSpeed = 10.0f * dt;
        if (valveState < targetValveState) {
            valveState = std::min(valveState + valveSpeed, targetValveState);
        } else if (valveState > targetValveState) {
            valveState = std::max(valveState - valveSpeed, targetValveState);
        }
    }
};

// ============================================================================
// PIPE MODEL WITH PHYSICS
// ============================================================================

enum PipeType { 
    TRUNK_MAIN, SECONDARY_MAIN, RING_MAIN, SERVICE_PIPE,
    SEWAGE_LATERAL, SEWAGE_COLLECTOR, SEWAGE_INTERCEPTOR
};

// Forward declarations
class CityNetwork;

struct Pipe {
    int id;
    Vec3 start, end;
    PipeType type;
    float diameter;
    float length;
    float elevationChange;
    int zoneID;
    
    bool hasLeak;
    float leakRate;
    float flowRate;
    float pressureDrop;
    float velocity;
    
    std::vector<WaterParticle> particles;
    
    Pipe(int id, Vec3 s, Vec3 e, PipeType t, float diam, int zoneID = -1)
        : id(id), start(s), end(e), type(t), diameter(diam),
          hasLeak(false), leakRate(0.0f), flowRate(0.0f),
          pressureDrop(0.0f), velocity(0.0f), zoneID(zoneID) {
        length = (end - start).length();
        elevationChange = end.y - start.y;
    }
    
    // Calculate pressure drop using Darcy-Weisbach equation
    float calculatePressureDrop(float flow, float viscosity = DYNAMIC_VISCOSITY) {
        if (diameter <= 0 || length <= 0) return 0.0f;
        
        // Calculate flow velocity
        float area = M_PI * diameter * diameter / 4.0f;
        velocity = flow / area;
        
        // Reynolds number
        float Re = (WATER_DENSITY * velocity * diameter) / viscosity;
        
        // Friction factor (using Churchill equation for simplicity)
        float frictionFactor;
        if (Re < 2000) {
            frictionFactor = 64.0f / Re; // Laminar flow
        } else {
            // Colebrook-White approximation for turbulent flow
            float rough = PIPE_ROUGHNESS / diameter;
            float A = rough / 3.7;
            float B = 5.74f / pow(Re, 0.9);
            frictionFactor = 0.25f / pow(log10(A + B), 2);
        }
        
        // Darcy-Weisbach equation
        pressureDrop = frictionFactor * (length / diameter) * 
                      (WATER_DENSITY * velocity * velocity / 2.0f);
        
        // Add elevation head
        pressureDrop += WATER_DENSITY * GRAVITY * elevationChange;
        
        return pressureDrop;
    }
    
    Color getColor() const {
        if (type >= SEWAGE_LATERAL) {
            return Color(0.65f, 0.38f, 0.12f);
        } else {
            // Color based on velocity (blue gradient)
            float intensity = std::min(1.0f, velocity / 5.0f);
            return Color(0.2f, 0.4f + intensity * 0.5f, 0.8f + intensity * 0.2f);
        }
    }
    
    void updateParticles(float dt, Color particleColor = Color(0.3f, 0.6f, 1.0f, 0.8f)) {
        // Update existing particles
        for (auto it = particles.begin(); it != particles.end();) {
            it->update(dt);
            
            // Remove particles that reached end of pipe or died
            if (it->t > 1.0f || it->life <= 0.0f) {
                it = particles.erase(it);
            } else {
                ++it;
            }
        }
        
        // Spawn new particles based on flow rate
        if (type < SEWAGE_LATERAL && flowRate > 0.05f) {
            int maxParticles = PARTICLES_PER_PIPE;
            if (type == TRUNK_MAIN) maxParticles *= 3;
            else if (type == SECONDARY_MAIN) maxParticles *= 2;
            
            // Spawn rate proportional to flow
            float spawnProbability = (flowRate / 10.0f) * dt * 60.0f;
            
            if (particles.size() < maxParticles && 
                static_cast<float>(rand()) / RAND_MAX < spawnProbability) {
                
                // Calculate speed based on flow velocity
                float area = M_PI * diameter * diameter / 4.0f;
                float velocity = flowRate / area;
                float normalizedSpeed = (velocity / 5.0f) * (1.0f / length);
                
                // Random color variation
                Color col = particleColor;
                col.r *= (0.7f + 0.3f * static_cast<float>(rand()) / RAND_MAX);
                col.g *= (0.7f + 0.3f * static_cast<float>(rand()) / RAND_MAX);
                col.b *= (0.8f + 0.2f * static_cast<float>(rand()) / RAND_MAX);
                
                // Spawn at beginning of pipe
                particles.emplace_back(0.0f, normalizedSpeed, col, id);
            }
        }
    }
    
    void drawParticles() const {
        if (particles.empty()) return;
        
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);
        
        for (const auto& particle : particles) {
            Vec3 pos = particle.getWorldPosition(*this);
            
            // Size based on flow velocity and life
            float area = M_PI * diameter * diameter / 4.0f;
            float velocity = flowRate / area;
            float sizeScale = 0.5f + velocity * 0.1f;
            
            glColor4f(particle.color.r, particle.color.g, particle.color.b, 
                     particle.life * 0.8f);
            glPushMatrix();
            glTranslatef(pos.x, pos.y, pos.z);
            glutSolidSphere(PARTICLE_SIZE * sizeScale * particle.life, 8, 8);
            glPopMatrix();
        }
        
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }

    void drawLeakParticles(float simulationTime) const;
};

void Pipe::drawLeakParticles(float simulationTime) const {
    if (!hasLeak || leakRate < 0.1f) return;
    
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    
    Vec3 pipeCenter = (start + end) * 0.5f;
    
    // Draw leak jet
    glColor4f(1.0f, 0.3f, 0.1f, 0.6f);
    glPushMatrix();
    glTranslatef(pipeCenter.x, pipeCenter.y, pipeCenter.z);
    glRotatef(90, 1, 0, 0);
    glutSolidCone(leakRate * 0.05f, leakRate * 0.3f, 12, 3);
    glPopMatrix();
    
    // Draw spray particles
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < 20; i++) {
        float angle = i * M_PI * 2.0f / 20.0f + simulationTime * 2.0f;
        float dist = 2.0f + sinf(simulationTime * 3.0f + i) * 1.5f;
        float height = 3.0f + sinf(angle * 2 + simulationTime * 4.0f);
        
        Vec3 pos = pipeCenter + Vec3(
            cos(angle) * dist,
            height,
            sin(angle) * dist
        );
        
        float alpha = 0.3f + 0.5f * sinf(simulationTime * 5.0f + i);
        glColor4f(1.0f, 0.5f + 0.3f * sinf(i), 0.2f, alpha);
        glVertex3f(pos.x, pos.y, pos.z);
    }
    glEnd();
    
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

Vec3 WaterParticle::getWorldPosition(const Pipe& pipe) const {
    Vec3 axis = pipe.end - pipe.start;
    float length = axis.length();
    Vec3 dir = axis.normalized();
    
    // Create local coordinate frame
    Vec3 up(0, 1, 0);
    if (fabs(dir.y) > 0.99f) up = Vec3(1, 0, 0);
    Vec3 right = dir.cross(up).normalized();
    up = right.cross(dir).normalized();
    
    // Radial offset
    float pipeRadius = pipe.diameter / 2.0f;
    float actualRadius = radial_r * pipeRadius * 0.9f; // Stay inside pipe
    
    Vec3 radialOffset = 
        right * (cosf(radial_theta) * actualRadius) +
        up * (sinf(radial_theta) * actualRadius);
    
    // Position along pipe
    Vec3 axialPos = pipe.start + dir * (t * length);
    
    return axialPos + radialOffset;
}

// ============================================================================
// BUILDING MODEL WITH REALISTIC CONSUMPTION
// ============================================================================

enum BuildingType {
    RESIDENTIAL,
    COMMERCIAL,
    HOSTEL,
    HOSPITAL,
    INDUSTRIAL
};

struct Building {
    // Core identification
    int id, clusterID, zoneID;
    Vec3 position;
    int numFloors;
    float height;
    
    // Infrastructure connections
    int waterSensorID;
    int sewageSensorID;
    int servicePipeID;
    int sewerPipeID;
    
    // Building characteristics
    BuildingType type;
    int population;              // REAL number of people/occupants
    float sewageFactor;          // 0.5 → 1.5 (water to sewage conversion)
    float randomDemandBias;      // Persistent randomness (0.6-1.4)
    float demandVolatility;      // Hour-to-hour variation factor
    
    // Water usage metrics
    float dailyWaterUsage;       // m³/day
    float currentWaterFlow;      // m³/s
    float totalWaterConsumed;    // m³ (lifetime)
    float waterDemandFactor;     // Current multiplier (0.0-2.0)
    float currentSewageFlow;     // m³/s
    
    // Visual properties
    Color waterColor;
    
    // Historical usage patterns
    std::vector<float> hourlyUsage;     // Base hourly pattern (0-1)
    std::vector<float> sewageHourly;    // Sewage-specific pattern
    float weeklyAverage;
    std::array<float, 7> weeklyPattern; // Day-of-week pattern
    
    static float* citySimulationTime;
    
    // ================== CONSTRUCTOR ==================
    Building(int id, Vec3 pos, int floors, int cluster, int zoneID = -1)
        : id(id), clusterID(cluster), zoneID(zoneID), position(pos), numFloors(floors),
          height(floors * FLOOR_HEIGHT),
          waterSensorID(-1), sewageSensorID(-1),
          servicePipeID(-1), sewerPipeID(-1),
          dailyWaterUsage(0), currentWaterFlow(0), totalWaterConsumed(0),
          waterDemandFactor(1.0f), currentSewageFlow(0),
          weeklyAverage(0) {
        
        // ================== RANDOM BUILDING TYPE ==================
        int r = rand() % 100;
        if (r < 60) type = RESIDENTIAL;       // 60% residential
        else if (r < 75) type = COMMERCIAL;   // 15% commercial
        else if (r < 85) type = HOSTEL;       // 10% hostel
        else if (r < 95) type = HOSPITAL;     // 10% hospital
        else type = INDUSTRIAL;               // 5% industrial
        
        // ================== POPULATION DISTRIBUTION ==================
        switch (type) {
            case RESIDENTIAL:
                population = 20 + rand() % 80;               // 20–100
                sewageFactor = 0.7f + (rand() % 20) / 100.0f; // 0.7–0.9
                break;
                
            case COMMERCIAL:
                population = 50 + rand() % 150;              // 50–200
                sewageFactor = 0.5f + (rand() % 20) / 100.0f; // 0.5–0.7
                break;
                
            case HOSTEL:
                population = 100 + rand() % 300;             // 100–400
                sewageFactor = 0.9f + (rand() % 30) / 100.0f; // 0.9–1.2
                break;
                
            case HOSPITAL:
                population = 200 + rand() % 600;             // 200–800
                sewageFactor = 1.0f + (rand() % 30) / 100.0f; // 1.0–1.3
                break;
                
            case INDUSTRIAL:
                population = 30 + rand() % 100;              // 30–130
                sewageFactor = 1.1f + (rand() % 40) / 100.0f; // 1.1–1.5
                break;
        }
        
        // ================== PERSISTENT RANDOMNESS ==================
        randomDemandBias = 0.6f + (rand() % 80) / 100.0f;   // 0.6–1.4
        demandVolatility = 0.8f + (rand() % 40) / 100.0f;   // 0.8–1.2
        
        // ================== DAILY WATER USAGE CALCULATION ==================
        // Base water usage per person per day (liters)
        float litersPerPersonPerDay = 0.0f;
        switch (type) {
            case RESIDENTIAL:  litersPerPersonPerDay = 100.0f + rand() % 100; break; // 100-200 L
            case COMMERCIAL:   litersPerPersonPerDay = 50.0f + rand() % 50; break;   // 50-100 L
            case HOSTEL:       litersPerPersonPerDay = 120.0f + rand() % 80; break;  // 120-200 L
            case HOSPITAL:     litersPerPersonPerDay = 400.0f + rand() % 300; break; // 400-700 L
            case INDUSTRIAL:   litersPerPersonPerDay = 300.0f + rand() % 700; break; // 300-1000 L
        }
        
        // Convert to m³/day
        dailyWaterUsage = (litersPerPersonPerDay * population) / 1000.0f;
        
        // Apply random demand bias
        dailyWaterUsage *= randomDemandBias;
        
        // ================== VISUAL COLOR ==================
        waterColor = Color(
            (rand() % 100) / 200.0f + 0.5f,
            (rand() % 100) / 200.0f + 0.5f,
            (rand() % 100) / 200.0f + 0.8f,
            0.7f
        );
        
        // ================== HOURLY USAGE PATTERNS ==================
        hourlyUsage.resize(24, 0.0f);
        sewageHourly.resize(24, 0.0f);
        
        // Initialize hourly patterns based on building type
        initializeHourlyPatterns();
        
        // Weekly pattern (Monday=0, Sunday=6)
        for (int i = 0; i < 7; i++) {
            weeklyPattern[i] = 0.8f + (rand() % 40) / 100.0f; // 0.8-1.2
        }
    }
    
    // ================== HOURLY PATTERN INITIALIZATION ==================
    void initializeHourlyPatterns() {
        switch (type) {
            case RESIDENTIAL:
                // Residential: morning and evening peaks
                for (int i = 0; i < 24; i++) {
                    float morningPeak = exp(-pow(i - 7, 2) / 8.0f);  // 7 AM
                    float eveningPeak = exp(-pow(i - 19, 2) / 8.0f); // 7 PM
                    float dayBase = (i >= 6 && i <= 22) ? 0.3f : 0.1f;
                    hourlyUsage[i] = (morningPeak + eveningPeak + dayBase) / 2.5f;
                    
                    // Sewage peaks 30-60 minutes after water usage
                    sewageHourly[i] = 0.0f;
                    for (int offset = 1; offset <= 3; offset++) {
                        int hour = (i - offset + 24) % 24;
                        sewageHourly[i] += hourlyUsage[hour] * (0.4f - offset * 0.1f);
                    }
                    sewageHourly[i] /= 0.6f; // Normalize
                }
                break;
                
            case COMMERCIAL:
                // Commercial: daytime usage
                for (int i = 0; i < 24; i++) {
                    if (i >= 8 && i <= 18) {
                        float peak = exp(-pow(i - 12, 2) / 18.0f); // Noon peak
                        hourlyUsage[i] = 0.7f + 0.3f * peak;
                    } else {
                        hourlyUsage[i] = 0.1f;
                    }
                    sewageHourly[i] = hourlyUsage[i] * 0.7f; // Immediate sewage
                }
                break;
                
            case HOSTEL:
                // Hostel: constant high usage
                for (int i = 0; i < 24; i++) {
                    hourlyUsage[i] = 0.6f + 0.4f * exp(-pow(i - 12, 2) / 32.0f);
                    sewageHourly[i] = hourlyUsage[i] * 0.8f;
                }
                break;
                
            case HOSPITAL:
                // Hospital: 24/7 operation with slight night reduction
                for (int i = 0; i < 24; i++) {
                    if (i >= 6 && i <= 22) {
                        hourlyUsage[i] = 0.8f + 0.2f * sin(i * M_PI / 12.0f);
                    } else {
                        hourlyUsage[i] = 0.6f + 0.2f * sin(i * M_PI / 12.0f);
                    }
                    sewageHourly[i] = hourlyUsage[i] * 0.9f;
                }
                break;
                
            case INDUSTRIAL:
                // Industrial: daytime with lunch break
                for (int i = 0; i < 24; i++) {
                    if (i >= 7 && i <= 19 && !(i >= 12 && i <= 13)) {
                        float hourFactor = 1.0f - abs(i - 13) / 6.0f;
                        hourlyUsage[i] = 0.5f + 0.5f * hourFactor;
                    } else {
                        hourlyUsage[i] = 0.1f;
                    }
                    sewageHourly[i] = hourlyUsage[i]; // Industrial sewage ≈ water usage
                }
                break;
        }
        
        // Normalize both patterns
        normalizePattern(hourlyUsage);
        normalizePattern(sewageHourly);
    }
    
    // ================== NORMALIZE PATTERN ==================
    void normalizePattern(std::vector<float>& pattern) {
        float sum = 0.0f;
        for (float val : pattern) sum += val;
        if (sum > 0.0f) {
            for (float& val : pattern) val /= sum;
        }
    }
    
    // ================== UPDATE WATER USAGE ==================
    void updateWaterUsage(float dt, const DayNightCycle& dayNight) {
        if (!citySimulationTime) return;
        
        float timeOfDay = fmod(*citySimulationTime, 86400.0f) / 3600.0f; // Hours (0-24)
        int hour = static_cast<int>(timeOfDay) % 24;
        int minute = static_cast<int>((timeOfDay - hour) * 60.0f);
        
        // Get base demand from hourly pattern
        float baseDemand = hourlyUsage[hour];
        
        // Interpolate for smooth minute-by-minute transitions
        if (minute > 0) {
            int nextHour = (hour + 1) % 24;
            float nextDemand = hourlyUsage[nextHour];
            float minuteFactor = minute / 60.0f;
            baseDemand = baseDemand * (1.0f - minuteFactor) + nextDemand * minuteFactor;
        }
        
        // Apply day/night factor
        float dayNightFactor = dayNight.getWaterDemandFactor();
        
        // Apply day-of-week factor
        int dayOfWeek = static_cast<int>(timeOfDay / 24.0f) % 7; // Simple day calculation
        float weeklyFactor = weeklyPattern[dayOfWeek];
        
        // Weekend adjustment (more water usage on weekends)
        float weekendFactor = (dayOfWeek == 0 || dayOfWeek == 6) ? 1.2f : 1.0f;
        
        // Random minute-to-minute variation (±10% * volatility)
        float minuteRandom = 1.0f + ((rand() % 200) / 1000.0f - 0.1f) * demandVolatility;
        
        // Calculate final demand factor
        waterDemandFactor = baseDemand * dayNightFactor * weeklyFactor * weekendFactor * minuteRandom;
        
        // Calculate water flow rate (m³/s)
        float baseFlow = (dailyWaterUsage / 86400.0f); // Base flow in m³/s
        currentWaterFlow = baseFlow * waterDemandFactor;
        
        // Calculate sewage flow (with delay simulation)
        int sewageHour = (hour - 1 + 24) % 24; // Sewage is 1 hour behind water usage
        float sewageBase = sewageHourly[sewageHour];
        currentSewageFlow = currentWaterFlow * sewageFactor * sewageBase;
        
        // Update total consumption
        totalWaterConsumed += currentWaterFlow * dt;
        
        // Update weekly average (EMA)
        weeklyAverage = (weeklyAverage * 0.99f) + (currentWaterFlow * 0.01f);
    }
    
    // ================== GETTERS ==================
    float getDailyConsumption() const {
        return currentWaterFlow * 86400.0f; // m³/day
    }
    
    float getHourlyUsage(int hour) const {
        if (hour >= 0 && hour < 24) {
            return hourlyUsage[hour];
        }
        return 0.0f;
    }
    
    float getSewageHourly(int hour) const {
        if (hour >= 0 && hour < 24) {
            return sewageHourly[hour];
        }
        return 0.0f;
    }
    
    // ================== BUILDING TYPE STRING ==================
    std::string getTypeString() const {
        switch (type) {
            case RESIDENTIAL: return "RESIDENTIAL";
            case COMMERCIAL:  return "COMMERCIAL";
            case HOSTEL:      return "HOSTEL";
            case HOSPITAL:    return "HOSPITAL";
            case INDUSTRIAL:  return "INDUSTRIAL";
            default:          return "UNKNOWN";
        }
    }
    
    // ================== STATISTICS ==================
    float getWaterPerPerson() const {
        if (population > 0) {
            return (getDailyConsumption() * 1000.0f) / population; // Liters/person/day
        }
        return 0.0f;
    }
    
    // ================== DEBUG INFO ==================
    void printDebugInfo() const {
        printf("Building %d [%s]\n", id, getTypeString().c_str());
        printf("  Population: %d people\n", population);
        printf("  Daily Water: %.2f m³/day (%.0f L/person)\n", 
               getDailyConsumption(), getWaterPerPerson());
        printf("  Current Flow: %.4f L/s\n", currentWaterFlow * 1000.0f);
        printf("  Sewage Factor: %.2f (Flow: %.4f L/s)\n", 
               sewageFactor, currentSewageFlow * 1000.0f);
        printf("  Random Bias: %.2f, Volatility: %.2f\n", 
               randomDemandBias, demandVolatility);
    }
};

// Static member initialization
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
    
    int waterSensorID;
    int sewageSensorID;
    int secondaryMainID;
    int sewageCollectorID;
    
    Cluster(int id, Vec3 center)
        : id(id), centerPos(center), waterSensorID(-1), sewageSensorID(-1),
          secondaryMainID(-1), sewageCollectorID(-1) {}
};

// ============================================================================
// RESERVOIR WITH PUMP SYSTEM
// ============================================================================

struct Reservoir {
    Vec3 position;
    float capacity;          // m³
    float volume;           // m³
    float level;            // meters
    float maxPumpCapacity;  // m³/s
    float availablePumpCapacity; // m³/s
    float inflowRate;       // m³/s
    float outflowRate;      // m³/s
    float levelTrend;       // Rising/falling rate
    float supplyMargin;     // Available capacity margin
    
    std::vector<Pipe> pumps;
    
    Reservoir() : position(0, 0, 0), capacity(10000.0f), volume(7000.0f),
                 level(0), maxPumpCapacity(2.0f), availablePumpCapacity(2.0f),
                 inflowRate(0.5f), outflowRate(0), levelTrend(0),
                 supplyMargin(1.5f) {
        level = (volume / capacity) * 30.0f;
    }
    
    void update(float dt, float totalDemand) {
        // Calculate outflow based on demand and pump capacity
        outflowRate = std::min(totalDemand, availablePumpCapacity);
        
        // Update volume
        float netFlow = inflowRate - outflowRate;
        volume += netFlow * dt;
        
        // Clamp volume
        if (volume < 0) volume = 0;
        if (volume > capacity) volume = capacity;
        
        // Update level
        level = (volume / capacity) * 30.0f;
        
        // Calculate trend (smoothed derivative)
        levelTrend = (levelTrend * 0.9f) + (netFlow / capacity * 10.0f * 0.1f);
        
        // Update supply margin
        supplyMargin = maxPumpCapacity - outflowRate;
        
        // Update available pump capacity (can vary due to maintenance, etc.)
        availablePumpCapacity = maxPumpCapacity * (0.8f + 0.2f * sinf(dayNight.timeOfDay * M_PI / 12.0f));
    }
    
    float getLevelPercent() const {
        return (volume / capacity) * 100.0f;
    }
    
    std::string getTrendString() const {
        if (levelTrend > 0.01f) return "rising";
        else if (levelTrend < -0.01f) return "falling";
        else return "stable";
    }
};

// ============================================================================
// RL STATE STRUCTURE FOR UDP BROADCAST
// ============================================================================

struct RLState {
    // Reservoir state
    float reservoir_level_pct;
    std::string reservoir_trend;
    float pump_capacity_available;
    float supply_margin;
    
    // Zone states (for up to 10 zones)
    float avg_pressure_zone[10];
    float min_pressure_zone[10];
    float flow_zone[10];
    float pressure_variance_zone[10];
    
    // Demand patterns
    float flow_to_pressure_ratio[10];
    float historical_flow[10];
    float time_of_day;
    std::string day_type;
    
    // Anomalies
    bool leak_flag[10];
    float leak_severity[10];
    bool overflow_flag[10];
    bool pressure_violation[10];
    
    // Control state
    float valve_state[50]; // Up to 50 valves
    float last_action_time[50];
    int recent_action_count;
    
    // Time encoding
    float sim_time_sin;
    float sim_time_cos;
    float episode_progress;

    float cumulative_volume_sensor[50] = {0};
    float hourly_usage[24] = {0};
    float non_revenue_water = 0.0f; // Total leak volume
    float supply_efficiency = 0.0f; // Volume delivered / volume pumped
    
    // Flow metrics
    float peak_flow_rate = 0.0f;
    float avg_flow_rate = 0.0f;
    float flow_variance = 0.0f;
    
    // Quality metrics
    float pressure_compliance = 0.0f; // % time in pressure range
    float service_continuity = 0.0f;  // % time with flow
    float response_time = 0.0f;       // Avg time to fix issues
    
    RLState() {
        // Initialize all members manually - NO memset on non-POD types!
        reservoir_level_pct = 0.0f;
        reservoir_trend = "";
        pump_capacity_available = 0.0f;
        supply_margin = 0.0f;
        time_of_day = 0.0f;
        day_type = "";
        recent_action_count = 0;
        sim_time_sin = 0.0f;
        sim_time_cos = 0.0f;
        episode_progress = 0.0f;
        
        // Initialize arrays
        for (int i = 0; i < 10; i++) {
            avg_pressure_zone[i] = 0.0f;
            min_pressure_zone[i] = 0.0f;
            flow_zone[i] = 0.0f;
            pressure_variance_zone[i] = 0.0f;
            flow_to_pressure_ratio[i] = 0.0f;
            historical_flow[i] = 0.0f;
            leak_flag[i] = false;
            leak_severity[i] = 0.0f;
            overflow_flag[i] = false;
            pressure_violation[i] = false;
        }
        
        for (int i = 0; i < 50; i++) {
            valve_state[i] = 0.0f;
            last_action_time[i] = 0.0f;
        }
    }
    
    std::string toJSON() const {
    Json::Value root;
    
    // Reservoir
    root["reservoir_level_pct"] = reservoir_level_pct;
    root["reservoir_trend"] = reservoir_trend;
    root["pump_capacity_available"] = pump_capacity_available;
    root["supply_margin"] = supply_margin;
    
    // Zones
    Json::Value zones(Json::arrayValue);
    for (int i = 0; i < 10; i++) {
        Json::Value zone;
        zone["avg_pressure"] = avg_pressure_zone[i];
        zone["min_pressure"] = min_pressure_zone[i];
        zone["flow"] = flow_zone[i];
        zone["pressure_variance"] = pressure_variance_zone[i];
        zone["flow_to_pressure_ratio"] = flow_to_pressure_ratio[i];
        zone["historical_flow"] = historical_flow[i];
        zone["leak_flag"] = leak_flag[i];
        zone["leak_severity"] = leak_severity[i];
        zone["overflow_flag"] = overflow_flag[i];
        zone["pressure_violation"] = pressure_violation[i];
        zones.append(zone);
    }
    root["zones"] = zones;
    
    // System state
    root["time_of_day"] = time_of_day;
    root["day_type"] = day_type;
    root["recent_action_count"] = recent_action_count;
    root["sim_time_sin"] = sim_time_sin;
    root["sim_time_cos"] = sim_time_cos;
    root["episode_progress"] = episode_progress;
    
    // Valves
    Json::Value valves(Json::arrayValue);
    for (int i = 0; i < 50; i++) {
        valves.append(valve_state[i]);
    }
    root["valve_states"] = valves;
    
    // Flow metrics
    root["peak_flow_rate"] = peak_flow_rate;
    root["avg_flow_rate"] = avg_flow_rate;
    root["flow_variance"] = flow_variance;
    
    // Water efficiency metrics
    root["non_revenue_water"] = non_revenue_water;
    root["supply_efficiency"] = supply_efficiency;
    
    // Quality metrics
    root["pressure_compliance"] = pressure_compliance;
    root["service_continuity"] = service_continuity;
    root["response_time"] = response_time;
    
    // Cumulative volumes
    Json::Value volumes(Json::arrayValue);
    for (int i = 0; i < 50; i++) {
        volumes.append(cumulative_volume_sensor[i]);
    }
    root["cumulative_volumes"] = volumes;
    
    // Hourly usage
    Json::Value hourly(Json::arrayValue);
    for (int i = 0; i < 24; i++) {
        hourly.append(hourly_usage[i]);
    }
    root["hourly_usage"] = hourly;
    
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, root);
    }
};

// ============================================================================
// UDP BROADCAST FUNCTIONS
// ============================================================================

bool initUDPBroadcast() {
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        std::cerr << "Failed to create UDP socket" << std::endl;
        return false;
    }
    
    // Enable broadcast
    int broadcastEnable = 1;
    if (setsockopt(udpSocket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        std::cerr << "Failed to enable broadcast" << std::endl;
        close(udpSocket);
        return false;
    }
    
    // Set non-blocking
    int flags = fcntl(udpSocket, F_GETFL, 0);
    fcntl(udpSocket, F_SETFL, flags | O_NONBLOCK);
    
    // Setup broadcast address
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(UDP_BROADCAST_PORT);
    broadcastAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    
    std::cout << "UDP broadcast initialized on port " << UDP_BROADCAST_PORT << std::endl;
    return true;
}

void udpBroadcastWorker() {
    while (udpRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
        std::lock_guard<std::mutex> lock(udpMutex);
        while (!udpMessageQueue.empty()) {
            std::string message = udpMessageQueue.front();
            udpMessageQueue.pop();
            
            sendto(udpSocket, message.c_str(), message.length(), 0,
                  (struct sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
        }
    }
}

void startUDPBroadcast() {
    if (initUDPBroadcast()) {
        udpRunning = true;
        udpThread = new std::thread(udpBroadcastWorker);
        std::cout << "UDP broadcast started at 10Hz" << std::endl;
    }
}

void stopUDPBroadcast() {
    udpRunning = false;
    
    // Stop thread FIRST
    if (udpThread && udpThread->joinable()) {
        udpThread->join();
        delete udpThread;
        udpThread = nullptr;
    }
    
    // THEN close socket
    if (udpSocket >= 0) {
        close(udpSocket);
        udpSocket = -1;
    }
}

void queueUDPBroadcast(const std::string& message) {
    std::lock_guard<std::mutex> lock(udpMutex);
    udpMessageQueue.push(message);
}

// ============================================================================
// ROS2 SENSOR SUBSCRIBER CLASS - IMPROVED VERSION
// ============================================================================

class ROS2SensorSubscriber : public rclcpp::Node {
private:
    CityNetwork* city_network_;  // Use raw pointer since CityNetwork is a global
    std::atomic<bool> running_;
    
    // Topic subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esp1_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esp2_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esp3_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esp4_sub_;
    
    // Statistics
    std::atomic<int> total_updates_;
    
public:
    ROS2SensorSubscriber(CityNetwork* city) 
    : Node("water_sim_subscriber"), 
      city_network_(city),
      running_(true),
      total_updates_(0) {
    
    RCLCPP_INFO(this->get_logger(), "Initializing ROS2 Sensor Subscriber");
    
    // Subscribe to all ESP topics with QoS settings
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Add debug logging for each subscription
    esp1_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/esp1/sensors", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received message from ESP1, size: %lu bytes", 
                       msg->data.size());
            this->processESPData(msg->data, 1);
        });
        
    esp2_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/esp2/sensors", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received message from ESP2, size: %lu bytes", 
                       msg->data.size());
            this->processESPData(msg->data, 2);
        });
        
    esp3_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/esp3/sensors", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received message from ESP3, size: %lu bytes", 
                       msg->data.size());
            this->processESPData(msg->data, 3);
        });
        
    esp4_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/esp4/sensors", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received message from ESP4, size: %lu bytes", 
                       msg->data.size());
            this->processESPData(msg->data, 4);
        });
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to ESP sensor topics");
    RCLCPP_INFO(this->get_logger(), "Topics: /esp1/sensors, /esp2/sensors, /esp3/sensors, /esp4/sensors");
}
    
    ~ROS2SensorSubscriber() {
        shutdown();
    }
    
    void shutdown() {
        running_ = false;
        
        // Reset subscriptions to prevent further callbacks
        esp1_sub_.reset();
        esp2_sub_.reset();
        esp3_sub_.reset();
        esp4_sub_.reset();
        
        RCLCPP_INFO(this->get_logger(), "ROS2 Sensor Subscriber shutdown");
    }
    
    void processESPData(const std::string& json_data, int esp_id);
};

// ============================================================================
// CITY NETWORK (MAIN CLASS)
// ============================================================================

class CityNetwork {
public:
    std::vector<Building> buildings;
    std::vector<Cluster> clusters;
    std::vector<Pipe> pipes;
    std::vector<Sensor> sensors;
    std::vector<Zone> zones;
    std::vector<WaterParticle> reservoirParticles;
    
    Reservoir reservoir;
    Vec3 stpPos;
    float cityExtent;
    
    int reservoirWaterSensorID;
    int stpSewerSensorID;
    
    float simulationTime;
    float totalWaterSupplied;
    float totalWaterConsumed;
    float totalLeakWater;
    
    // RL state
    RLState rlState;
    int episodeStep;
    
    // ROS2 subscriber
    std::shared_ptr<ROS2SensorSubscriber> ros2_subscriber;
    
    CityNetwork() : cityExtent(100), reservoirWaterSensorID(-1), stpSewerSensorID(-1), 
                   simulationTime(8 * 3600), // Start at 8 AM
                   totalWaterSupplied(0), totalWaterConsumed(0),
                   totalLeakWater(0), episodeStep(0) {
        Building::citySimulationTime = &simulationTime;
    }
    
    void generateCity(int numBuildings);
    void updateDynamicPositions();
    void generateWaterNetwork();
    void generateSewageNetwork();
    void createSensors();
    void createZones();
    void loadLeakData(const char* filename);
    void updateSensorFromROS2(int sensorID, float valve, float pressure, float level, int esp_id);
    void updateSimulation(float dt);
    void generateInitSensorJSON(const char* filename);
    void exportSensorJSON(const char* filename);
    void exportLeakJSON(const char* filename);
    void exportSystemStateJSON(const char* filename);
    void broadcastRLState();
    
    void updateBuildingWaterUsage(float dt);
    void updateReservoir(float dt);
    void updateZones(float dt);
    void generateReservoirParticles(float dt);
    void drawReservoirWithParticles() const;
    
    void initializeROS2();
    void shutdownROS2();
    void printSensorMapping() const;
    
    // Sensor statistics
    int getActiveSensors() const;
    int getStaleSensors() const;
    
    // Zone management
    Zone* getZoneForBuilding(int buildingID);
    Zone* getZoneForPipe(int pipeID);
    
    // Valve control
    void setValveState(int sensorID, float valveState);
    
    void drawReservoirStructure(Vec3 pos, float waterLevel) const;
    void updateRLState();
    float calculateTotalDemand() const;
    void updatePipePhysics(float dt);
    
private: 
    std::unordered_map<int, int> sensorIdToIndex;
};

// ============================================================================
// CITY NETWORK IMPLEMENTATION
// ============================================================================

void CityNetwork::initializeROS2() {
    if (ros2_initialized) {
        return;
    }
    
    std::cout << "Initializing ROS2 Humble..." << std::endl;
    
    try {
        // Initialize ROS2 if not already
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
        
        // Create ROS2 node for the subscriber - pass raw pointer
        ros2_subscriber = std::make_shared<ROS2SensorSubscriber>(this);
        
        // Create executor
        ros2_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        
        // Add subscriber node to executor
        if (ros2_subscriber) {
            ros2_executor->add_node(ros2_subscriber);
        }
        
        // Start ROS2 spinner in separate thread
        ros2_thread = std::make_shared<std::thread>([this]() {
            RCLCPP_INFO(rclcpp::get_logger("ros2_spinner"), "Starting ROS2 spinner thread");
            ros2_running = true;
            
            while (ros2_running && rclcpp::ok()) {
                ros2_executor->spin_some(std::chrono::milliseconds(10));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            
            RCLCPP_INFO(rclcpp::get_logger("ros2_spinner"), "ROS2 spinner thread stopped");
        });
        
        ros2_initialized = true;
        std::cout << "ROS2 Humble initialized successfully!" << std::endl;
        std::cout << "Listening to topics:" << std::endl;
        std::cout << "  /esp1/sensors" << std::endl;
        std::cout << "  /esp2/sensors" << std::endl;
        std::cout << "  /esp3/sensors" << std::endl;
        std::cout << "  /esp4/sensors" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize ROS2: " << e.what() << std::endl;
        ros2_initialized = false;
    }
}

void CityNetwork::shutdownROS2() {
    if (!ros2_initialized) {
        return;
    }
    
    std::cout << "Shutting down ROS2..." << std::endl;
    
    // First, stop the ROS2 subscriber from processing new messages
    if (ros2_subscriber) {
        ros2_subscriber->shutdown();
    }
    
    // Stop the executor thread FIRST
    ros2_running = false;
    
    // Give time for the executor to stop
    if (ros2_thread && ros2_thread->joinable()) {
        // Try to join with timeout
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
            if (ros2_thread->joinable()) {
                try {
                    ros2_thread->join();
                    break;
                } catch (const std::exception& e) {
                    std::cerr << "Thread join error: " << e.what() << std::endl;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ros2_thread.reset();
    }
    
    //  Remove node from executor BEFORE destroying anything
    if (ros2_executor && ros2_subscriber) {
        try {
            ros2_executor->remove_node(ros2_subscriber);
        } catch (const std::exception& e) {
            std::cerr << "Error removing node from executor: " << e.what() << std::endl;
        }
    }
    
    // Reset subscriptions in the subscriber BEFORE destroying it
    if (ros2_subscriber) {
        ros2_subscriber.reset();  // Let destructor handle cleanup
    }
    
    // Reset executor
    ros2_executor.reset();
    
    // Shutdown ROS2 context
    if (rclcpp::ok()) {
        try {
            rclcpp::shutdown();
        } catch (const std::exception& e) {
            std::cerr << "Error during rclcpp::shutdown(): " << e.what() << std::endl;
        }
    }
    
    // Give DDS time to clean up
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    ros2_initialized = false;
    std::cout << "ROS2 shutdown complete." << std::endl;
}

void CityNetwork::updateSensorFromROS2(int sensorID, float valve, float pressure, float level, int esp_id) {
    // Only lock here, not in Sensor::updateFromROS2()
    std::lock_guard<std::mutex> lock(sensor_mutex);
    
    // Use the mapping instead of direct indexing
    auto it = sensorIdToIndex.find(sensorID);
    if (it != sensorIdToIndex.end()) {
        int idx = it->second;
        if (idx >= 0 && idx < (int)sensors.size()) {
            // Modified Sensor::updateFromROS2() - removed internal locking
            sensors[idx].valveState = valve;
            sensors[idx].pressure = pressure;
            sensors[idx].waterLevel = level;
            sensors[idx].last_esp_id = esp_id;
            sensors[idx].last_update_time = time(nullptr);
            sensors[idx].pressureFromROS = true;
            
            static int first_updates_logged = 0;
            if (first_updates_logged < 20) {  // Increased to see more updates
                std::cout << "✓ ROS2 Update: Sensor ID=" << sensorID 
                          << " (index=" << idx << ") name=" << sensors[idx].name
                          << " from ESP" << esp_id 
                          << " - Valve:" << valve << "%"
                          << " Pressure:" << pressure << "kPa"
                          << " Level:" << level << "%" << std::endl;
                first_updates_logged++;
            }
        } else {
            static int invalid_index_warnings = 0;
            if (invalid_index_warnings < 10) {
                std::cerr << "ERROR: Mapped index out of range: sensorID=" 
                          << sensorID << " -> idx=" << idx 
                          << " but sensors.size()=" << sensors.size() << std::endl;
                invalid_index_warnings++;
            }
        }
    } else {
        static int unknown_sensor_warnings = 0;
        if (unknown_sensor_warnings < 20) {
            std::cerr << "⚠ WARNING: Received update for unknown sensor ID: " << sensorID 
                      << " from ESP" << esp_id 
                      << " (Known IDs: " << sensorIdToIndex.size() << " sensors)" << std::endl;
            
            // Print more details
            if (unknown_sensor_warnings == 0) {
                std::cout << "Looking for sensor ID " << sensorID << std::endl;
                std::cout << "Sensor ID range in simulation: ";
                if (!sensorIdToIndex.empty()) {
                    auto minmax = std::minmax_element(sensorIdToIndex.begin(), sensorIdToIndex.end());
                    std::cout << minmax.first->first << " to " << minmax.second->first << std::endl;
                }
                std::cout << "Total sensors created: " << sensors.size() << std::endl;
            }
            unknown_sensor_warnings++;
        }
    }
}

int CityNetwork::getActiveSensors() const {
    int count = 0;
    for (const auto& sensor : sensors) {
        if (!sensor.isStale()) {
            count++;
        }
    }
    return count;
}

int CityNetwork::getStaleSensors() const {
    int count = 0;
    for (const auto& sensor : sensors) {
        if (sensor.isStale()) {
            count++;
        }
    }
    return count;
}

void CityNetwork::setValveState(int sensorID, float valveState) {
    std::lock_guard<std::mutex> lock(sensor_mutex);
    auto it = sensorIdToIndex.find(sensorID);
    if (it != sensorIdToIndex.end()) {
        int idx = it->second;
        if (idx >= 0 && idx < (int)sensors.size()) {
            sensors[idx].setValve(valveState);
        }
    }
}

void CityNetwork::printSensorMapping() const {
    std::cout << "\n=== SENSOR ID MAPPING ===\n";
    std::cout << "Total sensors: " << sensors.size() << "\n";
    std::cout << "Mapped IDs: " << sensorIdToIndex.size() << "\n\n";
    
    std::cout << "ID -> Index mapping:\n";
    for (const auto& pair : sensorIdToIndex) {
        int id = pair.first;
        int idx = pair.second;
        if (idx < sensors.size()) {
            std::cout << "  Sensor ID " << id << " -> index " << idx 
                      << " (" << sensors[idx].name << ")" << std::endl;
        }
    }
}

// ROS2SensorSubscriber::processESPData IMPLEMENTATION
// (Must be AFTER CityNetwork is fully defined)
void ROS2SensorSubscriber::processESPData(const std::string& json_data, int esp_id) {
    if (!running_) {
        return;
    }
    
    if (json_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty JSON data from ESP%d", esp_id);
        return;
    }
    
    // Log first 200 characters of the JSON to see format
    std::string preview = json_data.substr(0, std::min(200, (int)json_data.size()));
    RCLCPP_INFO(this->get_logger(), "ESP%d JSON preview: %s...", esp_id, preview.c_str());
    
    try {
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(json_data, root);
        
        if (!parsingSuccessful) {
            RCLCPP_WARN(this->get_logger(),
                       "Failed to parse JSON from ESP%d: %s", 
                       esp_id, reader.getFormattedErrorMessages().c_str());
            return;
        }

        // Check for esp_id field in JSON
        if (root.isMember("esp_id")) {
            int json_esp_id = root["esp_id"].asInt();
            RCLCPP_INFO(this->get_logger(), "ESP%d message contains esp_id: %d", esp_id, json_esp_id);
        }

        if (!root.isMember("sensors") || !root["sensors"].isArray()) {
            RCLCPP_WARN(this->get_logger(),
                       "Invalid JSON format from ESP%d: no sensors array", esp_id);
            return;
        }

        const Json::Value& sensors = root["sensors"];
        int sensors_count = sensors.size();
        int sensors_updated = 0;
        
        RCLCPP_INFO(this->get_logger(), "ESP%d: Processing %d sensors", esp_id, sensors_count);

        for (const Json::Value& sensor_data : sensors) {
            if (!sensor_data.isMember("sensor_id") ||
                !sensor_data.isMember("pressure_bar") ||
                !sensor_data.isMember("level_pct") ||
                !sensor_data.isMember("valve")) {
                RCLCPP_WARN(this->get_logger(),
                           "Incomplete sensor data from ESP%d", esp_id);
                continue;
            }

            try {
                int sensor_id = sensor_data["sensor_id"].asInt();
                float pressure_kpa = sensor_data["pressure_bar"].asFloat() * 100.0f;
                float level_pct = sensor_data["level_pct"].asFloat();
                float valve_pct = sensor_data["valve"].asInt() ? 100.0f : 0.0f;

                // Check if city_network still exists
                if (!city_network_) {
                    RCLCPP_WARN(this->get_logger(),
                               "CityNetwork pointer is null, ignoring ESP%d data", esp_id);
                    return;
                }

                // Only log first few updates to avoid spam
                if (sensors_updated < 5) {
                    RCLCPP_INFO(this->get_logger(), 
                               "ESP%d updating sensor %d: valve=%.1f%%, pressure=%.1fkPa, level=%.1f%%",
                               esp_id, sensor_id, valve_pct, pressure_kpa, level_pct);
                }

                city_network_->updateSensorFromROS2(
                    sensor_id, valve_pct, pressure_kpa, level_pct, esp_id);

                sensors_updated++;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                            "Error processing individual sensor from ESP%d: %s",
                            esp_id, e.what());
            }
        }

        total_updates_++;
        RCLCPP_INFO(this->get_logger(), 
                   "ESP%d: Successfully updated %d/%d sensors (total updates: %d)",
                   esp_id, sensors_updated, sensors_count, total_updates_.load());

    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                    "ESP%d processing error: %s",
                    esp_id, e.what());
    }
    catch (...) {
        RCLCPP_ERROR(this->get_logger(),
                    "ESP%d processing error: Unknown exception", esp_id);
    }
}

// REST OF THE CITY NETWORK IMPLEMENTATION (UNCHANGED FROM FIRST CODE)
// ... [All the other CityNetwork methods remain exactly as in the first code] ...

void CityNetwork::updateDynamicPositions() {
    float maxDist = 0;
    for (const auto& cluster : clusters) {
        float dist = sqrt(cluster.centerPos.x * cluster.centerPos.x + 
                         cluster.centerPos.z * cluster.centerPos.z);
        maxDist = std::max(maxDist, dist);
    }
    cityExtent = maxDist + CLUSTER_SPACING;
    
    reservoir.position = Vec3(-cityExtent - 80, 20, -cityExtent - 80);
    stpPos = Vec3(cityExtent + 80, 0, cityExtent + 80);
}

void CityNetwork::generateInitSensorJSON(const char* filename) {
    if (std::remove(filename) == 0) {
        std::cout << "Old init file deleted: " << filename << "\n";
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot create " << filename << "\n";
        return;
    }

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

    for (size_t i = 0; i < sensors.size(); ++i) {
        const Sensor& s = sensors[i];
        bool isFresh = (s.type == WATER_SENSOR);

        file << "    {\n";
        file << "      \"sensor_id\": " << s.id << ",\n";
        file << "      \"name\": \"" << s.name << "\",\n";
        file << "      \"pipe_id\": " << s.connectedPipeID << ",\n";
        file << "      \"pipe_type\": " << (isFresh ? 0 : 1) << ",\n";
        file << "      \"pressure_bar\": " << (isFresh ? 1.03f : 1.01f) << ",\n";
        file << "      \"level_pct\": " << (isFresh ? 90.0f : 70.0f) << ",\n";
        file << "      \"valve\": 1\n";
        file << "    }";
        if (i < sensors.size() - 1) file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";
    file.close();

    std::cout << "Init sensor snapshot created: " << filename << "\n";
    std::cout << "Contains " << sensors.size() << " sensors with their IDs\n";
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
        file << "      \"name\": \"" << s.name << "\",\n";
        file << "      \"pipe_id\": " << s.connectedPipeID << ",\n";
        file << "      \"type\": " << (isFresh ? "\"water\"" : "\"sewage\"") << ",\n";
        file << "      \"pressure_bar\": " << std::setprecision(3) << (s.pressure / 100.0f) << ",\n";
        file << "      \"level_pct\": " << std::setprecision(1) << s.waterLevel << ",\n";
        file << "      \"valve\": " << (s.valveState > 50.0f ? 1 : 0) << ",\n";
        file << "      \"last_esp_id\": " << s.last_esp_id << ",\n";
        file << "      \"is_stale\": " << (s.isStale() ? "true" : "false") << "\n";
        file << "    }";
        if (i < sensors.size() - 1) file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";
    file.close();
}

void CityNetwork::exportLeakJSON(const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot write " << filename << "\n";
        return;
    }

    file << "{\n";
    file << "  \"timestamp\": " << time(nullptr) << ",\n";
    file << "  \"sim_time\": " << std::fixed << std::setprecision(3) << simulationTime << ",\n";
    file << "  \"total_leaks\": " << 0 << ",\n";
    file << "  \"leaks\": [\n";

    int leakCount = 0;
    bool first = true;
    
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            if (!first) file << ",\n";
            first = false;
            leakCount++;

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

    file << "  \"global_state\": {\n";
    file << "    \"reservoir_level_m\": " << std::setprecision(2) << reservoir.level << ",\n";
    file << "    \"reservoir_volume_m3\": " << std::setprecision(2) << reservoir.volume << ",\n";
    file << "    \"reservoir_level_pct\": " << std::setprecision(1) << reservoir.getLevelPercent() << ",\n";
    file << "    \"reservoir_trend\": \"" << reservoir.getTrendString() << "\",\n";
    file << "    \"pump_capacity_available\": " << std::setprecision(2) << reservoir.availablePumpCapacity << ",\n";
    file << "    \"supply_margin\": " << std::setprecision(2) << reservoir.supplyMargin << ",\n";

    float total_fresh_flow = calculateTotalDemand();
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
         << std::setprecision(4) << total_fresh_flow << ",\n";
    file << "    \"total_sewage_flow_m3s\": "
         << total_sewage_flow << ",\n";
    file << "    \"total_leak_flow_m3s\": "
         << total_leak_flow << ",\n";
    file << "    \"active_leak_count\": " << numLeaks << ",\n";

    file << "    \"leaking_pipe_ids\": [";
    for (size_t i = 0; i < leakingPipeIDs.size(); ++i) {
        file << leakingPipeIDs[i];
        if (i < leakingPipeIDs.size() - 1) file << ", ";
    }
    file << "],\n";

    // Zone statistics
    file << "    \"zones\": [\n";
    for (size_t i = 0; i < zones.size(); ++i) {
        const auto& zone = zones[i];
        file << "      {\n";
        file << "        \"zone_id\": " << zone.id << ",\n";
        file << "        \"avg_pressure\": " << std::setprecision(1) << zone.avgPressure << ",\n";
        file << "        \"min_pressure\": " << zone.minPressure << ",\n";
        file << "        \"flow\": " << std::setprecision(3) << zone.totalFlow << ",\n";
        file << "        \"pressure_variance\": " << std::setprecision(2) << zone.pressureVariance << ",\n";
        file << "        \"flow_to_pressure_ratio\": " << std::setprecision(4) << zone.flowToPressureRatio << ",\n";
        file << "        \"leak_flag\": " << (zone.leakFlag ? "true" : "false") << ",\n";
        file << "        \"leak_severity\": " << std::setprecision(2) << zone.leakSeverity << ",\n";
        file << "        \"overflow_flag\": " << (zone.overflowFlag ? "true" : "false") << ",\n";
        file << "        \"pressure_violation\": " << (zone.pressureViolation ? "true" : "false") << "\n";
        file << "      }";
        if (i < zones.size() - 1) file << ",";
        file << "\n";
    }
    file << "    ],\n";

    // Sensor statistics
    int active_sensors = getActiveSensors();
    int stale_sensors = getStaleSensors();
    
    file << "    \"ros2_sensor_stats\": {\n";
    file << "      \"total_sensors\": " << sensors.size() << ",\n";
    file << "      \"active_sensors\": " << active_sensors << ",\n";
    file << "      \"stale_sensors\": " << stale_sensors << ",\n";
    file << "      \"update_rate\": " << std::setprecision(1) 
         << (active_sensors > 0 ? 100.0f * active_sensors / sensors.size() : 0.0f) << "\n";
    file << "    },\n";

    // Time information
    file << "    \"time_of_day\": " << std::setprecision(2) << dayNight.timeOfDay << ",\n";
    file << "    \"day_type\": \"" << dayNight.getDayType() << "\",\n";
    file << "    \"water_demand_factor\": " << std::setprecision(3) << dayNight.getWaterDemandFactor() << "\n";

    file << "  },\n";
    
    file << "  \"building_water_usage\": {\n";
    file << "    \"total_buildings\": " << buildings.size() << ",\n";
    file << "    \"total_consumption_m3\": " << std::setprecision(2) << totalWaterConsumed << ",\n";
    file << "    \"current_demand_m3s\": " << std::setprecision(4) << total_fresh_flow << ",\n";
    file << "    \"buildings\": [\n";
    
    for (size_t i = 0; i < buildings.size(); ++i) {
        const auto& b = buildings[i];
        file << "      {\n";
        file << "        \"id\": " << b.id << ",\n";
        file << "        \"cluster\": " << b.clusterID << ",\n";
        file << "        \"zone\": " << b.zoneID << ",\n";
        file << "        \"floors\": " << b.numFloors << ",\n";
        file << "        \"population\": " << b.population << ",\n";
        file << "        \"current_flow_m3s\": " << std::setprecision(6) << b.currentWaterFlow << ",\n";
        file << "        \"daily_consumption_m3\": " << std::setprecision(2) << b.getDailyConsumption() << ",\n";
        file << "        \"total_consumed_m3\": " << std::setprecision(2) << b.totalWaterConsumed << ",\n";
        file << "        \"demand_factor\": " << std::setprecision(3) << b.waterDemandFactor << "\n";
        file << "      }";
        if (i < buildings.size() - 1) file << ",";
        file << "\n";
    }
    
    file << "    ]\n";
    file << "  }\n";
    
    file << "}\n";
    file.close();
}

void CityNetwork::broadcastRLState() {
    updateRLState();
    std::string json = rlState.toJSON();
    queueUDPBroadcast(json);
}

void CityNetwork::updateRLState() {
    // Reservoir state
    rlState.reservoir_level_pct = reservoir.getLevelPercent();
    rlState.reservoir_trend = reservoir.getTrendString();
    rlState.pump_capacity_available = reservoir.availablePumpCapacity;
    rlState.supply_margin = reservoir.supplyMargin;
    
    // Zone states
    for (size_t i = 0; i < zones.size() && i < 10; i++) {
        rlState.avg_pressure_zone[i] = zones[i].avgPressure;
        rlState.min_pressure_zone[i] = zones[i].minPressure;
        rlState.flow_zone[i] = zones[i].totalFlow;
        rlState.pressure_variance_zone[i] = zones[i].pressureVariance;
        rlState.flow_to_pressure_ratio[i] = zones[i].flowToPressureRatio;
        
        // Historical flow (average of last 24 hours)
        if (!zones[i].historicalFlow.empty()) {
            float sum = 0;
            for (float flow : zones[i].historicalFlow) {
                sum += flow;
            }
            rlState.historical_flow[i] = sum / zones[i].historicalFlow.size();
        }
        
        rlState.leak_flag[i] = zones[i].leakFlag;
        rlState.leak_severity[i] = zones[i].leakSeverity;
        rlState.overflow_flag[i] = zones[i].overflowFlag;
        rlState.pressure_violation[i] = zones[i].pressureViolation;
    }
    
    // Time information
    rlState.time_of_day = dayNight.timeOfDay;
    rlState.day_type = dayNight.getDayType();
    
    // Valve states
    for (size_t i = 0; i < sensors.size() && i < 50; i++) {
        rlState.valve_state[i] = sensors[i].valveState;
        rlState.last_action_time[i] = static_cast<float>(sensors[i].lastActionTime);
    }
    
    // Count recent actions
    rlState.recent_action_count = 0;
    for (const auto& sensor : sensors) {
        if (sensor.recentActionCount > 0) {
            rlState.recent_action_count++;
        }
    }
    
    // Time encoding for cyclic patterns
    float timeRadians = (dayNight.timeOfDay / 24.0f) * 2.0f * M_PI;
    rlState.sim_time_sin = sinf(timeRadians);
    rlState.sim_time_cos = cosf(timeRadians);
    
    // Episode progress (simulation time normalized)
    rlState.episode_progress = fmod(simulationTime, 86400.0f) / 86400.0f;
    
    // Calculate flow metrics
    float totalFlowSum = 0.0f;
    float flowSquaredSum = 0.0f;
    int flowCount = 0;
    
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) { // Water pipes only
            totalFlowSum += pipe.flowRate;
            flowSquaredSum += pipe.flowRate * pipe.flowRate;
            flowCount++;
            
            // Update peak flow rate
            if (pipe.flowRate > rlState.peak_flow_rate) {
                rlState.peak_flow_rate = pipe.flowRate;
            }
        }
    }
    
    if (flowCount > 0) {
        rlState.avg_flow_rate = totalFlowSum / flowCount;
        float variance = (flowSquaredSum / flowCount) - (rlState.avg_flow_rate * rlState.avg_flow_rate);
        rlState.flow_variance = std::max(0.0f, variance);
    }
    
    // Calculate Non-Revenue Water (NRW) and supply efficiency
    float totalDemand = calculateTotalDemand();
    float totalWaterPumped = reservoir.outflowRate;
    
    float totalLeakFlow = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeakFlow += pipe.leakRate / 1000.0f; // Convert L/s to m³/s
        }
    }
    
    rlState.non_revenue_water =
    std::max(0.0f, totalWaterPumped - totalDemand - totalLeakFlow);

    if (totalWaterPumped > 0.001f) {
        rlState.supply_efficiency = totalDemand / totalWaterPumped;
    } else {
        rlState.supply_efficiency = 0.0f;
    }
    
    // Calculate pressure compliance
    int compliantPipes = 0;
    int totalWaterPipes = 0;
    
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) {
            totalWaterPipes++;
            for (const auto& sensor : sensors) {
                if (sensor.connectedPipeID == pipe.id) {
                    if (sensor.pressure >= 100.0f && sensor.pressure <= 300.0f) {
                        compliantPipes++;
                    }
                    break;
                }
            }
        }
    }
    
    if (totalWaterPipes > 0) {
        rlState.pressure_compliance = static_cast<float>(compliantPipes) / totalWaterPipes;
    }
    
    // Calculate service continuity
    int activePipes = 0;
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL && pipe.flowRate > 0.001f) {
            activePipes++;
        }
    }
    
    if (totalWaterPipes > 0) {
        rlState.service_continuity = static_cast<float>(activePipes) / totalWaterPipes;
    }
    
    // Calculate average response time (simplified - time since last leak detection)
    float totalResponseTime = 0.0f;
    int leakEvents = 0;
    time_t currentTime = time(nullptr);
    
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            // Find when this leak was first detected (simplified: use pipe ID as time offset)
            float detectionTime = pipe.id * 10.0f; // Simplified
            totalResponseTime += (currentTime % 3600) - detectionTime; // More realistic in real system
            leakEvents++;
        }
    }
    
    if (leakEvents > 0) {
        rlState.response_time = totalResponseTime / leakEvents;
    } else {
        rlState.response_time = 0.0f;
    }
    
    // Update cumulative volumes
    for (size_t i = 0; i < sensors.size() && i < 50; i++) {
        rlState.cumulative_volume_sensor[i] = sensors[i].cumulativeVolume_m3;
    }
    
    // Update hourly usage
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    int currentHour = timeinfo->tm_hour;
    
    // Calculate total hourly usage across all sensors
    float hourlyTotal = 0.0f;
    for (const auto& sensor : sensors) {
        hourlyTotal += sensor.getHourlyUsage(currentHour);
    }
    rlState.hourly_usage[currentHour] = hourlyTotal;
    
    // Track hourly usage pattern (last 24 hours)
    static float hourlyPattern[24] = {0};
    static bool patternInitialized = false;
    
    if (!patternInitialized) {
        for (int i = 0; i < 24; i++) {
            hourlyPattern[i] = 0.0f;
        }
        patternInitialized = true;
    }
    
    hourlyPattern[currentHour] = hourlyTotal;
    
    // Update episode step counter
    episodeStep++;
}

float CityNetwork::calculateTotalDemand() const {
    float total = 0;
    for (const auto& building : buildings) {
        total += building.currentWaterFlow;
    }
    return total;
}

void CityNetwork::generateCity(int numBuildings) {
    buildings.clear();
    clusters.clear();
    pipes.clear();
    sensors.clear();
    zones.clear();
    reservoirParticles.clear();
    
    std::cout << "\n=== GENERATING ROS2 SCADA CITY ===\n";
    std::cout << "Target buildings: " << numBuildings << "\n";
    
    int numClusters = (numBuildings + BUILDINGS_PER_CLUSTER - 1) / BUILDINGS_PER_CLUSTER;
    int clustersPerRow = (int)ceil(sqrt(numClusters));
    
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
    
    // Create zones (one per cluster)
    createZones();
    
    // Initialize reservoir
    reservoir = Reservoir();
    reservoir.position = Vec3(-cityExtent - 80, 20, -cityExtent - 80);
    
    std::cout << "Created " << buildings.size() << " buildings in " << clusters.size() << " clusters\n";
    std::cout << "City extent: " << cityExtent << "m\n";
    std::cout << "Created " << zones.size() << " zones\n";
    
    generateWaterNetwork();
    generateSewageNetwork();
    createSensors();
    printSensorMapping();
    
    std::cout << "Total pipes: " << pipes.size() << "\n";
    std::cout << "Total sensors: " << sensors.size() << "\n";
    std::cout << "=== CITY GENERATION COMPLETE ===\n\n";
}

void CityNetwork::createZones() {
    zones.clear();
    for (size_t i = 0; i < clusters.size(); i++) {
        zones.push_back(Zone(i, clusters[i].centerPos));
        
        // Assign buildings to zones
        for (int buildingID : clusters[i].buildingIDs) {
            if (buildingID < (int)buildings.size()) {
                buildings[buildingID].zoneID = i;
            }
        }
    }
}

Zone* CityNetwork::getZoneForBuilding(int buildingID) {
    if (buildingID >= 0 && buildingID < (int)buildings.size()) {
        int zoneID = buildings[buildingID].zoneID;
        if (zoneID >= 0 && zoneID < (int)zones.size()) {
            return &zones[zoneID];
        }
    }
    return nullptr;
}

Zone* CityNetwork::getZoneForPipe(int pipeID) {
    if (pipeID >= 0 && pipeID < (int)pipes.size()) {
        int zoneID = pipes[pipeID].zoneID;
        if (zoneID >= 0 && zoneID < (int)zones.size()) {
            return &zones[zoneID];
        }
    }
    return nullptr;
}

void CityNetwork::generateWaterNetwork() {
    int pipeID = 0;
    
    Vec3 trunkStart = reservoir.position + Vec3(0, -10, 0);
    Vec3 trunkEnd(0, 0, 0);
    pipes.push_back(Pipe(pipeID++, trunkStart, trunkEnd, TRUNK_MAIN, TRUNK_DIAMETER, 0));
    
    for (size_t i = 0; i < clusters.size(); i++) {
        Vec3 secondaryEnd = clusters[i].centerPos + Vec3(0, 1, 0);
        clusters[i].secondaryMainID = pipeID;
        pipes.push_back(Pipe(pipeID++, trunkEnd, secondaryEnd, SECONDARY_MAIN, SECONDARY_DIAMETER, i));
    }
    
    for (size_t i = 0; i < clusters.size(); i++) {
        if (clusters[i].buildingIDs.empty()) continue;
        
        Vec3 center = clusters[i].centerPos + Vec3(0, 0.5f, 0);
        float ringRadius = CITY_GRID_SPACING * 1.5f;
        
        Vec3 ringPoints[4] = {
            center + Vec3(-ringRadius, 0, -ringRadius),
            center + Vec3(ringRadius, 0, -ringRadius),
            center + Vec3(ringRadius, 0, ringRadius),
            center + Vec3(-ringRadius, 0, ringRadius)
        };
        
        for (int j = 0; j < 4; ++j) {
            int next = (j + 1) % 4;
            clusters[i].ringPipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(pipeID++, ringPoints[j], ringPoints[next], RING_MAIN, RING_DIAMETER, i));
        }
        
        pipes.push_back(Pipe(pipeID++, center, ringPoints[0], RING_MAIN, RING_DIAMETER, i));
    }
    
    for (size_t i = 0; i < clusters.size(); i++) {
        for (int bid : clusters[i].buildingIDs) {
            Building& bldg = buildings[bid];
            
            float minDist = 1e9f;
            Vec3 nearestRingPoint;
            for (int ringPipeID : clusters[i].ringPipeIDs) {
                float d1 = (pipes[ringPipeID].start - bldg.position).length();
                float d2 = (pipes[ringPipeID].end - bldg.position).length();
                if (d1 < minDist) { minDist = d1; nearestRingPoint = pipes[ringPipeID].start; }
                if (d2 < minDist) { minDist = d2; nearestRingPoint = pipes[ringPipeID].end; }
            }
            
            Vec3 serviceEnd = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.25f, 0.5f, BUILDING_FOOTPRINT * 0.25f);
            bldg.servicePipeID = pipeID;
            clusters[i].servicePipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(pipeID++, nearestRingPoint, serviceEnd, SERVICE_PIPE, SERVICE_DIAMETER, i));
        }
    }
}

void CityNetwork::generateSewageNetwork() {
    int pipeID = pipes.size();
    
    for (size_t i = 0; i < clusters.size(); i++) {
        Vec3 collectorPoint = clusters[i].centerPos + Vec3(0, -2.5f, CLUSTER_SPACING * 0.7f);
        
        for (int bid : clusters[i].buildingIDs) {
            Building& bldg = buildings[bid];
            Vec3 lateralStart = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.75f, 1.0f, BUILDING_FOOTPRINT * 0.75f);
            Vec3 lateralEnd = collectorPoint;
            lateralEnd.y = -2.0f;
            
            bldg.sewerPipeID = pipeID;
            clusters[i].sewerPipeIDs.push_back(pipeID);
            pipes.push_back(Pipe(
                pipeID++,
                lateralStart,
                lateralEnd,
                SEWAGE_LATERAL,
                SERVICE_DIAMETER * 1.5f,
                i
            ));
        }
        
        Vec3 collectorEnd = Vec3(collectorPoint.x, -3.5f, stpPos.z - 30.0f);
        clusters[i].sewageCollectorID = pipeID;
        pipes.push_back(Pipe(
            pipeID++,
            collectorPoint,
            collectorEnd,
            SEWAGE_COLLECTOR,
            SECONDARY_DIAMETER * 1.2f,
            i
        ));
    }
    
    Vec3 interceptorStart(0, -4.0f, stpPos.z - 30.0f);
    Vec3 interceptorEnd = stpPos + Vec3(0, 0.5f, -15.0f);
    pipes.push_back(Pipe(
        pipeID++,
        interceptorStart,
        interceptorEnd,
        SEWAGE_INTERCEPTOR,
        TRUNK_DIAMETER,
        -1
    ));
}

void CityNetwork::createSensors() {
    int sensorID = 0;
    int waterSensorCount = 0;
    int sewageSensorCount = 0;
    
    // Clear any existing mapping
    sensorIdToIndex.clear();
    
    std::cout << "\n=== CREATING SENSORS ===\n";
    
    // Reservoir water sensor
    reservoirWaterSensorID = sensorID;
    Vec3 resSensorPos = reservoir.position + Vec3(0, -5, 0);
    sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, resSensorPos, 0));
    sensors.back().connectedPipeID = 0;
    sensorIdToIndex[sensors.back().id] = sensors.size() - 1;
    std::cout << "  Created sensor ID " << reservoirWaterSensorID << " (Reservoir)\n";
    
    // Cluster water sensors
    for (size_t i = 0; i < clusters.size(); i++) {
        clusters[i].waterSensorID = sensorID;
        Vec3 clusterWaterPos = clusters[i].centerPos + Vec3(-3, 2.5, 0);
        sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, clusterWaterPos, i));
        sensors.back().connectedPipeID = clusters[i].secondaryMainID;
        sensorIdToIndex[sensors.back().id] = sensors.size() - 1;
        if (i < 5) std::cout << "  Created sensor ID " << (sensorID-1) << " (Cluster " << i << ")\n";
    }
    
    // Building water sensors
    for (auto& bldg : buildings) {
        bldg.waterSensorID = sensorID;
        Vec3 bldgWaterPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.15f, 1.8f, BUILDING_FOOTPRINT * 0.25f);
        sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, bldgWaterPos, bldg.zoneID));
        sensors.back().connectedPipeID = bldg.servicePipeID;
        sensorIdToIndex[sensors.back().id] = sensors.size() - 1;
        if (bldg.id < 5) std::cout << "  Created sensor ID " << (sensorID-1) << " (Building " << bldg.id << " water)\n";
    }
    
    // Building sewage sensors
    for (auto& bldg : buildings) {
        bldg.sewageSensorID = sensorID;
        Vec3 bldgSewerPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.85f, 1.8f, BUILDING_FOOTPRINT * 0.75f);
        sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, bldgSewerPos, bldg.zoneID));
        sensors.back().connectedPipeID = bldg.sewerPipeID;
        sensorIdToIndex[sensors.back().id] = sensors.size() - 1;
        if (bldg.id < 5) std::cout << "  Created sensor ID " << (sensorID-1) << " (Building " << bldg.id << " sewage)\n";
    }
    
    // Cluster sewage sensors
    for (size_t i = 0; i < clusters.size(); i++) {
        clusters[i].sewageSensorID = sensorID;
        Vec3 clusterSewerPos = clusters[i].centerPos + Vec3(3, -0.5, CLUSTER_SPACING * 0.6f);
        sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, clusterSewerPos, i));
        sensors.back().connectedPipeID = clusters[i].sewageCollectorID;
        sensorIdToIndex[sensors.back().id] = sensors.size() - 1;
        if (i < 5) std::cout << "  Created sensor ID " << (sensorID-1) << " (Cluster " << i << " sewage)\n";
    }
    
    // STP sewage sensor
    stpSewerSensorID = sensorID;
    Vec3 stpSensorPos = stpPos + Vec3(0, 3, -15);
    sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, stpSensorPos, -1));
    sensorIdToIndex[sensors.back().id] = sensors.size() - 1;
    std::cout << "  Created sensor ID " << (sensorID-1) << " (STP)\n";
    
    std::cout << "\n=== SENSOR SUMMARY ===\n";
    std::cout << "Total Sensors Created: " << sensors.size() << "\n";
    std::cout << "Sensor ID Range: 0 to " << (sensors.size() - 1) << "\n";
    std::cout << "Water Sensors: " << waterSensorCount << "\n";
    std::cout << "Sewage Sensors: " << sewageSensorCount << "\n";
    
    if (sensors.size() < 111) {
        std::cout << "\n⚠ WARNING: Your ROS2 data expects sensor IDs up to 110, but simulation only has " 
                  << sensors.size() << " sensors.\n";
        std::cout << "   Adjust building count or sensor generation to match expected IDs.\n";
    }
}

void CityNetwork::loadLeakData(const char* filename) {
    std::cout << "Loading leak data from: " << filename << "\n";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "  No leak file found - NO LEAKS (default)\n";
        return;
    }
    
    std::string line;
    std::getline(file, line);
    
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
                
                // Update zone leak status
                Zone* zone = getZoneForPipe(pipeID);
                if (zone) {
                    zone->leakFlag = true;
                    zone->leakSeverity = leakRate / 1000.0f; // Convert to m³/s
                }
            }
        }
    }
    file.close();
}

void CityNetwork::updateBuildingWaterUsage(float dt) {
    totalWaterConsumed = 0;
    
    for (auto& building : buildings) {
        building.updateWaterUsage(dt, dayNight);
        totalWaterConsumed += building.currentWaterFlow * dt;
    }
}

void CityNetwork::updatePipePhysics(float dt) {
    // Update valve positions
    for (auto& sensor : sensors) {
        sensor.updateValve(dt);
    }
    
    // Calculate total demand
    float totalDemand = calculateTotalDemand();
    
    // Track flows for water balance
    float totalLeakFlow = 0.0f;
    
    // ================== ZONE AGGREGATION INIT ==================
    std::vector<float> zoneFlow(zones.size(), 0.0f);
    std::vector<float> zonePressure(zones.size(), 0.0f);
    std::vector<int>   zoneCount(zones.size(), 0);
    
    // Reset per-frame zone flags
    for (auto& zone : zones) {
        zone.leakFlag = false;
        zone.overflowFlag = false;
        zone.leakSeverity = 0.0f;
    }
    
    // ================== PIPE PHYSICS LOOP ==================
    for (auto& pipe : pipes) {
        float valveFactor = 1.0f;
        Sensor* connectedSensor = nullptr;
        
        // Find sensor controlling this pipe
        for (auto& sensor : sensors) {
            if (sensor.connectedPipeID == pipe.id) {
                valveFactor = sensor.valveState / 100.0f;
                connectedSensor = &sensor;
                break;
            }
        }
        
        // Calculate flow based on pipe type
        if (pipe.type < SEWAGE_LATERAL) {
            // Water pipes
            if (pipe.type == TRUNK_MAIN) {
                // Trunk main carries total system demand
                pipe.flowRate = totalDemand * valveFactor;
            } else if (pipe.type == SECONDARY_MAIN) {
                // Secondary mains distribute to zones
                float zoneDemand = 0.0f;
                for (const auto& building : buildings) {
                    if (building.zoneID == pipe.zoneID) {
                        zoneDemand += building.currentWaterFlow;
                    }
                }
                pipe.flowRate = zoneDemand * valveFactor;
            } else if (pipe.type == RING_MAIN) {
                // Ring mains supply clusters
                float clusterDemand = 0.0f;
                for (const auto& building : buildings) {
                    for (int clusterBuildingID : clusters[pipe.zoneID].buildingIDs) {
                        if (building.id == clusterBuildingID) {
                            clusterDemand += building.currentWaterFlow;
                            break;
                        }
                    }
                }
                pipe.flowRate = clusterDemand * valveFactor;
            } else if (pipe.type == SERVICE_PIPE) {
                // Service pipes supply individual buildings
                for (const auto& building : buildings) {
                    if (building.servicePipeID == pipe.id) {
                        pipe.flowRate = building.currentWaterFlow * valveFactor;
                        break;
                    }
                }
            }
            
            // Apply leak effects
            if (pipe.hasLeak) {
                // Convert leak rate from L/s to m³/s
                float leakFlow_m3s = pipe.leakRate / 1000.0f;
                pipe.flowRate += leakFlow_m3s;
                totalLeakFlow += leakFlow_m3s;
            }
            
            // Calculate pressure drop using Darcy-Weisbach
            float pressureDrop = pipe.calculatePressureDrop(pipe.flowRate);
            
            // Update sensor pressure with realistic pressure propagation
            if (connectedSensor) {
                // Calculate pressure based on upstream pressure minus drop
                float upstreamPressure = DEFAULT_PRESSURE;
                
                // Find upstream pipe pressure
                for (const auto& otherPipe : pipes) {
                    if (otherPipe.end.x == pipe.start.x && 
                        otherPipe.end.y == pipe.start.y && 
                        otherPipe.end.z == pipe.start.z) {
                        // This pipe connects to our pipe
                        for (const auto& upstreamSensor : sensors) {
                            if (upstreamSensor.connectedPipeID == otherPipe.id) {
                                upstreamPressure = upstreamSensor.pressure;
                                break;
                            }
                        }
                        break;
                    }
                }
                
                if (!connectedSensor->pressureFromROS) {
                    connectedSensor->pressure = std::max(0.0f, upstreamPressure - pressureDrop);
                    connectedSensor->pressureFromROS = false; // This is simulated pressure
                }
                connectedSensor->updateVolume(pipe.flowRate, dt);
            }
            
            // Update particles with realistic flow
            Color particleColor(0.3f, 0.6f, 1.0f, 0.8f);
            for (const auto& building : buildings) {
                if (building.servicePipeID == pipe.id) {
                    // Color based on building water usage
                    float usageIntensity = building.currentWaterFlow / 0.01f; // Normalize
                    particleColor = Color(
                        0.2f + 0.3f * usageIntensity,
                        0.4f + 0.3f * usageIntensity,
                        0.8f + 0.1f * usageIntensity,
                        0.8f
                    );
                    break;
                }
            }
            
            // Update pipe particles
            pipe.updateParticles(dt, particleColor);
            
        } else {
            // Sewage pipes
            if (pipe.type == SEWAGE_LATERAL) {
                // Building sewer laterals
                for (const auto& building : buildings) {
                    if (building.sewerPipeID == pipe.id) {
                        // Sewage flow ≈ 80% of water consumption
                        pipe.flowRate = building.currentWaterFlow * building.sewageFactor * valveFactor;
                        break;
                    }
                }
            } else if (pipe.type == SEWAGE_COLLECTOR) {
                // Cluster collectors
                float clusterSewage = 0.0f;
                for (const auto& building : buildings) {
                    for (int clusterBuildingID : clusters[pipe.zoneID].buildingIDs) {
                        if (building.id == clusterBuildingID) {
                            clusterSewage += building.currentWaterFlow * 0.8f;
                            break;
                        }
                    }
                }
                pipe.flowRate = clusterSewage * valveFactor;
            } else if (pipe.type == SEWAGE_INTERCEPTOR) {
                // Main interceptor to STP
                float totalSewage = 0.0f;
                for (const auto& pipe2 : pipes) {
                    if (pipe2.type == SEWAGE_COLLECTOR) {
                        totalSewage += pipe2.flowRate;
                    }
                }
                pipe.flowRate = totalSewage * valveFactor;
            }
            
            // Sewage pressure is typically near atmospheric
            if (connectedSensor) {
                if (!connectedSensor->isROSFresh()) {
                    connectedSensor->pressure = 101.3f; // Atmospheric pressure in kPa
                    connectedSensor->pressureFromROS = false;
                }
                connectedSensor->updateVolume(pipe.flowRate, dt);
            }
        }
        
        // ================== ZONE AGGREGATION (ACCUMULATE ONLY) ==================
        Zone* zone = getZoneForPipe(pipe.id);
        if (!zone) continue;
        
        int zid = zone->id;
        float pressure = connectedSensor ? connectedSensor->pressure : DEFAULT_PRESSURE;
        
        // Aggregate zone statistics
        zoneFlow[zid] += pipe.flowRate;
        zonePressure[zid] += pressure;
        zoneCount[zid]++;
        
        // Set zone flags (per-pipe checks)
        if (pipe.hasLeak && pipe.leakRate > 5.0f) {
            zone->leakFlag = true;
            zone->leakSeverity = std::max(zone->leakSeverity, pipe.leakRate / 1000.0f);
        }
        
        if (pipe.type >= SEWAGE_LATERAL && pipe.flowRate > (pipe.diameter * 0.5f)) {
            zone->overflowFlag = true;
        }
    } // End of pipe loop
    
    // ================== ZONE FINALIZATION ==================
    for (size_t i = 0; i < zones.size(); i++) {
        if (zoneCount[i] == 0) continue;
        
        float avgPressure = zonePressure[i] / zoneCount[i];
        float totalFlow = zoneFlow[i];
        
        zones[i].updateStatistics(totalFlow, avgPressure);
    }
    
    // ================== RESERVOIR UPDATE ==================
    // Find TRUNK_MAIN pipe for reservoir outflow
    for (const auto& pipe : pipes) {
        if (pipe.type == TRUNK_MAIN) {
            reservoir.outflowRate = pipe.flowRate;
            break;
        }
    }
    
    // ================== RL STATE UPDATE ==================
    // Calculate Non-Revenue Water (NRW)
    float totalWaterPumped = reservoir.outflowRate;
    float totalBilledWater = totalDemand; // Water that reaches customers
    float nrw_m3s = totalWaterPumped - totalBilledWater - totalLeakFlow;
    
    // Update RL state with flow metrics
    rlState.non_revenue_water = std::max(0.0f, nrw_m3s);
    rlState.supply_efficiency = totalBilledWater / std::max(0.001f, totalWaterPumped);
    
    // Find peak flow rate
    float currentPeak = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.flowRate > currentPeak) {
            currentPeak = pipe.flowRate;
        }
    }
    rlState.peak_flow_rate = std::max(rlState.peak_flow_rate, currentPeak);
    
    // Calculate average flow and variance
    float sumFlow = 0.0f;
    float sumSquaredFlow = 0.0f;
    int count = 0;
    
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) { // Water pipes only
            sumFlow += pipe.flowRate;
            sumSquaredFlow += pipe.flowRate * pipe.flowRate;
            count++;
        }
    }
    
    if (count > 0) {
        rlState.avg_flow_rate = sumFlow / count;
        float variance = (sumSquaredFlow / count) - (rlState.avg_flow_rate * rlState.avg_flow_rate);
        rlState.flow_variance = std::max(0.0f, variance);
    }
    
    // Update pressure compliance (percentage of pipes with pressure in range)
    int compliantSensors = 0;
    int totalWaterSensors = 0;
    
    for (const auto& sensor : sensors) {
        if (sensor.type == WATER_SENSOR) {
            totalWaterSensors++;
            if (sensor.pressure >= 100.0f && sensor.pressure <= 300.0f) {
                compliantSensors++;
            }
        }
    }
    
    rlState.pressure_compliance = totalWaterSensors > 0 ?
        (float)compliantSensors / totalWaterSensors : 0.0f;
    
    // Update service continuity (pipes with flow > 0)
    int activePipes = 0;
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL && pipe.flowRate > 0.001f) {
            activePipes++;
        }
    }
    rlState.service_continuity = (float)activePipes / std::max(1, (int)pipes.size());
    
    // Update cumulative volumes for RL state
    for (size_t i = 0; i < sensors.size() && i < 50; i++) {
        rlState.cumulative_volume_sensor[i] = sensors[i].cumulativeVolume_m3;
    }
    
    // Update hourly usage in RL state
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    int currentHour = timeinfo->tm_hour;
    
    float hourlyTotal = 0.0f;
    for (const auto& sensor : sensors) {
        hourlyTotal += sensor.getHourlyUsage(currentHour);
    }
    rlState.hourly_usage[currentHour] = hourlyTotal;
}

void CityNetwork::updateReservoir(float dt) {
    float totalDemand = calculateTotalDemand();
    
    float totalLeaks = 0;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeaks += pipe.leakRate / 1000.0f; // Convert L/s to m³/s
        }
    }
    
    reservoir.update(dt, totalDemand + totalLeaks);
    
    totalWaterSupplied += reservoir.outflowRate * dt;
    totalLeakWater += totalLeaks * dt;
    
    if (reservoirWaterSensorID >= 0 && reservoirWaterSensorID < (int)sensors.size()) {
        sensors[reservoirWaterSensorID].waterLevel = reservoir.getLevelPercent();
        if (!sensors[reservoirWaterSensorID].isROSFresh()) {
            sensors[reservoirWaterSensorID].pressure = 150.0f;
            sensors[reservoirWaterSensorID].pressureFromROS = false;
        }
    }
}

void CityNetwork::updateZones(float dt) {
    for (auto& zone : zones) {
        // Check for overflow (simplified - if flow exceeds capacity)
        float capacity = 10.0f; // m³/s capacity per zone
        zone.overflowFlag = (zone.totalFlow > capacity * 0.9f);
        
        // Update leak detection (simplified)
        if (zone.leakFlag) {
            zone.leakSeverity *= 0.99f; // Gradual reduction
            if (zone.leakSeverity < 0.01f) {
                zone.leakFlag = false;
            }
        }
    }
}

void CityNetwork::generateReservoirParticles(float dt) {
    for (auto it = reservoirParticles.begin(); it != reservoirParticles.end();) {
        it->update(dt);
        if (it->life <= 0) {
            it = reservoirParticles.erase(it);
        } else {
            ++it;
        }
    }
    
    if (reservoir.volume > 0 && reservoirParticles.size() < 100) {
        float radius = 10.0f;
        float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
        float dist = static_cast<float>(rand()) / RAND_MAX * radius;
        
        Vec3 spawnPos = reservoir.position + Vec3(
            cos(angle) * dist,
            -5 + static_cast<float>(rand()) / RAND_MAX * 5,
            sin(angle) * dist
        );
        
        Vec3 direction = (reservoir.position + Vec3(0, -10, 0) - spawnPos).normalized();
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
    std::lock_guard<std::mutex> lock(city_mutex);  // ALWAYS lock, no try_lock
    
    simulationTime += dt;
    
    // Update day-night cycle
    dayNight.update(dt);
    
    updateBuildingWaterUsage(dt);
    updatePipePhysics(dt);
    updateReservoir(dt);
    updateZones(dt);
    generateReservoirParticles(dt);
    
    // Update UDP broadcast timer
    udpBroadcastTimer += dt;
    if (udpBroadcastTimer >= UDP_BROADCAST_INTERVAL) {
        broadcastRLState();
        udpBroadcastTimer = 0.0f;
    }
    
    // Update system JSON
    systemJsonUpdateTimer += dt;
    if (systemJsonUpdateTimer >= SYSTEM_JSON_UPDATE_INTERVAL) {
        exportSystemStateJSON("/home/arka/aqua_sentinel/simulator/system_state.json");
        systemJsonUpdateTimer = 0.0f;
    }
}

void CityNetwork::drawReservoirStructure(Vec3 pos, float waterLevel) const {
    float tankRadius = 25.0f;
    float tankHeight = 30.0f;
    float wallThickness = 1.0f;

    glColor3f(0.7f, 0.7f, 0.7f);

    // Tank walls
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight / 2, pos.z);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, tankRadius, tankRadius, tankHeight, 64, 8);
    gluDeleteQuadric(quad);
    glPopMatrix();

    // Tank top
    glPushMatrix();
    glTranslatef(pos.x, pos.y + tankHeight / 2, pos.z);
    glRotatef(-90, 1, 0, 0);
    GLUquadric* disk = gluNewQuadric();
    gluDisk(disk, 0, tankRadius, 64, 8);
    gluDeleteQuadric(disk);
    glPopMatrix();

    // Water volume
    if (waterLevel > 0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glColor4f(0.1f, 0.3f, 0.8f, 0.7f);
        glPushMatrix();
        glTranslatef(pos.x, pos.y - tankHeight / 2 + waterLevel, pos.z);
        glRotatef(-90, 1, 0, 0);
        GLUquadric* waterDisk = gluNewQuadric();
        gluDisk(waterDisk, 0, tankRadius, 64, 8);
        gluDeleteQuadric(waterDisk);
        glPopMatrix();

        glColor4f(0.15f, 0.4f, 0.9f, 0.5f);
        glPushMatrix();
        glTranslatef(pos.x, pos.y - tankHeight / 2, pos.z);
        GLUquadric* waterQuad = gluNewQuadric();
        gluCylinder(
            waterQuad,
            tankRadius - wallThickness,
            tankRadius - wallThickness,
            waterLevel,
            64, 8
        );
        gluDeleteQuadric(waterQuad);
        glPopMatrix();

        glDisable(GL_BLEND);
    }
}

void CityNetwork::drawReservoirWithParticles() const {
    drawReservoirStructure(reservoir.position, reservoir.level);
    
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
// RENDERING FUNCTIONS (UNCHANGED FROM FIRST CODE)
// ============================================================================

// Global variables
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
bool showGround = true;
bool transparentBuildings = false;
bool showWaterParticles = true;
bool showReservoirParticles = true;
bool showSunMoon = true;
int highlightedCluster = -1;
int currentBuildingCount = 200;

void drawSunMoon() {
    if (!showSunMoon) return;
    
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    
    float sunHeight = 800.0f;
    float sunDistance = 2000.0f;
    float sunAngle = dayNight.timeOfDay * 15.0f; // 15 degrees per hour
    
    float sunX = sinf(sunAngle * M_PI / 180.0f) * sunDistance;
    float sunZ = cosf(sunAngle * M_PI / 180.0f) * sunDistance;
    
    glPushMatrix();
    glTranslatef(sunX, sunHeight, sunZ);
    
    if (dayNight.isDay()) {
        // Draw sun
        // Sun glow
        glColor4f(dayNight.sunColor.r, dayNight.sunColor.g, dayNight.sunColor.b, 0.3f);
        glutSolidSphere(100.0f, 32, 32);
        
        // Sun core
        glColor3f(dayNight.sunColor.r, dayNight.sunColor.g, dayNight.sunColor.b);
        glutSolidSphere(40.0f, 32, 32);
    } else {
        // Draw moon
        // Moon glow
        glColor4f(dayNight.moonColor.r, dayNight.moonColor.g, dayNight.moonColor.b, 0.2f);
        glutSolidSphere(80.0f, 32, 32);
        
        // Moon core
        glColor3f(dayNight.moonColor.r, dayNight.moonColor.g, dayNight.moonColor.b);
        glutSolidSphere(30.0f, 32, 32);
    }
    
    glPopMatrix();
    
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

void setupLighting() {
    // Set up sun light
    float sunHeight = 800.0f;
    float sunDistance = 2000.0f;
    float sunAngle = dayNight.timeOfDay * 15.0f; // 15 degrees per hour
    
    float sunX = sinf(sunAngle * M_PI / 180.0f) * sunDistance;
    float sunZ = cosf(sunAngle * M_PI / 180.0f) * sunDistance;
    
    GLfloat sunPos[] = {sunX, sunHeight, sunZ, 1.0f};
    GLfloat sunAmb[] = {dayNight.ambientLight * 0.4f, dayNight.ambientLight * 0.4f, dayNight.ambientLight * 0.4f, 1.0f};
    GLfloat sunDif[] = {dayNight.sunIntensity, dayNight.sunIntensity * 0.9f, dayNight.sunIntensity * 0.8f, 1.0f};
    GLfloat sunSpec[] = {1.0f, 1.0f, 1.0f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, sunPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, sunAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, sunDif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, sunSpec);
    
    // Set up ambient light
    GLfloat ambPos[] = {0.0f, 500.0f, 0.0f, 1.0f};
    GLfloat ambAmb[] = {dayNight.ambientLight * 0.3f, dayNight.ambientLight * 0.3f, dayNight.ambientLight * 0.4f, 1.0f};
    GLfloat ambDif[] = {dayNight.ambientLight * 0.5f, dayNight.ambientLight * 0.5f, dayNight.ambientLight * 0.6f, 1.0f};
    
    glLightfv(GL_LIGHT1, GL_POSITION, ambPos);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambAmb);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, ambDif);
}

void drawCylinder(Vec3 start, Vec3 end, float radius, Color color) {
    glColor3f(color.r, color.g, color.b);
    
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
    
    float pulse = 0.75f + 0.35f * sinf(city.simulationTime * 3.0f);
    
    if (sensor.active) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        
        if (sensor.isStale()) {
            glColor4f(col.r * 0.5f, col.g * 0.3f, col.b * 0.3f, 0.2f);
        } else {
            glColor4f(col.r, col.g, col.b, 0.3f * pulse);
        }
        
        float glowSize = 2.5f;
        glPushMatrix();
        glTranslatef(sensor.position.x, sensor.position.y, sensor.position.z);
        glutSolidSphere(glowSize, 16, 16);
        glPopMatrix();
        glDisable(GL_BLEND);
    }
    
    GLfloat mat_specular[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat mat_shininess[] = { 80.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    
    float sensorSize = 2.2f;
    if (sensor.isStale()) {
        glColor3f(col.r * 0.5f, col.g * 0.5f, col.b * 0.5f);
    } else {
        glColor3f(col.r * pulse, col.g * pulse, col.b * pulse);
    }
    
    glPushMatrix();
    glTranslatef(sensor.position.x, sensor.position.y, sensor.position.z);
    glutSolidCube(sensorSize);
    glPopMatrix();
    
    // Valve wheel
    glDisable(GL_LIGHTING);
    if (sensor.isStale()) {
        glColor3f(col.r * 0.4f, col.g * 0.4f, col.b * 0.4f);
    } else {
        glColor3f(col.r * 0.7f, col.g * 0.7f, col.b * 0.7f);
    }
    glPushMatrix();
    glTranslatef(sensor.position.x, sensor.position.y + 1.8f, sensor.position.z);
    glRotatef(90, 1, 0, 0);
    glutSolidTorus(0.4, 0.9, 10, 20);
    glPopMatrix();
    glEnable(GL_LIGHTING);
    
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        
        if (sensor.isStale()) {
            glColor3f(0.6f, 0.6f, 0.6f);
        } else if (sensor.type == WATER_SENSOR) {
            glColor3f(0.3f, 0.85f, 1.0f);
        } else {
            glColor3f(1.0f, 0.7f, 0.2f);
        }
        
        glRasterPos3f(sensor.position.x, sensor.position.y + 3.5f, sensor.position.z);
        std::stringstream ss;
        ss << "ID:" << sensor.id << " " << sensor.name;
        if (sensor.last_esp_id != -1) {
            ss << " [ESP" << sensor.last_esp_id << "]";
        }
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
        }
        
        glRasterPos3f(sensor.position.x, sensor.position.y + 2.5f, sensor.position.z);
        ss.str("");
        ss << "V:" << (int)sensor.valveState << "% P:" << (int)sensor.pressure << "kPa L:" << (int)sensor.waterLevel << "%";
        if (sensor.isStale()) {
            ss << " [STALE]";
        }
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        
        glEnable(GL_LIGHTING);
    }
}

void drawPipe(const Pipe& pipe) {
    Color color = pipe.getColor();
    float radius = pipe.diameter / 2.0f;
    
    if (pipe.type >= SEWAGE_LATERAL) {
        Color sewerColor(0.65f, 0.38f, 0.12f);
        drawCylinder(pipe.start, pipe.end, radius, sewerColor);
        drawSphere(pipe.start, radius * 1.4f, sewerColor);
        drawSphere(pipe.end, radius * 1.4f, sewerColor);
        
        Vec3 dir = (pipe.end - pipe.start).normalized();
        float len = pipe.length;
        int numStripes = std::max(3, (int)(len / 3.0f));
        
        glDisable(GL_LIGHTING);
        for (int i = 0; i < numStripes; i++) {
            float t = (float)i / numStripes;
            Vec3 stripePos = pipe.start + (pipe.end - pipe.start) * t;
            glColor3f(1.0f, 0.6f, 0.1f);
            glPushMatrix();
            glTranslatef(stripePos.x, stripePos.y, stripePos.z);
            glutSolidSphere(radius * 1.3f, 12, 12);
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
        
    } else {
        Color waterColor(0.25f, 0.65f, 0.95f);
        drawCylinder(pipe.start, pipe.end, radius, waterColor);
        
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f(0.4f, 0.8f, 1.0f, 0.3f);
        drawCylinder(pipe.start, pipe.end, radius * 1.1f, Color(0.4f, 0.8f, 1.0f));
        glDisable(GL_BLEND);
        
        drawSphere(pipe.start, radius * 1.3f, waterColor);
        drawSphere(pipe.end, radius * 1.3f, waterColor);
        
        if (showWaterParticles && pipe.type < SEWAGE_LATERAL && pipe.flowRate > 0.1f) {
            pipe.drawParticles();
        }
    }
    
    if (pipe.hasLeak && showLeaks) {
        // Draw leak particles with simulation time
        pipe.drawLeakParticles(city.simulationTime);
        
        // Also draw the existing leak indicator
        Vec3 mid = (pipe.start + pipe.end) * 0.5f;
        float pulse = 0.8f + 0.4f * sinf(city.simulationTime * 5.0f);
        
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f(1.0f, 0.0f, 0.0f, 0.4f);
        drawSphere(mid, 3.5f * pulse, Color(1, 0, 0));
        glDisable(GL_BLEND);
        
        drawSphere(mid, 2.0f * pulse, Color(1, 0, 0));
        
        glDisable(GL_LIGHTING);
        glColor3f(1, 0.3f, 0);
        glPushMatrix();
        glTranslatef(mid.x, mid.y + 5, mid.z);
        glRotatef(-90, 1, 0, 0);
        glutSolidCone(2.5, 5.0, 12, 2);
        glPopMatrix();
        
        glColor3f(1, 0, 0);
        glRasterPos3f(mid.x, mid.y + 8, mid.z);
        std::stringstream ss;
        ss << "⚠ LEAK: Pipe " << pipe.id << " (" << std::fixed << std::setprecision(1) << pipe.leakRate << " L/s)";
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
        glEnable(GL_LIGHTING);
    }
}

void drawBuilding(const Building& bldg) {
    Color baseColor(0.55f, 0.55f, 0.6f);
    Color roofColor(0.4f, 0.4f, 0.45f);
    
    if (transparentBuildings) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(baseColor.r, baseColor.g, baseColor.b, 0.25f);
    } else {
        glColor3f(baseColor.r, baseColor.g, baseColor.b);
    }
    
    glPushMatrix();
    glTranslatef(bldg.position.x + BUILDING_FOOTPRINT/2, bldg.position.y + bldg.height/2, bldg.position.z + BUILDING_FOOTPRINT/2);
    glScalef(BUILDING_FOOTPRINT, bldg.height, BUILDING_FOOTPRINT);
    glutSolidCube(1.0f);
    glPopMatrix();
    
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
    
    Vec3 riserPos = bldg.position + Vec3(2, 0, 2);
    drawCylinder(riserPos, riserPos + Vec3(0, bldg.height, 0), 0.12f, Color(0.4f, 0.6f, 0.9f));
    
    Vec3 dropPos = bldg.position + Vec3(BUILDING_FOOTPRINT - 2, bldg.height, BUILDING_FOOTPRINT - 2);
    drawCylinder(dropPos, dropPos + Vec3(0, -bldg.height - 1.5f, 0), 0.25f, Color(1.0f, 0.5f, 0.0f));
    
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        glColor3f(0.2f, 0.8f, 0.2f);
        
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
    
    ss << "ROS2 HUMBLE SCADA - REAL-TIME SENSOR DATA FROM ESP NODES";
    drawText(10, windowHeight - 25, ss.str());
    ss.str("");
    
    ss << "Buildings: " << city.buildings.size() << " | Clusters: " << city.clusters.size() 
       << " | Zones: " << city.zones.size() << " | Pipes: " << city.pipes.size() << " | Sensors: " << city.sensors.size();
    drawText(10, windowHeight - 50, ss.str());
    ss.str("");
    
    // Time and day information
    ss << "Time: " << std::fixed << std::setprecision(1) << dayNight.timeOfDay << "h"
       << " | Day: " << dayNight.getDayType()
       << " | Demand Factor: " << std::setprecision(2) << dayNight.getWaterDemandFactor();
    drawText(10, windowHeight - 75, ss.str(), Color(0.8f, 0.8f, 0.2f));
    ss.str("");
    
    // ROS2 status
    if (ros2_initialized) {
        int active = city.getActiveSensors();
        int stale = city.getStaleSensors();
        float update_rate = city.sensors.size() > 0 ? (100.0f * active / city.sensors.size()) : 0;
        
        ss << "✓ ROS2 ACTIVE: " << active << "/" << city.sensors.size() 
           << " sensors updated (" << std::fixed << std::setprecision(1) << update_rate << "%)";
        Color status_color;
        if (update_rate > 80) status_color = Color(0.2f, 0.9f, 0.2f);
        else if (update_rate > 50) status_color = Color(0.9f, 0.9f, 0.2f);
        else status_color = Color(0.9f, 0.2f, 0.2f);
        drawText(10, windowHeight - 100, ss.str(), status_color);
    } else {
        drawText(10, windowHeight - 100, "✗ ROS2 NOT INITIALIZED - Using default values", Color(1.0f, 0.2f, 0.2f));
    }
    ss.str("");
    
    // Reservoir status
    ss << "RESERVOIR: " << std::fixed << std::setprecision(1) 
       << city.reservoir.getLevelPercent() << "% (" << city.reservoir.getTrendString() << ")"
       << " | Pump: " << std::setprecision(2) << city.reservoir.availablePumpCapacity << " m³/s"
       << " | Margin: " << std::setprecision(2) << city.reservoir.supplyMargin << " m³/s";
    drawText(10, windowHeight - 125, ss.str());
    ss.str("");
    
    // Water consumption
    float totalDemand = city.calculateTotalDemand();
    
    ss << "CITY WATER: " << std::fixed << std::setprecision(2) << (totalDemand * 1000.0f) 
       << " L/s | Total consumed: " << std::setprecision(0) << city.totalWaterConsumed << " m³";
    drawText(10, windowHeight - 150, ss.str());
    ss.str("");
    
    // UDP broadcast status
    ss << "UDP BROADCAST: " << UDP_BROADCAST_PORT << " @ " << (1.0f/UDP_BROADCAST_INTERVAL) << "Hz"
       << " | RL Steps: " << city.episodeStep;
    drawText(10, windowHeight - 175, ss.str(), Color(0.6f, 0.8f, 1.0f));
    ss.str("");
    
    // Zone status summary
    if (!city.zones.empty()) {
        int leakZones = 0;
        int pressureViolations = 0;
        for (const auto& zone : city.zones) {
            if (zone.leakFlag) leakZones++;
            if (zone.pressureViolation) pressureViolations++;
        }
        
        ss << "ZONES: " << city.zones.size() 
           << " | Leaks: " << leakZones 
           << " | Pressure Violations: " << pressureViolations;
        drawText(10, windowHeight - 200, ss.str(), 
                (leakZones > 0 || pressureViolations > 0) ? Color(1.0f, 0.3f, 0.3f) : Color(0.3f, 0.9f, 0.3f));
        ss.str("");
    }
    
    // ESP topics being monitored
    drawText(10, windowHeight - 225, "TOPICS: /esp1/sensors, /esp2/sensors, /esp3/sensors, /esp4/sensors", Color(0.6f, 0.8f, 1.0f));
    
    // Controls
    drawText(10, 360, "CONTROLS:");
    drawText(10, 340, "B: Buildings | W: Water | S: Sewage | N: Sensors | L: Leaks | T: Labels");
    drawText(10, 320, "X: Transparent | G: Ground | P: Particles | H: Highlight | R: Reset | M: Sun/Moon");
    drawText(10, 300, "+/-: Buildings | Q: Quit | K: Load leaks | 1-9: Test sensor | 0: Broadcast test");
    
    // Day-night info
    drawText(10, 270, "DAY-NIGHT CYCLE:");
    drawText(10, 250, "• Time affects water demand patterns");
    drawText(10, 230, "• Weekend usage is 20% higher");
    drawText(10, 210, "• Morning (7h) and Evening (19h) peaks");
    
    // RL info
    drawText(10, 180, "RL STATE BROADCAST:");
    drawText(10, 160, "• 10Hz UDP broadcast to port " + std::to_string(UDP_BROADCAST_PORT));
    drawText(10, 140, "• Includes reservoir, zone, valve, and anomaly states");
    drawText(10, 120, "• Used for reinforcement learning training");
    
    if (transparentBuildings) {
        drawText(10, 100, ">>> BUILDINGS TRANSPARENT <<<", Color(0.2f, 1.0f, 0.2f));
    }
    
    if (showWaterParticles) {
        drawText(10, 80, ">>> WATER PARTICLES ENABLED <<<", Color(0.2f, 0.8f, 1.0f));
    }
}

void display() {
    std::lock_guard<std::mutex> lock(city_mutex);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    // Set background color (dark, no sky gradient)
    glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
    
    float autoCamDistance = std::max(200.0f, city.cityExtent * 2.5f);
    float actualDistance = camDistance == 200.0f ? autoCamDistance : camDistance;
    
    float camX = actualDistance * cos(camElevation * M_PI / 180.0f) * sin(camAngle * M_PI / 180.0f);
    float camY = actualDistance * sin(camElevation * M_PI / 180.0f);
    float camZ = actualDistance * cos(camElevation * M_PI / 180.0f) * cos(camAngle * M_PI / 180.0f);
    
    gluLookAt(camX, camY, camZ, 0, 10, 0, 0, 1, 0);
    
    // Setup lighting based on time of day
    setupLighting();
    
    // Draw sun/moon
    if (showSunMoon) {
        drawSunMoon();
    }
    
    if (showGround) {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Ground color - DARK GRAY/BLACK, NOT BLUE
        float groundBrightness = dayNight.ambientLight;
        glColor4f(0.15f * groundBrightness,  // Much darker
                  0.15f * groundBrightness, 
                  0.15f * groundBrightness,  // Very little blue
                  0.8f);                     // More opaque
                  
        float groundSize = city.cityExtent * 2.5f;
        glBegin(GL_QUADS);
        glVertex3f(-groundSize, 0, -groundSize);
        glVertex3f(groundSize, 0, -groundSize);
        glVertex3f(groundSize, 0, groundSize);
        glVertex3f(-groundSize, 0, groundSize);
        glEnd();
        glDisable(GL_BLEND);
        
        // Grid lines - also darker
        glColor3f(0.3f * groundBrightness, 
                  0.3f * groundBrightness, 
                  0.3f * groundBrightness);
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
    
    city.drawReservoirWithParticles();
    drawSewageTreatmentPlant(city.stpPos);
    
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
        
        // Show zone info if highlighted
        if (highlightedCluster < (int)city.zones.size()) {
            const Zone& zone = city.zones[highlightedCluster];
            glRasterPos3f(c.centerPos.x, 15, c.centerPos.z);
            std::stringstream ss;
            ss << "Zone " << zone.id << ": P=" << (int)zone.avgPressure << "kPa F=" 
               << std::setprecision(2) << zone.totalFlow << "m³/s";
            for (char ch : ss.str()) {
                glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ch);
            }
        }
        glEnable(GL_LIGHTING);
    }
    
    if (showBuildings) {
        for (const auto& bldg : city.buildings) {
            if (highlightedCluster < 0 || bldg.clusterID == highlightedCluster) {
                drawBuilding(bldg);
            }
        }
    }
    
    for (const auto& pipe : city.pipes) {
        bool isWater = pipe.type < SEWAGE_LATERAL;
        if ((isWater && showWaterNetwork) || (!isWater && showSewageNetwork)) {
            drawPipe(pipe);
        }
    }
    
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
    static int frame_count = 0;
    frame_count++;
    
    // Throttle updates to ~60 FPS
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    
    // Cap dt to avoid large jumps
    if (dt > 0.1f) dt = 0.016f;
    
    city.updateSimulation(dt);
    
    jsonExportCounter++;
    if (jsonExportCounter >= 60) {
        std::lock_guard<std::mutex> lock(city_mutex);
        city.exportSensorJSON("sensor.json");
        city.exportLeakJSON("leak.json");
        jsonExportCounter = 0;
    }
    
    glutPostRedisplay();
}

void cleanup() {
    simulation_running = false;
    
    // Give threads time to stop
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Proper shutdown order
    city.shutdownROS2();
    stopUDPBroadcast();
    
    std::cout << "\nCleanup complete. Exiting...\n";
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
            std::cout << "Ground visibility: " << (showGround ? "ON" : "OFF") << "\n";
            break;
        case 'x': case 'X':
            transparentBuildings = !transparentBuildings;
            std::cout << "Buildings transparency: " << (transparentBuildings ? "ON" : "OFF") << "\n";
            break;
        case 'm': case 'M':
            showSunMoon = !showSunMoon;
            std::cout << "Sun/Moon: " << (showSunMoon ? "ON" : "OFF") << "\n";
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
            int sensorID = key - '1';
            if (sensorID < (int)city.sensors.size()) {
                float newValve = (city.sensors[sensorID].valveState > 50) ? 20.0f : 100.0f;
                city.setValveState(sensorID, newValve);
                std::cout << "TEST: Set Sensor " << sensorID << " valve to " << newValve << "%\n";
            }
            break;
        }
        case '0':
            // Test broadcast
            city.broadcastRLState();
            std::cout << "Test broadcast sent\n";
            break;
        case 'q': case 'Q': case 27:
            cleanup(); 
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
        camElevation -= (y - lastMouseY) * 0.5f;
        if (camElevation > 180.0f) camElevation -= 360.0f;
        if (camElevation < -180.0f) camElevation += 360.0f;
        lastMouseX = x;
        lastMouseY = y;
    }
}

void signal_handler(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        std::cout << "\n\nReceived shutdown signal (Ctrl+C)" << std::endl;
        g_shutdown_requested = true;
        
        // Don't exit immediately, let the main loop handle it
        static int signal_count = 0;
        signal_count++;
        
        if (signal_count >= 3) {
            std::cout << "Force exiting..." << std::endl;
            exit(1);
        }
    }
}

void printInstructions() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   ROS2 HUMBLE SCADA SIMULATION - REAL-TIME ESP SENSOR DATA  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "ENHANCED FEATURES:\n";
    std::cout << "  1. Realistic physics with Darcy-Weisbach pressure drops\n";
    std::cout << "  2. Day-night cycle affecting water demand patterns\n";
    std::cout << "  3. Zone-based management with statistics\n";
    std::cout << "  4. UDP broadcast at 10Hz for RL training\n";
    std::cout << "  5. Realistic building water consumption (morning/evening peaks)\n";
    std::cout << "  6. Weekend vs weekday demand patterns\n\n";
    
    std::cout << "RL STATE BROADCAST (10Hz UDP on port " << UDP_BROADCAST_PORT << "):\n";
    std::cout << "  - Reservoir state (level, trend, pump capacity, supply margin)\n";
    std::cout << "  - Zone pressures, flows, variances, anomalies\n";
    std::cout << "  - Valve states with hysteresis control\n";
    std::cout << "  - Time encoding (sin/cos) for cyclic patterns\n";
    std::cout << "  - Leak and overflow detection flags\n\n";
    
    std::cout << "DATA FLOW:\n";
    std::cout << "  1. Simulation creates sensors with IDs (0 to N)\n";
    std::cout << "  2. ESP nodes publish sensor data with matching IDs\n";
    std::cout << "  3. Simulation subscribes to ROS2 topics:\n";
    std::cout << "     - /esp1/sensors\n";
    std::cout << "     - /esp2/sensors\n";
    std::cout << "     - /esp3/sensors\n";
    std::cout << "     - /esp4/sensors\n";
    std::cout << "  4. When message arrives, updates sensor by ID\n";
    std::cout << "  5. Visual display updates in real-time\n";
    std::cout << "  6. RL state broadcast via UDP at 10Hz\n\n";
    
    std::cout << "EXAMPLE ESP MESSAGE:\n";
    std::cout << "  {\n";
    std::cout << "    \"esp_id\": 1,\n";
    std::cout << "    \"sensors\": [\n";
    std::cout << "      {\n";
    std::cout << "        \"sensor_id\": 0,        ← MATCHES SIMULATION SENSOR ID\n";
    std::cout << "        \"pressure_bar\": 1.03,\n";
    std::cout << "        \"level_pct\": 90.0,\n";
    std::cout << "        \"valve\": 1\n";
    std::cout << "      }\n";
    std::cout << "    ]\n";
    std::cout << "  }\n\n";
    
    std::cout << "SENSOR IDS CREATED:\n";
    std::cout << "  Water sensors: ID 0 to N\n";
    std::cout << "  Sewage sensors: ID N+1 to M\n";
    std::cout << "  (See init_sensor.json for complete list)\n\n";
    
    std::cout << "COMPILATION:\n";
    std::cout << "  g++ -o sim sim.cpp -lGL -lGLU -lglut -lm -O2 -std=c++17 \\\n";
    std::cout << "      -I/opt/ros/humble/include \\\n";
    std::cout << "      -L/opt/ros/humble/lib \\\n";
    std::cout << "      -lrclcpp -lrcutils -lrmw_cyclonedds_cpp -ljsoncpp -lpthread\n\n";
    
    std::cout << "RUNNING:\n";
    std::cout << "  1. Source ROS2: source /opt/ros/humble/setup.bash\n";
    std::cout << "  2. Run: ./sim\n";
    std::cout << "  3. ESP nodes should publish to topics\n";
    std::cout << "  4. RL agent can receive UDP broadcasts on port " << UDP_BROADCAST_PORT << "\n";
    std::cout << "  5. Watch sensor values update in real-time!\n\n";
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    srand(time(nullptr));
    
    printInstructions();
    
    std::cout << "Generating city with " << currentBuildingCount << " buildings...\n";
    city.generateCity(currentBuildingCount);
    city.generateInitSensorJSON("/home/arka/aqua_sentinel/simulator/init_sensor.json");
    
    // Initialize GLUT FIRST (MUST BE FIRST!)
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("ROS2 Humble SCADA - Real-time Sensor Data from ESP Nodes");
    
    // Setup OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);
    glEnable(GL_RESCALE_NORMAL);
    
    // Set up GLUT callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    
    // Initialize systems after GLUT is set up
    std::cout << "\nInitializing systems...\n";
    
    // Start UDP broadcast
    std::cout << "Starting UDP broadcast on port " << UDP_BROADCAST_PORT << "...\n";
    startUDPBroadcast();
    
    // Initialize ROS2
    std::cout << "Initializing ROS2 Humble...\n";
    city.initializeROS2();
    
    if (!ros2_initialized) {
        std::cout << "WARNING: ROS2 initialization failed.\n";
        std::cout << "Make sure ROS2 Humble is installed and sourced.\n";
        std::cout << "Continuing with default sensor values...\n";
    }
    
    std::cout << "\nSimulation running...\n";
    std::cout << "ROS2 Status: " << (ros2_initialized ? "ACTIVE" : "INACTIVE") << "\n";
    std::cout << "UDP Broadcast: ACTIVE on port " << UDP_BROADCAST_PORT << " @ 10Hz\n";
    std::cout << "Sensor IDs: 0 to " << (city.sensors.size() - 1) << "\n";
    std::cout << "Zones: " << city.zones.size() << "\n";
    std::cout << "Press 'Q' to quit\n";
    std::cout << "Press 'M' to toggle sun/moon visibility\n";
    std::cout << "Press 'G' to toggle ground visibility\n\n";
    
    // Register cleanup with atexit
    atexit(cleanup);
    
    // Enter main loop
    glutMainLoop();
    
    return 0;
}