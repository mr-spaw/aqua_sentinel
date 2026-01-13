//g++ -o sim sim.cpp -lGL -lGLU -lglut -lm -O2 -std=c++17 -I/opt/ros/humble/include -L/opt/ros/humble/lib -lrclcpp -lrcutils -lrmw_cyclonedds_cpp -ljsoncpp

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
#include <csignal>
#include <atomic>


// ============================================================================
// ROS2 HUMBLE INTEGRATION
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <json/json.h>

// ROS2 globals
std::shared_ptr<rclcpp::Node> ros2_node = nullptr;
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
const float DEFAULT_PRESSURE = 15.0f;
const float DEFAULT_WATER_LEVEL = 70.0f;

const float WATER_CONSUMPTION_BASE = 0.0005f;
const float DAILY_VARIATION = 0.3f;
const float PARTICLE_SPEED = 2.0f;
const int PARTICLES_PER_PIPE = 20;
const float PARTICLE_SIZE = 0.15f;

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

class CityNetwork;
class ROS2SensorSubscriber;

// ============================================================================
// WATER PARTICLE STRUCTURE
// ============================================================================

struct WaterParticle {
    Vec3 position;
    Vec3 direction;
    float speed;
    float life;
    Color color;
    int pipeID;
    
    WaterParticle(Vec3 pos, Vec3 dir, float spd, Color col, int pid)
        : position(pos), direction(dir.normalized()), speed(spd), life(1.0f), color(col), pipeID(pid) {}
    
    void update(float dt) {
        position = position + direction * (speed * dt);
        life -= dt * 0.2f;
        if (life < 0) life = 0;
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
    
    float valveState;
    float pressure;
    float waterLevel;
    
    bool active;
    int connectedPipeID;
    
    // Last update source tracking
    int last_esp_id;
    time_t last_update_time;
    
    Sensor(int id, std::string name, SensorType type, Vec3 pos)
        : id(id), name(name), type(type), position(pos),
          valveState(DEFAULT_VALVE_STATE),
          pressure(DEFAULT_PRESSURE),
          waterLevel(DEFAULT_WATER_LEVEL),
          active(true), connectedPipeID(-1),
          last_esp_id(-1), last_update_time(0) {}
    
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
        std::lock_guard<std::mutex> lock(sensor_mutex);
        valveState = valve;
        pressure = press;
        waterLevel = level;
        last_esp_id = esp_id;
        last_update_time = time(nullptr);
    }
    
    bool isStale() const {
        return (time(nullptr) - last_update_time) > 5; // 5 seconds stale
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
    
    bool hasLeak;
    float leakRate;
    float flowRate;
    
    std::vector<WaterParticle> particles;
    
    Pipe(int id, Vec3 s, Vec3 e, PipeType t, float diam)
        : id(id), start(s), end(e), type(t), diameter(diam),
          hasLeak(false), leakRate(0.0f), flowRate(0.0f) {
        length = (end - start).length();
    }
    
    Color getColor() const {
        if (type >= SEWAGE_LATERAL) {
            return Color(1.0f, 0.6f, 0.0f);
        } else {
            float intensity = std::min(1.0f, flowRate / 50.0f);
            return Color(0.2f, 0.5f + intensity * 0.4f, 0.9f);
        }
    }
    
    void updateParticles(float dt, Color particleColor = Color(0.3f, 0.6f, 1.0f, 0.8f)) {
        for (auto it = particles.begin(); it != particles.end();) {
            it->update(dt);
            if (it->life <= 0) {
                it = particles.erase(it);
            } else {
                ++it;
            }
        }
        
        if (type < SEWAGE_LATERAL && flowRate > 0.1f && particles.size() < PARTICLES_PER_PIPE) {
            float t = static_cast<float>(rand()) / RAND_MAX;
            Vec3 spawnPos = start + (end - start) * t * 0.1f;
            Vec3 direction = (end - start).normalized();
            float speed = PARTICLE_SPEED * (0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX);
            
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
    
    int waterSensorID;
    int sewageSensorID;
    int servicePipeID;
    int sewerPipeID;
    
    float dailyWaterUsage;
    float currentWaterFlow;
    float totalWaterConsumed;
    Color waterColor;
    
    static float* citySimulationTime;
    
    Building(int id, Vec3 pos, int floors, int cluster)
        : id(id), clusterID(cluster), position(pos), numFloors(floors),
          height(floors * FLOOR_HEIGHT),
          waterSensorID(-1), sewageSensorID(-1),
          servicePipeID(-1), sewerPipeID(-1),
          dailyWaterUsage(0), currentWaterFlow(0), totalWaterConsumed(0) {
        dailyWaterUsage = 0.8f + (static_cast<float>(rand()) / RAND_MAX) * 0.4f;
        waterColor = Color(
            (rand() % 100) / 200.0f + 0.5f,
            (rand() % 100) / 200.0f + 0.5f,
            (rand() % 100) / 200.0f + 0.8f,
            0.7f
        );
    }
    
    void updateWaterUsage(float dt) {
        if (!citySimulationTime) return;
        
        float timeOfDay = fmod(*citySimulationTime, 86400.0f) / 86400.0f;
        float dailyPattern = 0.7f + 0.6f * sinf(timeOfDay * 2 * M_PI - M_PI/2);
        float randomFactor = 0.9f + (static_cast<float>(rand()) / RAND_MAX) * 0.2f;
        
        currentWaterFlow = WATER_CONSUMPTION_BASE * dailyWaterUsage * dailyPattern * randomFactor;
        totalWaterConsumed += currentWaterFlow * dt;
    }
    
    float getDailyConsumption() const {
        return currentWaterFlow * 86400.0f;
    }
};

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
// ROS2 SENSOR SUBSCRIBER CLASS
// ============================================================================

//class CityNetwork;  // Forward declaration

// ============================================================================
// ROS2 SENSOR SUBSCRIBER CLASS - THREAD-SAFE VERSION
// ============================================================================

// ============================================================================
// ROS2 SENSOR SUBSCRIBER CLASS - SIMPLIFIED VERSION
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
    
    // Declaration only - definition will come AFTER CityNetwork is fully defined
    void processESPData(const std::string& json_data, int esp_id);
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
    float cityExtent;
    
    int reservoirWaterSensorID;
    int stpSewerSensorID;
    
    float simulationTime;
    float reservoirVolume;
    float reservoirCapacity;
    float reservoirLevel;
    
    float totalWaterSupplied;
    float totalWaterConsumed;
    float totalLeakWater;
    
    // ROS2 subscriber
    std::shared_ptr<ROS2SensorSubscriber> ros2_subscriber;
    
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
    void updateSensorFromROS2(int sensorID, float valve, float pressure, float level, int esp_id);
    void updateSimulation(float dt);
    void generateInitSensorJSON(const char* filename);
    void exportSensorJSON(const char* filename);
    void exportLeakJSON(const char* filename);
    void exportSystemStateJSON(const char* filename);
    
    void updateBuildingWaterUsage(float dt);
    void updateReservoir(float dt);
    void generateReservoirParticles(float dt);
    void drawReservoirWithParticles() const;
    
    void initializeROS2();
    void shutdownROS2();
    
    // Sensor statistics
    int getActiveSensors() const;
    int getStaleSensors() const;
    
private:
    void drawReservoirStructure(Vec3 pos, float waterLevel) const;
};

// ============================================================================
// ROS2 INITIALIZATION
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
        // Cast to ROS2SensorSubscriber and call shutdown if possible
        // Or simply set a flag to stop processing
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
    
    if (sensorID >= 0 && sensorID < (int)sensors.size()) {
        // Modified Sensor::updateFromROS2() - removed internal locking
        sensors[sensorID].valveState = valve;
        sensors[sensorID].pressure = pressure;
        sensors[sensorID].waterLevel = level;
        sensors[sensorID].last_esp_id = esp_id;
        sensors[sensorID].last_update_time = time(nullptr);
        
        // Log first few updates for debugging
        static int first_updates_logged = 0;
        if (first_updates_logged < 20) {  // Increased to 20 to see more ESPs
            std::cout << "ROS2 Update: Sensor ID=" << sensorID 
                      << " from ESP" << esp_id 
                      << " - Valve:" << valve << "%"
                      << " Pressure:" << pressure << "kPa"
                      << " Level:" << level << "%" << std::endl;
            first_updates_logged++;
        }
    } else {
        static int invalid_sensor_warnings = 0;
        if (invalid_sensor_warnings < 20) {  // Increased limit
            std::cerr << "WARNING: Received update for invalid sensor ID: " << sensorID 
                      << " from ESP" << esp_id 
                      << " (valid range: 0-" << (sensors.size() - 1) << ")" << std::endl;
            invalid_sensor_warnings++;
            
            // Debug: Print sensor distribution
            if (invalid_sensor_warnings == 1) {
                std::cout << "\n=== SENSOR DISTRIBUTION ===" << std::endl;
                std::cout << "ESP1 handles sensors: 0-110 (water sensors)" << std::endl;
                std::cout << "ESP2 handles sensors: 111-221 (water sensors)" << std::endl;
                std::cout << "ESP3 handles sensors: 222-332 (sewage sensors)" << std::endl;
                std::cout << "ESP4 handles sensors: 333-441 (sewage sensors)" << std::endl;
                std::cout << "Total sensors in simulation: 0-" << (sensors.size() - 1) << std::endl;
            }
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

// ============================================================================
// CITY GENERATION
// ============================================================================

void CityNetwork::updateDynamicPositions() {
    float maxDist = 0;
    for (const auto& cluster : clusters) {
        float dist = sqrt(cluster.centerPos.x * cluster.centerPos.x + 
                         cluster.centerPos.z * cluster.centerPos.z);
        maxDist = std::max(maxDist, dist);
    }
    cityExtent = maxDist + CLUSTER_SPACING;
    
    reservoirPos = Vec3(-cityExtent - 80, 20, -cityExtent - 80);
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
    file << "    \"reservoir_level_m\": " << std::setprecision(2) << reservoirLevel << ",\n";
    file << "    \"reservoir_volume_m3\": " << std::setprecision(2) << reservoirVolume << ",\n";

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

    file << "    \"leaking_pipe_ids\": [";
    for (size_t i = 0; i < leakingPipeIDs.size(); ++i) {
        file << leakingPipeIDs[i];
        if (i < leakingPipeIDs.size() - 1) file << ", ";
    }
    file << "],\n";

    // Sensor statistics
    int active_sensors = getActiveSensors();
    int stale_sensors = getStaleSensors();
    
    file << "    \"ros2_sensor_stats\": {\n";
    file << "      \"total_sensors\": " << sensors.size() << ",\n";
    file << "      \"active_sensors\": " << active_sensors << ",\n";
    file << "      \"stale_sensors\": " << stale_sensors << ",\n";
    file << "      \"update_rate\": " << std::setprecision(1) 
         << (active_sensors > 0 ? 100.0f * active_sensors / sensors.size() : 0.0f) << "\n";
    file << "    }\n";

    file << "  },\n";
    
    file << "  \"building_water_usage\": {\n";
    file << "    \"total_buildings\": " << buildings.size() << ",\n";
    file << "    \"total_consumption_m3\": " << std::setprecision(2) << totalWaterConsumed << ",\n";
    file << "    \"current_demand_m3s\": " << std::setprecision(4) << (total_fresh_flow / 1000.0f) << ",\n";
    file << "    \"buildings\": [\n";
    
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
    
    reservoirVolume = reservoirCapacity * 0.7f;
    reservoirLevel = (reservoirVolume / reservoirCapacity) * 30.0f;
    
    std::cout << "Created " << buildings.size() << " buildings in " << clusters.size() << " clusters\n";
    std::cout << "City extent: " << cityExtent << "m\n";
    
    generateWaterNetwork();
    generateSewageNetwork();
    createSensors();
    
    std::cout << "Total pipes: " << pipes.size() << "\n";
    std::cout << "Total sensors: " << sensors.size() << "\n";
    std::cout << "=== CITY GENERATION COMPLETE ===\n\n";
}

void CityNetwork::generateWaterNetwork() {
    int pipeID = 0;
    
    Vec3 trunkStart = reservoirPos + Vec3(0, -10, 0);
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
        Vec3 collectorPoint = cluster.centerPos + Vec3(0, -2.5f, CLUSTER_SPACING * 0.7f);
        
        for (int bid : cluster.buildingIDs) {
            Building& bldg = buildings[bid];
            Vec3 lateralStart = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.75f, 1.0f, BUILDING_FOOTPRINT * 0.75f);
            Vec3 lateralEnd = collectorPoint;
            lateralEnd.y = -2.0f;
            
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
    
    Vec3 interceptorStart(0, -4.0f, stpPos.z - 30.0f);
    Vec3 interceptorEnd = stpPos + Vec3(0, 0.5f, -15.0f);
    pipes.push_back(Pipe(
        pipeID++,
        interceptorStart,
        interceptorEnd,
        SEWAGE_INTERCEPTOR,
        TRUNK_DIAMETER
    ));
}

void CityNetwork::createSensors() {
    int sensorID = 0;
    int waterSensorCount = 0;
    int sewageSensorCount = 0;
    
    // Reservoir water sensor
    reservoirWaterSensorID = sensorID;
    Vec3 resSensorPos = reservoirPos + Vec3(0, -5, 0);
    sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, resSensorPos));
    sensors.back().connectedPipeID = 0;
    std::cout << "  Created " << sensors.back().name << " ID: " << reservoirWaterSensorID << "\n";
    
    // Cluster water sensors
    for (auto& cluster : clusters) {
        cluster.waterSensorID = sensorID;
        Vec3 clusterWaterPos = cluster.centerPos + Vec3(-3, 2.5, 0);
        sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, clusterWaterPos));
        sensors.back().connectedPipeID = cluster.secondaryMainID;
    }
    
    // Building water sensors
    for (auto& bldg : buildings) {
        bldg.waterSensorID = sensorID;
        Vec3 bldgWaterPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.15f, 1.8f, BUILDING_FOOTPRINT * 0.25f);
        sensors.push_back(Sensor(sensorID++, "sensor_f" + std::to_string(waterSensorCount++), WATER_SENSOR, bldgWaterPos));
        sensors.back().connectedPipeID = bldg.servicePipeID;
    }
    
    // Building sewage sensors
    for (auto& bldg : buildings) {
        bldg.sewageSensorID = sensorID;
        Vec3 bldgSewerPos = bldg.position + Vec3(BUILDING_FOOTPRINT * 0.85f, 1.8f, BUILDING_FOOTPRINT * 0.75f);
        sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, bldgSewerPos));
        sensors.back().connectedPipeID = bldg.sewerPipeID;
    }
    
    // Cluster sewage sensors
    for (auto& cluster : clusters) {
        cluster.sewageSensorID = sensorID;
        Vec3 clusterSewerPos = cluster.centerPos + Vec3(3, -0.5, CLUSTER_SPACING * 0.6f);
        sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, clusterSewerPos));
        sensors.back().connectedPipeID = cluster.sewageCollectorID;
    }
    
    // STP sewage sensor
    stpSewerSensorID = sensorID;
    Vec3 stpSensorPos = stpPos + Vec3(0, 3, -15);
    sensors.push_back(Sensor(sensorID++, "sensor_s" + std::to_string(sewageSensorCount++), SEWAGE_SENSOR, stpSensorPos));
    
    std::cout << "\n=== SENSOR IDS CREATED ===\n";
    std::cout << "Water Sensors: ID 0 to " << (waterSensorCount - 1) << " (" << waterSensorCount << " sensors)\n";
    std::cout << "Sewage Sensors: ID " << waterSensorCount << " to " << (sensors.size() - 1) 
              << " (" << sewageSensorCount << " sensors)\n";
    std::cout << "Total Sensors: " << sensors.size() << " (IDs 0 to " << (sensors.size() - 1) << ")\n\n";
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
            }
        }
    }
    file.close();
}

void CityNetwork::updateBuildingWaterUsage(float dt) {
    totalWaterConsumed = 0;
    
    for (auto& building : buildings) {
        building.updateWaterUsage(dt);
        totalWaterConsumed += building.currentWaterFlow * dt;
    }
}

void CityNetwork::updateReservoir(float dt) {
    float totalDemand = 0;
    for (const auto& building : buildings) {
        totalDemand += building.currentWaterFlow;
    }
    
    float totalLeaks = 0;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeaks += pipe.leakRate / 1000.0f;
        }
    }
    
    float outflow = (totalDemand + totalLeaks) * dt;
    reservoirVolume -= outflow;
    
    if (reservoirVolume < 0) reservoirVolume = 0;
    if (reservoirVolume > reservoirCapacity) reservoirVolume = reservoirCapacity;
    
    reservoirLevel = (reservoirVolume / reservoirCapacity) * 30.0f;
    
    totalWaterSupplied += outflow;
    totalLeakWater += totalLeaks * dt;
    
    if (reservoirWaterSensorID >= 0 && reservoirWaterSensorID < (int)sensors.size()) {
        sensors[reservoirWaterSensorID].waterLevel = (reservoirVolume / reservoirCapacity) * 100.0f;
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
    
    if (reservoirVolume > 0 && reservoirParticles.size() < 100) {
        float radius = 10.0f;
        float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
        float dist = static_cast<float>(rand()) / RAND_MAX * radius;
        
        Vec3 spawnPos = reservoirPos + Vec3(
            cos(angle) * dist,
            -5 + static_cast<float>(rand()) / RAND_MAX * 5,
            sin(angle) * dist
        );
        
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
    
    updateBuildingWaterUsage(dt);
    updateReservoir(dt);
    generateReservoirParticles(dt);
    
    float totalBuildingDemand = 0;
    for (const auto& building : buildings) {
        totalBuildingDemand += building.currentWaterFlow;
    }
    
    for (auto& pipe : pipes) {
        float valveFactor = 1.0f;
        for (const auto& sensor : sensors) {
            if (sensor.connectedPipeID == pipe.id) {
                valveFactor = sensor.valveState / 100.0f;
                break;
            }
        }
        
        if (pipe.type < SEWAGE_LATERAL) {
            if (pipe.type == TRUNK_MAIN) {
                pipe.flowRate = totalBuildingDemand * 1000.0f * valveFactor;
            } else if (pipe.type == SERVICE_PIPE) {
                for (const auto& building : buildings) {
                    if (building.servicePipeID == pipe.id) {
                        pipe.flowRate = building.currentWaterFlow * 1000.0f * valveFactor;
                        break;
                    }
                }
            } else {
                pipe.flowRate = 10.0f * valveFactor;
            }
            
            Color particleColor(0.3f, 0.6f, 1.0f, 0.8f);
            for (const auto& building : buildings) {
                if (building.servicePipeID == pipe.id) {
                    particleColor = building.waterColor;
                    break;
                }
            }
            pipe.updateParticles(dt, particleColor);
            
        } else {
            pipe.flowRate = 5.0f;
        }
    }
}

void CityNetwork::drawReservoirStructure(Vec3 pos, float waterLevel) const {
    float tankRadius = 25.0f;
    float tankHeight = 30.0f;
    float wallThickness = 1.0f;
    
    glColor3f(0.7f, 0.7f, 0.7f);
    
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight/2, pos.z);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, tankRadius, tankRadius, tankHeight, 64, 8);
    gluDeleteQuadric(quad);
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(pos.x, pos.y + tankHeight/2, pos.z);
    glRotatef(-90, 1, 0, 0);
    GLUquadric* disk = gluNewQuadric();
    gluDisk(disk, 0, tankRadius, 64, 8);
    gluDeleteQuadric(disk);
    glPopMatrix();
    
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
        
        glColor4f(0.15f, 0.4f, 0.9f, 0.5f);
        glPushMatrix();
        glTranslatef(pos.x, pos.y - tankHeight/2 + waterHeight/2, pos.z);
        gluCylinder(quad, tankRadius - wallThickness, tankRadius - wallThickness, waterHeight, 64, 8);
        glPopMatrix();
        
        glDisable(GL_BLEND);
    }
    
    glColor3f(0.5f, 0.5f, 0.5f);
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight - 5, pos.z);
    glScalef(40.0f, 10.0f, 40.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
    
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
    drawReservoirStructure(reservoirPos, reservoirLevel);
    
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
// ROS2SensorSubscriber::processESPData IMPLEMENTATION
// (Must be AFTER CityNetwork is fully defined)
// ============================================================================

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
bool showLeaks = true;
bool showSensorLabels = false;
bool showGround = true;
bool transparentBuildings = false;
bool showWaterParticles = true;
bool showReservoirParticles = true;
int highlightedCluster = -1;
int currentBuildingCount = 200;

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
    
    // Pulse effect for active sensors
    float pulse = 0.75f + 0.35f * sinf(city.simulationTime * 3.0f);
    
    if (sensor.active) {
        // Outer glow
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        
        if (sensor.isStale()) {
            // Stale sensors have dimmer, red-tinted glow
            glColor4f(col.r * 0.5f, col.g * 0.3f, col.b * 0.3f, 0.2f);
        } else {
            // Active sensors have bright glow
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
    
    // Sensor body
    float sensorSize = 2.2f;
    if (sensor.isStale()) {
        // Stale sensors are darker
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
            // Stale sensors show in gray
            glColor3f(0.6f, 0.6f, 0.6f);
        } else if (sensor.type == WATER_SENSOR) {
            glColor3f(0.3f, 0.85f, 1.0f);
        } else {
            glColor3f(1.0f, 0.7f, 0.2f);
        }
        
        // Show sensor ID and ESP source
        glRasterPos3f(sensor.position.x, sensor.position.y + 3.5f, sensor.position.z);
        std::stringstream ss;
        ss << "ID:" << sensor.id << " " << sensor.name;
        if (sensor.last_esp_id != -1) {
            ss << " [ESP" << sensor.last_esp_id << "]";
        }
        for (char c : ss.str()) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
        }
        
        // Show values
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
        
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        for (int i = 0; i < 60; i++) {
            float angle = i * M_PI * 2.0f / 60.0f + city.simulationTime * 2.0f;
            float dist = 3.5f + 2.0f * sinf(city.simulationTime * 3.0f + i);
            Vec3 p = mid + Vec3(cos(angle) * dist, sinf(angle * 3) * dist + 2, sin(angle) * dist);
            
            if (i % 2 == 0) {
                glColor3f(0.5f, 0.8f, 1.0f);
            } else {
                glColor3f(1.0f, 1.0f, 1.0f);
            }
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
        
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
       << " | Pipes: " << city.pipes.size() << " | Sensors: " << city.sensors.size();
    drawText(10, windowHeight - 50, ss.str());
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
        drawText(10, windowHeight - 75, ss.str(), status_color);
    } else {
        drawText(10, windowHeight - 75, "✗ ROS2 NOT INITIALIZED - Using default values", Color(1.0f, 0.2f, 0.2f));
    }
    ss.str("");
    
    // Leak status
    int leakCount = 0;
    for (const auto& pipe : city.pipes) {
        if (pipe.hasLeak) leakCount++;
    }
    
    if (leakCount > 0) {
        ss << "ACTIVE LEAKS: " << leakCount;
        drawText(10, windowHeight - 100, ss.str(), Color(1, 0, 0));
        ss.str("");
    } else {
        drawText(10, windowHeight - 100, "NO LEAKS DETECTED", Color(0.2f, 0.9f, 0.2f));
    }
    
    // Reservoir status
    ss << "RESERVOIR: " << std::fixed << std::setprecision(1) 
       << (city.reservoirVolume / city.reservoirCapacity * 100.0f) << "%"
       << " (" << std::setprecision(0) << city.reservoirVolume << "/" << city.reservoirCapacity << " m³)";
    drawText(10, windowHeight - 125, ss.str());
    ss.str("");
    
    // Water consumption
    float totalDemand = 0;
    for (const auto& bldg : city.buildings) {
        totalDemand += bldg.currentWaterFlow;
    }
    
    ss << "CITY WATER: " << std::fixed << std::setprecision(1) << (totalDemand * 1000.0f) 
       << " L/s | Total consumed: " << std::setprecision(0) << city.totalWaterConsumed << " m³";
    drawText(10, windowHeight - 150, ss.str());
    ss.str("");
    
    // System state JSON
    ss << "SYSTEM STATE JSON: Updates every " << SYSTEM_JSON_UPDATE_INTERVAL << "s";
    drawText(10, windowHeight - 175, ss.str(), Color(0.8f, 0.8f, 0.2f));
    
    // ESP topics being monitored
    drawText(10, windowHeight - 200, "TOPICS: /esp1/sensors, /esp2/sensors, /esp3/sensors, /esp4/sensors", Color(0.6f, 0.8f, 1.0f));
    
    // Controls
    drawText(10, 360, "CONTROLS:");
    drawText(10, 340, "B: Buildings | W: Water | S: Sewage | N: Sensors | L: Leaks | T: Labels");
    drawText(10, 320, "X: Transparent | G: Ground | P: Particles | H: Highlight | R: Reset");
    drawText(10, 300, "+/-: Buildings | Q: Quit | K: Load leaks | 1-9: Test sensor");
    
    // ROS2 info
    drawText(10, 270, "ROS2 DATA FLOW:");
    drawText(10, 250, "1. ESP nodes publish sensor data to ROS2 topics");
    drawText(10, 230, "2. Simulation subscribes and updates sensor values by ID");
    drawText(10, 210, "3. Visual display updates in real-time");
    drawText(10, 190, "4. System state exported for RL agent");
    
    if (transparentBuildings) {
        drawText(10, 170, ">>> BUILDINGS TRANSPARENT <<<", Color(0.2f, 1.0f, 0.2f));
    }
    
    if (showWaterParticles) {
        drawText(10, 150, ">>> WATER PARTICLES ENABLED <<<", Color(0.2f, 0.8f, 1.0f));
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    float autoCamDistance = std::max(200.0f, city.cityExtent * 2.5f);
    float actualDistance = camDistance == 200.0f ? autoCamDistance : camDistance;
    
    float camX = actualDistance * cos(camElevation * M_PI / 180.0f) * sin(camAngle * M_PI / 180.0f);
    float camY = actualDistance * sin(camElevation * M_PI / 180.0f);
    float camZ = actualDistance * cos(camElevation * M_PI / 180.0f) * cos(camAngle * M_PI / 180.0f);
    
    gluLookAt(camX, camY, camZ, 0, 10, 0, 0, 1, 0);
    
    if (showGround) {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.12f, 0.12f, 0.12f, 0.4f);
        float groundSize = city.cityExtent * 2.0f;
        glBegin(GL_QUADS);
        glVertex3f(-groundSize, 0, -groundSize);
        glVertex3f(groundSize, 0, -groundSize);
        glVertex3f(groundSize, 0, groundSize);
        glVertex3f(-groundSize, 0, groundSize);
        glEnd();
        glDisable(GL_BLEND);
        
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
    float dt = 0.016f;
    city.updateSimulation(dt);
    
    systemJsonUpdateTimer += dt;
    
    if (systemJsonUpdateTimer >= SYSTEM_JSON_UPDATE_INTERVAL) {
        city.exportSystemStateJSON("system_state.json");
        systemJsonUpdateTimer = 0.0f;
    }
    
    jsonExportCounter++;
    if (jsonExportCounter >= 60) {
        city.exportSensorJSON("sensor.json");
        city.exportLeakJSON("leak.json");
        jsonExportCounter = 0;
    }
    
    glutPostRedisplay();
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
                city.updateSensorFromROS2(sensorID, newValve, DEFAULT_PRESSURE, DEFAULT_WATER_LEVEL, 0);
                std::cout << "TEST: Toggled Sensor " << sensorID << " valve to " << newValve << "%\n";
            }
            break;
        }
        case 'q': case 'Q': case 27:
            std::cout << "\n=== SHUTTING DOWN SIMULATION ===" << std::endl;
            std::cout << "Stopping ROS2..." << std::endl;
            city.shutdownROS2();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::cout << "Exiting simulation." << std::endl;
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
        camElevation -= (y - lastMouseY) * 0.5f;
        if (camElevation > 180.0f) camElevation -= 360.0f;
        if (camElevation < -180.0f) camElevation += 360.0f;
        lastMouseX = x;
        lastMouseY = y;
    }
}

void printInstructions() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   ROS2 HUMBLE SCADA SIMULATION - REAL-TIME ESP SENSOR DATA  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "DATA FLOW:\n";
    std::cout << "  1. Simulation creates sensors with IDs (0 to N)\n";
    std::cout << "  2. ESP nodes publish sensor data with matching IDs\n";
    std::cout << "  3. Simulation subscribes to ROS2 topics:\n";
    std::cout << "     - /esp1/sensors\n";
    std::cout << "     - /esp2/sensors\n";
    std::cout << "     - /esp3/sensors\n";
    std::cout << "     - /esp4/sensors\n";
    std::cout << "  4. When message arrives, updates sensor by ID\n";
    std::cout << "  5. Visual display updates in real-time\n\n";
    
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
    std::cout << "      -lrclcpp -lrcutils -lrmw_cyclonedds_cpp -ljsoncpp\n\n";
    
    std::cout << "RUNNING:\n";
    std::cout << "  1. Source ROS2: source /opt/ros/humble/setup.bash\n";
    std::cout << "  2. Run: ./sim\n";
    std::cout << "  3. ESP nodes should publish to topics\n";
    std::cout << "  4. Watch sensor values update in real-time!\n\n";
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    srand(time(nullptr));
    
    printInstructions();
    
    std::cout << "Generating city with " << currentBuildingCount << " buildings...\n";
    city.generateCity(currentBuildingCount);
    city.generateInitSensorJSON("/home/arka/aqua_sentinel/simulator/init_sensor.json");
    
    std::cout << "\nInitializing ROS2 Humble...\n";
    city.initializeROS2();
    
    if (!ros2_initialized) {
        std::cout << "WARNING: ROS2 initialization failed.\n";
        std::cout << "Make sure ROS2 Humble is installed and sourced.\n";
        std::cout << "Continuing with default sensor values...\n";
    }
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("ROS2 Humble SCADA - Real-time Sensor Data from ESP Nodes");
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    GLfloat lightPos[] = {200.0f, 400.0f, 200.0f, 1.0f};
    GLfloat lightAmb[] = {0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat lightDif[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightSpec[] = {1.0f, 1.0f, 1.0f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);
    
    GLfloat lightPos2[] = {-100.0f, -200.0f, 100.0f, 1.0f};
    GLfloat lightAmb2[] = {0.3f, 0.3f, 0.35f, 1.0f};
    GLfloat lightDif2[] = {0.6f, 0.6f, 0.7f, 1.0f};
    
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos2);
    glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmb2);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDif2);
    
    glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);
    glEnable(GL_RESCALE_NORMAL);
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    
    std::cout << "\nSimulation running...\n";
    std::cout << "ROS2 Status: " << (ros2_initialized ? "ACTIVE" : "INACTIVE") << "\n";
    std::cout << "Sensor IDs: 0 to " << (city.sensors.size() - 1) << "\n";
    std::cout << "Check init_sensor.json for sensor-pipe mappings\n";
    std::cout << "Press 'T' to see sensor labels with ESP source\n";
    std::cout << "Press 'Q' to quit\n\n";
    
    glutMainLoop();
    return 0;
}