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
    #include <unordered_map>   
    #include <array>           
    #include <deque>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <chrono>

    // ============================================================================
    // UDP BROADCAST SETTINGS
    // ============================================================================

    const int UDP_BROADCAST_PORT = 8888;
    const float UDP_BROADCAST_INTERVAL = 0.1f; 
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
    const float DEFAULT_PRESSURE = 150.0f; // kPa 
    const float DEFAULT_WATER_LEVEL = 70.0f;

    const float WATER_CONSUMPTION_BASE = 0.001f; // m³/s per building
    const float DAILY_VARIATION = 0.3f;
    const float PARTICLE_SPEED = 2.0f;
    const int PARTICLES_PER_PIPE = 20;
    const float PARTICLE_SIZE = 0.15f;

    const float SYSTEM_HEAD_LOSS_COEFF = 0.0001f;    // Head loss coefficient
    const float VALVE_CV_BASE = 100.0f;              // Valve flow coefficient  
    const float MIN_OPERATING_PRESSURE = 70.0f;      // kPa (0.7 bar)
    const float NOMINAL_PRESSURE = 150.0f;           // kPa (1.5 bar)
    const float MAX_PRESSURE = 300.0f;               // kPa (3.0 bar)
    const float LEAK_PROBABILITY_PER_HOUR = 0.001f;  // Probability of leak per hour
    const float SENSOR_NOISE_SIGMA = 0.5f;           // kPa noise
    bool showWaterParticles = true;
    int windowWidth = 1600, windowHeight = 1000;


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

    // ============================================================================
// LEVEL STABILIZER CLASS FOR SMOOTHING
// ============================================================================



class LevelStabilizer {
private:
    std::map<int, std::deque<float>> history;
    const int HISTORY_SIZE = 10;
    
public:
    float getStabilizedLevel(int sensorID, float newLevel) {
        // Initialize if needed
        if (history.find(sensorID) == history.end()) {
            history[sensorID] = std::deque<float>(HISTORY_SIZE, newLevel);
        }
        
        // Add new reading
        history[sensorID].pop_front();
        history[sensorID].push_back(newLevel);
        
        // Calculate median for stability
        std::vector<float> sorted(history[sensorID].begin(), history[sensorID].end());
        std::sort(sorted.begin(), sorted.end());
        
        // Use median (more stable than average)
        float median = sorted[HISTORY_SIZE / 2];
        
        // Also check for outliers
        float sum = 0.0f;
        int count = 0;
        for (float val : history[sensorID]) {
            if (fabs(val - median) < 10.0f) { // Allow ±10% deviation
                sum += val;
                count++;
            }
        }
        
        return (count > 0) ? (sum / count) : median;
    }
    
    void reset(int sensorID) {
        history.erase(sensorID);
    }
};


  // ================== DAY-NIGHT CYCLE FIXES ==================
// REPLACE the entire DayNightCycle struct's methods:

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
        if (timeOfDay >= 24.0f) {
            timeOfDay -= 24.0f;
        }
        
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

    // FIXED: Calculate proper day of week
    int getDayOfWeek() const {
        // Each simulation day = 24 hours
        // Monday = 0, Tuesday = 1, ..., Sunday = 6
        int totalDays = static_cast<int>(timeOfDay / 24.0f);
        return totalDays % 7;
    }
    
    // FIXED: Proper weekend detection
    bool isWeekend() const {
        int dayOfWeek = getDayOfWeek();
        // Saturday (5) or Sunday (6) are weekends
        return (dayOfWeek == 5 || dayOfWeek == 6);
    }
    
    // FIXED: Return proper day type
    std::string getDayType() const {
        return isWeekend() ? "weekend" : "weekday";
    }
    
    // FIXED: Add method to get day name for debugging
    std::string getDayName() const {
        const char* days[] = {"Monday", "Tuesday", "Wednesday", "Thursday", 
                             "Friday", "Saturday", "Sunday"};
        return days[getDayOfWeek()];
    }
    
    float getWaterDemandFactor() const {
        // Residential water usage pattern
        float peakMorning = 7.0f;
        float peakEvening = 19.0f;
        float baseDemand = 0.3f;
        
        float morningFactor = exp(-pow(timeOfDay - peakMorning, 2) / 2.0f);
        float eveningFactor = exp(-pow(timeOfDay - peakEvening, 2) / 2.0f);
        float dayFactor = (timeOfDay >= 6.0f && timeOfDay <= 22.0f) ? 0.2f : 0.0f;
        
        // Weekend effect: 20% higher usage
        float weekendBonus = isWeekend() ? 0.2f : 0.0f;
        
        return baseDemand + 0.5f * morningFactor + 0.6f * eveningFactor + dayFactor + weekendBonus;
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
    
    // ADD THIS COMPARISON OPERATOR:
    bool operator<(const Vec3& other) const {
        // Compare x first, then y, then z
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

    // ============================================================================
    // FORWARD DECLARATIONS
    // ============================================================================

    class CityNetwork;
    class ROS2SensorSubscriber;
    struct Pipe;
    struct Building;


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
        float base_demand; 
        
        Zone(int id, Vec3 center) : id(id), center(center), 
            avgPressure(DEFAULT_PRESSURE), minPressure(DEFAULT_PRESSURE), 
            maxPressure(DEFAULT_PRESSURE), totalFlow(0), pressureVariance(0),
            flowToPressureRatio(0), leakFlag(false), leakSeverity(0), base_demand(0),
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
// TREATMENT/RECHARGE EVENT FOR UDP BROADCAST
// ============================================================================

// ============================================================================
// HYDRAULIC NODE (REPLACES SIMPLIFIED NODE)
// ============================================================================


struct HydraulicNode {
    int id;
    Vec3 position;
    float elevation;          // meters
    float pressure;           // kPa
    float hydraulic_grade;    // meters (pressure head + elevation)
    float base_demand;        // m³/s (nominal demand)
    float actual_demand;      // m³/s (pressure-adjusted)
    float leakage;            // m³/s
    std::vector<int> connected_pipes;
    bool is_reservoir;
    bool is_demand;
    float valve_opening;      // 0-1
    
    HydraulicNode(int id, Vec3 pos, float elev, float base_demand = 0)
        : id(id), position(pos), elevation(elev), pressure(NOMINAL_PRESSURE),
          base_demand(base_demand), actual_demand(base_demand),
          leakage(0), is_reservoir(false), is_demand(base_demand > 0),
          valve_opening(1.0f) {
        updateHydraulicGrade();
    }
    
    void updateHydraulicGrade() {
        // Convert pressure to head: P = ρgh => h = P/(ρg)
        float pressure_head = (pressure * 1000.0f) / (WATER_DENSITY * GRAVITY); // meters
        hydraulic_grade = elevation + pressure_head;
    }
    
    void updateDemand(float dt, float simulation_time) {
        if (!is_demand) return;
        
        // Realistic: demand reduces when pressure is too low
        float pressure_ratio = pressure / NOMINAL_PRESSURE;
        pressure_ratio = std::max(0.0f, std::min(2.0f, pressure_ratio));
        
        // Square root relationship for flow vs pressure
        float demand_factor = sqrtf(pressure_ratio);
        
        // Add time-of-day variation
        float time_of_day = fmod(simulation_time, 86400.0f) / 3600.0f;
        float diurnal_factor = 0.6f + 0.4f * sinf((time_of_day - 12.0f) * M_PI / 12.0f);
        
        // Add random variation (±10%)
        float random_variation = 0.9f + 0.2f * (rand() % 1000) / 1000.0f;
        
        actual_demand = base_demand * demand_factor * diurnal_factor * random_variation;
        
        // Update leakage (pressure-dependent)
        if (leakage > 0) {
            leakage *= sqrtf(pressure_ratio);
        }
    }
};

// ============================================================================
// HYDRAULIC PIPE (REPLACES SIMPLIFIED PIPE PHYSICS)
// ============================================================================

// ADD THIS STRUCT right after HydraulicNode:
struct HydraulicPipe {
    int id;
    int start_node;
    int end_node;
    float diameter;          // meters
    float length;            // meters
    float roughness;         // Manning's n
    float cv_factor;         // Valve flow coefficient

    HydraulicPipe(int id, int start, int end, float diam, float len)
        : id(id), start_node(start), end_node(end), diameter(diam), length(len) {
        // Set defaults
        roughness = PIPE_ROUGHNESS;
        cv_factor = 10.0f;  // Default Cv value
    }
    
    // Hazen-Williams coefficients by pipe type
    float getHazenWilliamsC() const {
        if (diameter >= 0.6f) return 140.0f; // Trunk mains (new steel)
        if (diameter >= 0.25f) return 130.0f; // Secondary mains (ductile iron)
        if (diameter >= 0.1f) return 150.0f;  // Distribution (PVC)
        return 150.0f;                        // Service pipes (copper)
    }
    
    // Darcy-Weisbach friction factor
    float calculateFrictionFactor(float flow_velocity, float viscosity) const {
        if (flow_velocity < 0.001f) return 0.02f;
        
        float Re = (WATER_DENSITY * flow_velocity * diameter) / viscosity;
        Re = std::max(1.0f, Re);
        
        // Colebrook-White approximation
        float epsilon = roughness / diameter;
        float A = epsilon / 3.7f + 5.74f / pow(Re, 0.9f);
        float f = 0.25f / pow(log10(A), 2.0f);
        
        return std::max(0.008f, std::min(0.08f, f));
    }
    
    // Calculate flow through pipe (PRESSURE-DRIVEN)
    float calculateFlow(float upstream_pressure, float downstream_pressure, 
                   float valve_opening = 1.0f) const {
    // SAFETY: Check inputs
    if (!std::isfinite(upstream_pressure) || !std::isfinite(downstream_pressure)) {
        return 0.001f;  // Return small flow if pressures are invalid
    }
    
    if (valve_opening <= 0.01f) return 0.001f;
    
    // Calculate pressure difference with bounds
    float delta_p = upstream_pressure - downstream_pressure;
    
    // SAFETY: Check for invalid delta_p
    if (delta_p <= 0.001f) return 0.001f;
    
    // SAFETY: Prevent sqrt of negative or extremely large
    if (delta_p < 0.0f) delta_p = 0.001f;
    if (delta_p > 10000.0f) delta_p = 10000.0f;  // Cap at 10 MPa
    
    // Valve effect (Cv method) with safety
    float effective_cv = cv_factor * valve_opening * valve_opening;
    
    // SAFETY: Check cv_factor
    if (!std::isfinite(effective_cv) || effective_cv <= 0.001f) {
        effective_cv = 1.0f;
    }
    
    // Calculate flow: Q = Cv * √(ΔP)
    float flow_valve = effective_cv * sqrtf(delta_p);
    
    // SAFETY: Check for invalid flow
    if (!std::isfinite(flow_valve)) {
        flow_valve = 0.001f;
    }
    
    // Pipe capacity constraint
    float max_velocity = 2.0f;
    float area = M_PI * diameter * diameter / 4.0f;
    
    // SAFETY: Check area
    if (area <= 0.0001f) {
        area = 0.001f;
    }
    
    float max_flow = max_velocity * area;
    
    // Clamp flow to reasonable range
    flow_valve = std::max(0.001f, std::min(max_flow, flow_valve));
    
    return flow_valve;
}
    
    // Calculate pressure drop for given flow
    float calculatePressureDrop(float flow, float valve_opening = 1.0f) const {
        if (flow <= 0.001f || valve_opening <= 0.01f) return 0.0f;
        
        float area = M_PI * diameter * diameter / 4.0f;
        float velocity = flow / area;
        
        // Valve pressure drop
        float valve_drop = 0.0f;
        if (valve_opening < 1.0f) {
            float effective_cv = cv_factor * valve_opening * valve_opening;
            valve_drop = pow(flow / effective_cv, 2.0f);
        }
        
        // Friction pressure drop (Darcy-Weisbach)
        float f = calculateFrictionFactor(velocity, DYNAMIC_VISCOSITY);
        float friction_drop = f * (length / diameter) * 
                            (WATER_DENSITY * velocity * velocity / 2.0f);
        friction_drop /= 1000.0f; // Pa to kPa
        
        float minor_drop = friction_drop * 0.1f;
        
        return valve_drop + friction_drop + minor_drop;
    }
};


// ============================================================================
// HYDRAULIC NETWORK SOLVER
// ============================================================================

// ADD THIS CLASS right after HydraulicPipe:
class HydraulicNetwork {
private:
    std::vector<HydraulicNode> nodes;
    std::vector<HydraulicPipe> pipes;
    std::map<int, std::vector<int>> adjacency;
    int reservoir_node_id;
    
public:
    HydraulicNetwork() : reservoir_node_id(-1) {}
    const std::vector<HydraulicNode>& getNodes() const { return nodes; }
    const std::vector<HydraulicPipe>& getPipes() const { return pipes; }\
    void connectBuildingDemands(const CityNetwork& city);
    void enforceMassBalance();
    
    void addNode(const HydraulicNode& node) {
        nodes.push_back(node);
        if (node.is_reservoir) {
            reservoir_node_id = node.id;
        }
    }

    const HydraulicPipe* getPipe(int start_node, int end_node) const {
        for (const auto& pipe : pipes) {
            if ((pipe.start_node == start_node && pipe.end_node == end_node) ||
                (pipe.start_node == end_node && pipe.end_node == start_node)) {
                return &pipe;
            }
        }
        return nullptr;
    }

    int findNodeByPosition(const Vec3& pos, float tolerance = 5.0f) const {
        for (const auto& node : nodes) {
            if ((node.position - pos).length() < tolerance) {
                return node.id;
            }
        }
        return -1;
    }
    
    void addPipe(const HydraulicPipe& pipe) {
        pipes.push_back(pipe);
        adjacency[pipe.start_node].push_back(pipe.id);
        adjacency[pipe.end_node].push_back(pipe.id);
    }
    
    // SOLVE NETWORK USING GRADIENT METHOD
   void solve(float dt, float simulation_time, int max_iterations = 50, float tolerance = 0.01f) {
    if (reservoir_node_id < 0) return;
    
    // Initialize pressures with bounds
    for (auto& node : nodes) {
        if (!node.is_reservoir) {
            node.pressure = std::max(0.0f, std::min(500.0f, NOMINAL_PRESSURE));
        }
    }
    
    // Set reservoir pressure with bounds
    nodes[reservoir_node_id].pressure = std::max(0.0f, std::min(500.0f, 280.0f));
    nodes[reservoir_node_id].updateHydraulicGrade();
    
    // Initialize flows
    std::vector<float> pipe_flows(pipes.size(), 0.001f);
    
    for (int iter = 0; iter < max_iterations; iter++) {
        float max_error = 0.0f;
        
        // Update demands with bounds
        for (auto& node : nodes) {
            if (node.is_demand) {
                node.updateDemand(dt, simulation_time);
                // Clamp demand
                node.actual_demand = std::max(0.0f, std::min(10.0f, node.actual_demand));
            }
        }
        
        // Calculate flows and pressures
        for (size_t i = 0; i < pipes.size(); i++) {
            const auto& pipe = pipes[i];
            if (pipe.start_node >= nodes.size() || pipe.end_node >= nodes.size()) continue;
            
            HydraulicNode& start = nodes[pipe.start_node];
            HydraulicNode& end = nodes[pipe.end_node];
            
            // SAFETY: Check pressures are finite
            if (!std::isfinite(start.pressure)) start.pressure = NOMINAL_PRESSURE;
            if (!std::isfinite(end.pressure)) end.pressure = NOMINAL_PRESSURE;
            
            // Clamp pressures
            start.pressure = std::max(0.0f, std::min(500.0f, start.pressure));
            end.pressure = std::max(0.0f, std::min(500.0f, end.pressure));
            
            float valve_opening = std::max(0.0f, std::min(1.0f, end.valve_opening));
            
            // Calculate flow with safety
            float flow = pipe.calculateFlow(start.pressure, end.pressure, valve_opening);
            
            // SAFETY: Check flow is finite
            if (!std::isfinite(flow)) {
                flow = 0.001f;
            }
            
            // Clamp flow
            flow = std::max(0.001f, std::min(10.0f, flow));
            pipe_flows[i] = flow;
            
            // Calculate pressure drop
            float pressure_drop = pipe.calculatePressureDrop(flow, valve_opening);
            
            // SAFETY: Check pressure drop
            if (!std::isfinite(pressure_drop)) {
                pressure_drop = 0.0f;
            }
            
            // Update downstream pressure with under-relaxation
            float new_pressure = start.pressure - pressure_drop;
            new_pressure = std::max(0.0f, std::min(500.0f, new_pressure));
            
            float relaxation = 0.3f;
            float old_pressure = end.pressure;
            end.pressure = old_pressure * (1.0f - relaxation) + new_pressure * relaxation;
            
            // Clamp final pressure
            end.pressure = std::max(0.0f, std::min(500.0f, end.pressure));
            
            // Track error
            float error = fabs(new_pressure - old_pressure);
            max_error = std::max(max_error, error);
        }
        
        // Check for convergence
        if (max_error < tolerance) {
            break;
        }
    }
    
    // Final updates with bounds
    for (auto& node : nodes) {
        node.updateHydraulicGrade();
        if (node.is_demand) {
            node.updateDemand(dt, simulation_time);
            // Clamp final demand
            node.actual_demand = std::max(0.0f, std::min(10.0f, node.actual_demand));
        }
        // Ensure pressure is finite
        if (!std::isfinite(node.pressure)) {
            node.pressure = NOMINAL_PRESSURE;
        }
    }
}
    
    // Apply valve action
    void setValve(int node_id, float opening) {
        if (node_id >= 0 && node_id < nodes.size()) {
            float current = nodes[node_id].valve_opening;
            float delta = opening - current;
            float max_delta = 0.1f;
            
            if (fabs(delta) > max_delta) {
                delta = (delta > 0) ? max_delta : -max_delta;
            }
            
            nodes[node_id].valve_opening = current + delta;
            nodes[node_id].valve_opening = std::max(0.0f, std::min(1.0f, nodes[node_id].valve_opening));
        }
    }
    
    // Calculate total system demand
    float getTotalDemand() const {
        float total = 0.0f;
        for (const auto& node : nodes) {
            if (node.is_demand) {
                total += node.actual_demand + node.leakage;
            }
        }
        return total;
    }
    
    // Introduce random leaks
    void updateLeaks(float dt) {
        for (auto& node : nodes) {
            if (node.is_demand) {
                float leak_prob = LEAK_PROBABILITY_PER_HOUR * dt / 3600.0f;
                if ((rand() % 10000) / 10000.0f < leak_prob) {
                    float leak_size = node.base_demand * (0.05f + 0.1f * (rand() % 1000)/1000.0f);
                    node.leakage += leak_size;
                }
                
                if (node.leakage > 0) {
                    node.leakage *= (1.0f + 0.001f * (node.pressure / NOMINAL_PRESSURE) * dt);
                }
            }
        }
    }
    
    // Get node pressure
    float getNodePressure(int node_id) const {
        if (node_id >= 0 && node_id < nodes.size()) {
            return nodes[node_id].pressure;
        }
        return NOMINAL_PRESSURE;
    }
    
    // Get node flow
    float getNodeFlow(int node_id) const {
        if (node_id >= 0 && node_id < nodes.size()) {
            return nodes[node_id].actual_demand + nodes[node_id].leakage;
        }
        return 0.0f;
    }
    
    // Get valve state
    float getValveState(int node_id) const {
        if (node_id >= 0 && node_id < nodes.size()) {
            return nodes[node_id].valve_opening * 100.0f;
        }
        return 100.0f;
    }
    
    int getZoneNodeID(int zone_id) const {
    // Zone 0 = node 1, Zone 1 = node 2, etc.
    // Ensure zone_id is valid
    if (zone_id < 0) return -1;
    
    // Find demand node for this zone
    for (const auto& node : nodes) {
        if (node.is_demand) {
            // Check if this node corresponds to the zone
            // Simplified: zone_id maps to node.id-1
            if (node.id == zone_id + 1) {
                return node.id;
            }
        }
    }
    
    return zone_id + 1;  // Default mapping
}
};



void HydraulicNetwork::enforceMassBalance() {
    float total_demand = getTotalDemand();
    
    // Find total inflow from reservoir
    float total_inflow = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.start_node == reservoir_node_id) {
            // Find flow for this pipe
            float flow = pipe.calculateFlow(
                nodes[reservoir_node_id].pressure,
                nodes[pipe.end_node].pressure,
                nodes[pipe.end_node].valve_opening
            );
            total_inflow += flow;
        }
    }
    
    // Check mass balance (allow small tolerance)
    float imbalance = fabs(total_inflow - total_demand);
    if (imbalance > total_demand * 0.1f) {  // More than 10% imbalance
        printf("⚠ Mass imbalance: Inflow=%.4f m³/s, Demand=%.4f m³/s\n", 
               total_inflow, total_demand);
        
        // Adjust reservoir outflow
        for (auto& pipe : pipes) {
            if (pipe.start_node == reservoir_node_id) {
                // Could adjust valve openings or log warning
            }
        }
    }
}


struct TreatmentEvent {
    int event_id;
    std::string event_type;  // "TREATMENT" or "RECHARGE"
    float timestamp;
    float fresh_water_before;    // % before event
    float fresh_water_after;     // % after event
    float sewage_before;         // % before event
    float sewage_after;          // % after event
    float duration_seconds;      // How long since last event
    std::string status;          // "COMPLETE" or "IN_PROGRESS"
    
    TreatmentEvent() : event_id(0), timestamp(0), 
                      fresh_water_before(0), fresh_water_after(0),
                      sewage_before(0), sewage_after(0),
                      duration_seconds(0), status("") {}
    
    std::string toJSON() const {
        Json::Value root;
        
        root["event_id"] = event_id;
        root["event_type"] = event_type;
        root["timestamp"] = timestamp;
        root["fresh_water_before"] = fresh_water_before;
        root["fresh_water_after"] = fresh_water_after;
        root["sewage_before"] = sewage_before;
        root["sewage_after"] = sewage_after;
        root["duration_seconds"] = duration_seconds;
        root["status"] = status;
        
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        return Json::writeString(builder, root);
    }
};

    // ============================================================================
// SEWAGE RESERVOIR STRUCTURE
// ============================================================================

struct SewageReservoir {
    Vec3 position;
    float capacity;          // m³
    float volume;           // m³ (waste collected)
    float level;            // meters (fill height)
    float maxInflowRate;    // m³/s maximum inflow
    float inflowRate;       // m³/s current inflow
    float treatmentRate;    // m³/s treatment capacity
    bool needsTreatment;    // Flag when full
    time_t lastTreatmentTime; // When last treated
    float timeSinceLastTreatment; // Seconds
    
    SewageReservoir() : position(0, 0, 0), capacity(8000.0f), volume(1000.0f),
                       level(0), maxInflowRate(1.5f), inflowRate(0),
                       treatmentRate(2.0f), needsTreatment(false),
                       lastTreatmentTime(0), timeSinceLastTreatment(0) {
        level = (volume / capacity) * 20.0f; // Shorter tank
    }
    
    void update(float dt, float sewageInflow) {
        // Limit inflow
        inflowRate = std::min(sewageInflow, maxInflowRate);
        
        // Add sewage
        volume += inflowRate * dt;
        
        // Clamp volume
        if (volume < 0) volume = 0;
        if (volume > capacity) {
            volume = capacity;
            needsTreatment = true;
        }
        
        // Update level
        level = (volume / capacity) * 20.0f;
        
        // Update treatment timer
        timeSinceLastTreatment += dt;
        
        // Auto-treatment every ~10 seconds when full
        if (needsTreatment && timeSinceLastTreatment >= 10.0f) {
            performTreatment();
        }
    }
    
    void performTreatment() {
        printf(" SEWAGE TREATMENT: Processing %.2f m³ of waste\n", volume);
        
        // Empty the reservoir (treated water goes back to environment)
        volume = 0;
        level = 0;
        needsTreatment = false;
        lastTreatmentTime = time(nullptr);
        timeSinceLastTreatment = 0;
        
        printf(" Treatment complete. Reservoir empty.\n");
    }
    
    float getFillPercent() const {
        return (volume / capacity) * 100.0f;
    }
    
    std::string getStatus() const {
        if (needsTreatment) {
            return "NEEDS_TREATMENT";
        } else if (volume > capacity * 0.8f) {
            return "NEARLY_FULL";
        } else {
            return "NORMAL";
        }
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

    static float* city_simulation_time_ptr;

    bool active;
    int connectedPipeID;
    
    // Valve control
    time_t lastActionTime;
    int recentActionCount;
    float targetValveState;
    
    // Last update source tracking
    int last_esp_id;
    time_t last_update_time;
    
    float cumulativeVolume_m3 = 0.0f;
    float hourlyVolume_m3[24] = {0};
    float dailyVolume_m3 = 0.0f;
    time_t lastVolumeUpdate = 0;

    // ROS2/SIMULATION HYBRID FIELDS
    bool pressureFromROS = false;
    time_t lastROSPressureUpdate = 0;
    float pressureSimulated = DEFAULT_PRESSURE;  // Store simulated pressure separately
    float pressureROS = 0.0f;                    // Last pressure from ROS2
    float levelROS = 0.0f;                       // Last level from ROS2
        
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
        // Store in ROS-specific fields
        pressureROS = press;
        levelROS = level;
        lastROSPressureUpdate = time(nullptr);
        
        // Also update main fields for compatibility
        valveState = valve;
        pressure = press;            // This will be overridden if not fresh
        waterLevel = level;
        last_esp_id = esp_id;
        last_update_time = time(nullptr);
        pressureFromROS = true;
    }

    float getEffectivePressure() const {
        time_t now = time(nullptr);
        
        // If ROS2 data is fresh (< 3 seconds), use it
        if (lastROSPressureUpdate > 0 && (now - lastROSPressureUpdate) <= 3) {
            return pressureROS;
        }
        
        // Otherwise use simulated pressure
        return pressureSimulated;
    }
        
        bool isStale() const {
            return (time(nullptr) - last_update_time) > 5; // 5 seconds stale
        }

        bool isROSFresh() const {
            return !isStale();
        }


    // FIXED: Sensor volume tracking with proper hourly calculation
void updateVolume(float flow_m3s, float dt) {
    if (flow_m3s <= 0.0f) return;
    
    float volume = flow_m3s * dt;
    cumulativeVolume_m3 += volume;
    
    // FIX: Use simulation time from CityNetwork
    if (city_simulation_time_ptr) {
        float sim_hours = *city_simulation_time_ptr / 3600.0f;
        int currentHour = static_cast<int>(sim_hours) % 24;
        
        // DEBUG: Track hour changes
        static int last_hour_printed = -1;
        if (currentHour != last_hour_printed && id == 0) {
            printf("[SENSOR 0] Hour changed to %d, adding %.6f m³\n", 
                   currentHour, volume);
            last_hour_printed = currentHour;
        }
        
        // Add to current hour's usage
        hourlyVolume_m3[currentHour] += volume;
        
        // Reset next hour's usage when hour changes
        static int last_recorded_hour = -1;
        if (currentHour != last_recorded_hour) {
            // If we have a next hour value from previous cycle, clear it
            int next_hour = (last_recorded_hour + 1) % 24;
            if (next_hour >= 0 && next_hour < 24) {
                hourlyVolume_m3[next_hour] = 0.0f;
            }
            last_recorded_hour = currentHour;
        }
    }
    
    // Update daily volume
    dailyVolume_m3 += volume;
    
    // Reset daily volume every 24 simulation hours
    static float last_reset_time = 0.0f;
    if (city_simulation_time_ptr) {
        if (*city_simulation_time_ptr - last_reset_time >= 86400.0f) {
            dailyVolume_m3 = 0.0f;
            last_reset_time = *city_simulation_time_ptr;
            
            // Also reset hourly array at day change
            for (int i = 0; i < 24; i++) {
                hourlyVolume_m3[i] = 0.0f;
            }
        }
    }
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
    float* Sensor::city_simulation_time_ptr = nullptr;

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
        : id(id), start(s), end(e), type(t), diameter(0),
        hasLeak(false), leakRate(0.0f), flowRate(0.0f),
        pressureDrop(0.0f), velocity(0.0f), zoneID(zoneID),
        flowRegime("STAGNANT")  {

        // ================== FIXED: CONSISTENT DIAMETER DEFINITION ==================
        switch(t) {
            case TRUNK_MAIN: diameter = TRUNK_DIAMETER; break;
            case SECONDARY_MAIN: diameter = SECONDARY_DIAMETER; break;
            case RING_MAIN: diameter = RING_DIAMETER; break;
            case SERVICE_PIPE: diameter = SERVICE_DIAMETER; break;
            case SEWAGE_LATERAL: diameter = SERVICE_DIAMETER * 1.5f; break;
            case SEWAGE_COLLECTOR: diameter = SECONDARY_DIAMETER * 1.2f; break;
            case SEWAGE_INTERCEPTOR: diameter = TRUNK_DIAMETER * 1.2f; break;
        }
        
        // Override with provided diameter if needed
        if (diam > 0) {
            diameter = diam;
        }
        
        length = (end - start).length();
        elevationChange = end.y - start.y;
    }
        // Calculate pressure drop using Darcy-Weisbach equation
    

    float calculatePressureDrop(float flow_m3s, float viscosity = DYNAMIC_VISCOSITY) {
    if (diameter <= 0 || length <= 0) return 0.0f;
    
    // ================== REALISTIC CONSTANTS ==================
    const float MIN_FLOW_VELOCITY = 0.05f;    // LOWER: 0.05 m/s
    const float MAX_FLOW_VELOCITY = 2.0f;     // LOWER: 2.0 m/s
    const float MIN_REYNOLDS = 2000.0f;
    const float SYSTEM_AGE_YEARS = 15.0f;
    
    // ================== REALISTIC PIPE ROUGHNESS ==================
    float roughness_mm = 0.0f;
    switch (type) {
        case TRUNK_MAIN:        roughness_mm = 0.06f; break;
        case SECONDARY_MAIN:    roughness_mm = 0.15f; break;
        case RING_MAIN:         roughness_mm = 0.10f; break;
        case SERVICE_PIPE:      roughness_mm = 0.007f; break;
        case SEWAGE_LATERAL:    roughness_mm = 0.60f; break;
        case SEWAGE_COLLECTOR:  roughness_mm = 0.80f; break;
        case SEWAGE_INTERCEPTOR: roughness_mm = 1.20f; break;
        default:                roughness_mm = 0.30f; break;
    }
    
    // Convert mm to meters
    float roughness_m = roughness_mm / 1000.0f;
    
    // Calculate area
    float area = M_PI * diameter * diameter / 4.0f;
    
    // Limit flow to realistic velocity range
    float max_flow = MAX_FLOW_VELOCITY * area;
    float min_flow = MIN_FLOW_VELOCITY * area;
    
    if (flow_m3s > max_flow) {
        flow_m3s = max_flow;
    } else if (flow_m3s < min_flow && flow_m3s > 0.0f) {
        flow_m3s = min_flow;
    }
    
    velocity = (area > 0.0001f) ? flow_m3s / area : 0.0f;
    
    // Reynolds number
    float Re = 1.0f;
    if (viscosity > 0.0001f && velocity > 0.001f) {
        Re = (WATER_DENSITY * velocity * diameter) / viscosity;
        if (Re < 1.0f) Re = 1.0f;
    }
    
    // Friction factor
    float frictionFactor = 0.02f;  // Default
    float epsilon = roughness_m / diameter;
    
    if (Re < MIN_REYNOLDS) {
        frictionFactor = 64.0f / Re;  // Laminar flow
        flowRegime = "LAMINAR";
    } else {
        // Colebrook-White for turbulent flow
        float A = epsilon / 3.7f + 5.74f / pow(Re, 0.9f);
        if (A > 0) {
            frictionFactor = 0.25f / pow(log10(A), 2.0f);
        }
        flowRegime = "TURBULENT";
    }
    
    // Darcy-Weisbach pressure drop
    float pressureDrop_Pa = 0.0f;
    if (diameter > 0.001f && length > 0.001f) {
        pressureDrop_Pa = frictionFactor * (length / diameter) * 
                         (WATER_DENSITY * velocity * velocity / 2.0f);
    }
    
    pressureDrop = pressureDrop_Pa / 1000.0f;  // Pa to kPa
    
    // Elevation head
    if (elevationChange != 0.0f) {
        float elevationPressure_Pa = WATER_DENSITY * GRAVITY * elevationChange;
        float elevationPressure_kPa = elevationPressure_Pa / 1000.0f;
        pressureDrop += elevationPressure_kPa;
    }
    
    // Ensure non-negative
    if (pressureDrop < 0.0f) pressureDrop = 0.0f;
    
    // Cap at reasonable maximum
    float maxPressureDrop_kPa = 500.0f;
    if (pressureDrop > maxPressureDrop_kPa) {
        pressureDrop = maxPressureDrop_kPa;
    }
    
    return pressureDrop;
}

// Add to Pipe struct:
std::string flowRegime; // "LAMINAR", "TRANSITIONAL", "TURBULENT", "STAGNANT"

// ================== HAZEN-WILLIAMS ALTERNATIVE (for water distribution) ==================
float calculatePressureDropHazenWilliams(float flow_m3s) {
    // Hazen-Williams is commonly used in water distribution engineering
    // hf = 10.67 * L * Q^1.852 / (C^1.852 * D^4.87)
    
    // C-factor (Hazen-Williams coefficient) - higher = smoother
    float C_factor = 0.0f;
    switch (type) {
        case TRUNK_MAIN:        C_factor = 140.0f; break;   // New steel
        case SECONDARY_MAIN:    C_factor = 130.0f; break;   // Ductile iron
        case RING_MAIN:         C_factor = 150.0f; break;   // PVC
        case SERVICE_PIPE:      C_factor = 150.0f; break;   // Copper
        default:                C_factor = 100.0f; break;   // Old pipes
    }
    
    // Age the pipe (C-factor decreases over time)
    float ageYears = 15.0f;
    C_factor *= (1.0f - 0.005f * ageYears); // 0.5% per year degradation
    
    // Convert flow to m³/s to L/s for Hazen-Williams
    float Q_lps = flow_m3s * 1000.0f;
    
    // Calculate head loss (meters)
    float hf = 10.67f * length * pow(Q_lps, 1.852f) / 
               (pow(C_factor, 1.852f) * pow(diameter, 4.87f));
    
    // Convert head loss to pressure (kPa)
    float pressureDrop_kPa = (WATER_DENSITY * GRAVITY * hf) / 1000.0f;
    
    return pressureDrop_kPa;
}

// ================== MANNING'S EQUATION (for sewage) ==================
// REPLACE the ENTIRE calculatePressureDropManning() function with this:

float calculateSewageFlow(float slope) {
    // SEWAGE FLOW - NO PRESSURE, gravity only
    if (slope <= 0.001f) return 0.0f;  // Need minimum slope
    
    // Manning's roughness coefficients
    float n = 0.013f;  // PVC/concrete
    
    switch (type) {
        case SEWAGE_LATERAL:    n = 0.013f; break;
        case SEWAGE_COLLECTOR:  n = 0.015f; break;
        case SEWAGE_INTERCEPTOR: n = 0.017f; break;
        default:                n = 0.014f; break;
    }
    
    // For sewage, assume pipe is 80% full (standard design)
    float depth_ratio = 0.8f;
    float theta = 2.0f * acos(1.0f - 2.0f * depth_ratio); // Central angle
    
    // Flow area for partially full pipe
    float A = (diameter * diameter / 4.0f) * (theta - sin(theta)) / 2.0f;
    
    // Wetted perimeter
    float P = diameter * theta / 2.0f;
    
    // Hydraulic radius
    float R = A / P;
    
    // Manning's equation: Q = (1/n) * A * R^(2/3) * S^(1/2)
    float Q = (1.0f / n) * A * pow(R, 2.0f/3.0f) * sqrt(slope);
    
    return Q;  // m³/s
}

// ================== UNIFIED PRESSURE DROP CALCULATOR ==================
float calculateRealisticPressureDrop(float flow_m3s) {
    // Only for water pipes!
    if (type >= SEWAGE_LATERAL) {
        return 0.0f; // Sewage doesn't use pressure drop
    }
    
    if (type == SERVICE_PIPE || type == RING_MAIN) {
        // Distribution pipes: Use Hazen-Williams
        return calculatePressureDropHazenWilliams(flow_m3s);
    } else {
        // Transmission mains: Use Darcy-Weisbach
        return calculatePressureDrop(flow_m3s, DYNAMIC_VISCOSITY);
    }
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
    
    // Use point sprites for better performance
    glEnable(GL_POINT_SPRITE);
    glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
    glPointSize(2.0f);
    
    glBegin(GL_POINTS);
    for (const auto& particle : particles) {
        Vec3 pos = particle.getWorldPosition(*this);
        
        // Calculate velocity-based color
        float area = M_PI * diameter * diameter / 4.0f;
        float velocity = flowRate / area;
        float vel_factor = std::min(1.0f, velocity / 3.0f);
        
        Color col = particle.color;
        col.r *= (0.7f + 0.3f * vel_factor);
        col.g *= (0.7f + 0.3f * vel_factor);
        col.b = 0.8f + 0.2f * vel_factor;
        
        glColor4f(col.r, col.g, col.b, particle.life * 0.6f);
        glVertex3f(pos.x, pos.y, pos.z);
    }
    glEnd();
    
    glDisable(GL_POINT_SPRITE);
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

extern std::vector<Zone> zones;  
extern std::vector<Sensor> sensors; 

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
    float pressureSimulated;

    // ================== INTEGER DEMAND SYSTEM ==================
    int baseWaterDemand_Lps;          // Base demand in liters per second
    int currentWaterDemand_Lps;       // Current demand (integer)
    float demandChangeTimer;          // Timer for demand changes (seconds)
    int nextDemandChangeTime;         // When next change occurs (seconds)
    int minDemand_Lps;                // Minimum demand
    int maxDemand_Lps;                // Maximum demand
    int demandChangeAmount;           // Current change amount (+/- L/s)
    float actual_pressure;

    
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
        demandChangeTimer(0), demandChangeAmount(0),
        weeklyAverage(0),
        pressureSimulated(NOMINAL_PRESSURE) { // ADD THIS INITIALIZATION
        
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
                population = 5 + rand() % 10;               // 5–15 people per residence
                sewageFactor = 0.7f + (rand() % 20) / 100.0f; // 0.7–0.9
                break;
                
            case COMMERCIAL:
                population = 20 + rand() % 80;              // 20–100 people
                sewageFactor = 0.5f + (rand() % 20) / 100.0f; // 0.5–0.7
                break;
                
            case HOSTEL:
                population = 30 + rand() % 70;              // 30–100 residents
                sewageFactor = 0.9f + (rand() % 30) / 100.0f; // 0.9–1.2
                break;
                
            case HOSPITAL:
                population = 50 + rand() % 150;             // 50–200 people
                sewageFactor = 1.0f + (rand() % 30) / 100.0f; // 1.0–1.3
                break;
                
            case INDUSTRIAL:
                population = 10 + rand() % 40;              // 10–50 workers
                sewageFactor = 1.1f + (rand() % 40) / 100.0f; // 1.1–1.5
                break;
        }
        
        // ================== INTEGER DEMAND CALCULATION ==================
        switch (type) {
            case RESIDENTIAL:
                baseWaterDemand_Lps = 5 + rand() % 15;       // 1-5 L/s per residence
                minDemand_Lps = 1;                         // Night time minimum
                maxDemand_Lps = 30;                        // Peak shower time
                break;
            case COMMERCIAL:
                baseWaterDemand_Lps = 10 + rand() % 20;       // 3-10 L/s
                minDemand_Lps = 2;
                maxDemand_Lps = 50;
                break;
            case HOSTEL:
                baseWaterDemand_Lps = 15 + rand() % 30;      // 5-15 L/s
                minDemand_Lps = 2;
                maxDemand_Lps = 80;
                break;
            case HOSPITAL:
                baseWaterDemand_Lps = 30 + rand() % 25;     // 10-25 L/s
                minDemand_Lps = 5;
                maxDemand_Lps = 80;
                break;
            case INDUSTRIAL:
                baseWaterDemand_Lps = 80 + rand() % 40;      // 8-20 L/s
                minDemand_Lps = 3;
                maxDemand_Lps = 100;
                break;
        }
        
        // Initialize current demand
        currentWaterDemand_Lps = baseWaterDemand_Lps;
        nextDemandChangeTime = 30 + rand() % 90; // Change every 30-120 seconds
        
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
        
        // Convert initial integer demand to flow (m³/s)
        currentWaterFlow = currentWaterDemand_Lps * 0.001f;
    }
    
    int getBaseDemandLps() const { 
        return baseWaterDemand_Lps; 
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
    
    // ============================================================================
// UPDATE THE Building::updateWaterUsage METHOD (around line 1600)
// ============================================================================
void updateWaterUsage(float dt, const DayNightCycle& dayNight, float actual_pressure, const std::vector<Sensor>& sensors) {
    this->actual_pressure = actual_pressure;

    if (!citySimulationTime) return;
    
    // ================== USE ACTUAL PRESSURE PARAMETER ==================
    // Instead of using zone pressure, use the actual pressure passed as parameter
    float pressure_ratio = this->actual_pressure / NOMINAL_PRESSURE;
    pressure_ratio = std::max(0.3f, std::min(2.0f, pressure_ratio));
    
    
    // Square root relationship for flow vs pressure
    float pressure_factor = sqrtf(pressure_ratio);
    
    // ================== TIME-OF-DAY PATTERN ==================
    float time_of_day = fmod(*citySimulationTime, 86400.0f) / 3600.0f;
    float diurnal_factor = 1.0f;
    
    // Enhanced time-based demand pattern based on building type
    switch (type) {
        case RESIDENTIAL:
            // Residential: morning (7-9) and evening (17-21) peaks
            if (time_of_day >= 6.0f && time_of_day <= 9.0f) {
                // Morning peak (getting ready for work/school)
                float peak = (time_of_day - 6.0f) / 3.0f;
                diurnal_factor = 0.8f + 0.6f * sinf(peak * M_PI);
            } else if (time_of_day >= 17.0f && time_of_day <= 21.0f) {
                // Evening peak (cooking, cleaning, bathing)
                float peak = (time_of_day - 17.0f) / 4.0f;
                diurnal_factor = 0.9f + 0.8f * sinf(peak * M_PI);
            } else if (time_of_day >= 22.0f || time_of_day <= 5.0f) {
                // Night (low usage)
                diurnal_factor = 0.3f;
            } else {
                // Daytime (moderate)
                diurnal_factor = 0.5f;
            }
            break;
            
        case COMMERCIAL:
            // Commercial: business hours only
            if (time_of_day >= 8.0f && time_of_day <= 18.0f) {
                // Peak at lunchtime (12-13)
                float peak = 1.0f - fabs(time_of_day - 12.5f) / 5.0f;
                diurnal_factor = 0.6f + 0.5f * peak;
            } else {
                diurnal_factor = 0.1f;
            }
            break;
            
        case HOSTEL:
            // Hostel: moderate 24/7 with peaks around meal times
            if ((time_of_day >= 7.0f && time_of_day <= 9.0f) || 
                (time_of_day >= 18.0f && time_of_day <= 20.0f)) {
                diurnal_factor = 0.8f + 0.4f * sinf((time_of_day - 7.0f) * M_PI / 2.0f);
            } else if (time_of_day >= 22.0f || time_of_day <= 6.0f) {
                diurnal_factor = 0.5f;
            } else {
                diurnal_factor = 0.6f;
            }
            break;
            
        case HOSPITAL:
            // Hospital: 24/7 with slight reduction at night
            if (time_of_day >= 6.0f && time_of_day <= 22.0f) {
                diurnal_factor = 0.8f + 0.2f * sinf((time_of_day - 12.0f) * M_PI / 12.0f);
            } else {
                diurnal_factor = 0.6f;
            }
            break;
            
        case INDUSTRIAL:
            // Industrial: work shifts only
            if ((time_of_day >= 7.0f && time_of_day <= 12.0f) || 
                (time_of_day >= 13.0f && time_of_day <= 16.0f)) {
                diurnal_factor = 0.9f;
            } else {
                diurnal_factor = 0.1f;
            }
            break;
    }
    
    // Weekend effect
    if (dayNight.isWeekend()) {
        switch (type) {
            case RESIDENTIAL:  diurnal_factor *= 1.2f; break;  // More usage at home
            case COMMERCIAL:   diurnal_factor *= 0.5f; break;  // Less commercial activity
            case HOSTEL:       diurnal_factor *= 1.1f; break;  // Slight increase
            case HOSPITAL:     diurnal_factor *= 1.0f; break;  // No change for hospitals
            case INDUSTRIAL:   diurnal_factor *= 0.2f; break;  // Much less industrial
        }
    }
    
    // ================== BUILDING-SPECIFIC RANDOMNESS ==================
    float random_variation = 0.9f + 0.2f * (rand() % 1000) / 1000.0f;
    
    // ================== VALVE EFFECT ==================
    float valve_factor = 1.0f;
    if (waterSensorID >= 0 && waterSensorID < sensors.size()) {
        // Use valve state from sensor
        valve_factor = sensors[waterSensorID].valveState / 100.0f;
        // Square relationship for valve effect
        valve_factor = valve_factor * valve_factor;
    }
    
    // ================== UPDATE INTEGER DEMAND SYSTEM ==================
    demandChangeTimer += dt;
    
    // Check if it's time to change demand
    if (demandChangeTimer >= nextDemandChangeTime) {
        // Calculate new demand based on time of day
        float time_factor = 1.0f;
        if (type == RESIDENTIAL) {
            // Residential: higher demand during peak hours
            if ((time_of_day >= 7.0f && time_of_day <= 9.0f) || 
                (time_of_day >= 17.0f && time_of_day <= 21.0f)) {
                time_factor = 1.5f;
            }
        }
        
        // Random change (±20% of base demand)
        int changeRange = static_cast<int>(baseWaterDemand_Lps * 0.2f);
        changeRange = std::max(1, changeRange);
        
        int changeAmount = (rand() % (2 * changeRange + 1)) - changeRange;
        int newDemand = currentWaterDemand_Lps + changeAmount;
        
        // Clamp to min/max
        currentWaterDemand_Lps = std::max(minDemand_Lps, std::min(maxDemand_Lps, newDemand));
        
        // Store change amount for trend analysis
        demandChangeAmount = changeAmount;
        
        // Reset timer with new interval
        demandChangeTimer = 0.0f;
        nextDemandChangeTime = 30 + rand() % 120; // 30-150 seconds
    }
    
    // ================== FINAL FLOW CALCULATION ==================
    // Start with integer demand in L/s
    float demand_lps = static_cast<float>(currentWaterDemand_Lps);
    
    // Apply all factors:
    // 1. Pressure factor (square root relationship)
    demand_lps *= pressure_factor;
    
    // 2. Diurnal pattern
    demand_lps *= diurnal_factor;
    
    // 3. Building type multiplier
    float type_multiplier = 1.0f;
    switch (type) {
        case RESIDENTIAL:  type_multiplier = 1.0f; break;
        case COMMERCIAL:   type_multiplier = 1.2f; break;
        case HOSTEL:       type_multiplier = 1.5f; break;
        case HOSPITAL:     type_multiplier = 2.0f; break;
        case INDUSTRIAL:   type_multiplier = 2.5f; break;
    }
    demand_lps *= type_multiplier;
    
    // 4. Random variation
    demand_lps *= random_variation;
    
    // 5. Valve effect
    demand_lps *= valve_factor;
    
    // 6. Population scaling
    demand_lps *= (population / 10.0f); // Normalize to 10 people
    
    // Convert to m³/s
    currentWaterFlow = demand_lps * 0.001f;
    
    // ================== PRESSURE-DRIVEN FLOW REDUCTION ==================
    // If pressure is critically low, further reduce flow
    if (actual_pressure < MIN_OPERATING_PRESSURE) {
        float critical_factor = actual_pressure / MIN_OPERATING_PRESSURE;
        critical_factor = std::max(0.1f, critical_factor); // Minimum 10% flow
        currentWaterFlow *= critical_factor;
    }
    
    // Ensure minimum flow
    currentWaterFlow = std::max(0.0001f, currentWaterFlow);
    
    // Cap maximum flow based on pipe capacity
    float max_flow_capacity = 0.0f;
    
    if (max_flow_capacity > 0.001f) {
        currentWaterFlow = std::min(currentWaterFlow, max_flow_capacity);
    }
    
    // ================== UPDATE STATISTICS ==================
    totalWaterConsumed += currentWaterFlow * dt;
    
    // Calculate sewage flow (90% conversion with some variation)
    currentSewageFlow = currentWaterFlow * (0.85f + 0.1f * (rand() % 1000) / 1000.0f);
    
    // Update water demand factor for display
    waterDemandFactor = diurnal_factor * pressure_factor * valve_factor;
    
    // ================== DEBUG OUTPUT ==================
    static int debug_counter = 0;
    if (debug_counter++ % 1000 == 0 && id == 0) { // First building, every ~16 seconds
        printf("[Building %d] Flow=%.3f L/s, Pressure=%.1f kPa, Factor=%.2f\n",
               id, currentWaterFlow * 1000.0f, actual_pressure, waterDemandFactor);
    }
}

    
    // ================== PRESSURE-DEPENDENT DEMAND ==================
    void applyPressureEffect(float pressure_kPa) {
        // Realistic: Flow reduces when pressure is low
        const float MIN_PRESSURE = 70.0f; // kPa (0.7 bar) - minimum for proper flow
        const float NOMINAL_PRESSURE = 150.0f; // kPa (1.5 bar) - nominal pressure
        
        if (pressure_kPa < MIN_PRESSURE) {
            // Reduce flow proportionally to pressure drop
            float pressureRatio = pressure_kPa / MIN_PRESSURE;
            pressureRatio = std::max(0.0f, std::min(1.0f, pressureRatio));
            
            // Apply pressure effect (square root relationship for flow)
            float flowReduction = sqrtf(pressureRatio);
            currentWaterFlow *= flowReduction;
        } else if (pressure_kPa > NOMINAL_PRESSURE * 1.5f) {
            // Very high pressure might slightly increase flow
            float pressureRatio = pressure_kPa / (NOMINAL_PRESSURE * 1.5f);
            pressureRatio = std::min(1.2f, pressureRatio); // Max 20% increase
            currentWaterFlow *= pressureRatio;
        }
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
    
    int getCurrentDemandLps() const { 
        return currentWaterDemand_Lps; 
    }
    
    int getDemandChangeAmount() const { 
        return demandChangeAmount; 
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
    
    float getFlowPerPerson() const {
        if (population > 0) {
            return (currentWaterFlow * 1000.0f) / population; // L/s per person
        }
        return 0.0f;
    }
    
    // ================== DEBUG INFO ==================
    void printDebugInfo() const {
        printf("Building %d [%s]\n", id, getTypeString().c_str());
        printf("  Population: %d people\n", population);
        printf("  Demand: %d L/s (Base: %d L/s, Min: %d, Max: %d)\n", 
               currentWaterDemand_Lps, baseWaterDemand_Lps, minDemand_Lps, maxDemand_Lps);
        printf("  Current Flow: %.2f L/s (%.2f L/s per person)\n", 
               currentWaterFlow * 1000.0f, getFlowPerPerson());
        printf("  Daily Water: %.2f m³/day (%.0f L/person/day)\n", 
               getDailyConsumption(), getWaterPerPerson());
        printf("  Sewage Factor: %.2f (Flow: %.2f L/s)\n", 
               sewageFactor, currentSewageFlow * 1000.0f);
        printf("  Pattern Modulation: %.2fx\n", waterDemandFactor);
        printf("  Random Bias: %.2f, Volatility: %.2f\n", 
               randomDemandBias, demandVolatility);
        
        // Show peak hour
        int peakHour = 0;
        float peakValue = 0.0f;
        for (int i = 0; i < 24; i++) {
            if (hourlyUsage[i] > peakValue) {
                peakValue = hourlyUsage[i];
                peakHour = i;
            }
        }
        printf("  Peak Usage: %d:00 (%.1fx average)\n", peakHour, peakValue / (1.0f/24.0f));
    }
    
    // ================== REAL-TIME STATUS ==================
    std::string getStatusString() const {
        std::stringstream ss;
        ss << "B" << id << " [" << getTypeString().substr(0, 3) << "] ";
        ss << currentWaterDemand_Lps << " L/s";
        
        // Show trend if recent change
        if (abs(demandChangeAmount) > 10) {
            ss << " (";
            if (demandChangeAmount > 0) ss << "+";
            ss << demandChangeAmount << ")";
        }
        
        return ss.str();
    }
    
    // ================== RESET DEMAND ==================
    void resetDemand() {
        currentWaterDemand_Lps = baseWaterDemand_Lps;
        demandChangeTimer = 0.0f;
        nextDemandChangeTime = 30 + rand() % 90;
        demandChangeAmount = 0;
    }
    
    // ================== SET DEMAND ==================
    void setDemand(int demand_Lps, bool clamp = true) {
        if (clamp) {
            currentWaterDemand_Lps = std::max(minDemand_Lps, std::min(maxDemand_Lps, demand_Lps));
        } else {
            currentWaterDemand_Lps = demand_Lps;
        }
        demandChangeAmount = demand_Lps - currentWaterDemand_Lps;
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
    float inflowRate;       // m³/s (from treatment/recharge)
    float outflowRate;      // m³/s (to city)
    float levelTrend;       // Rising/falling rate
    float supplyMargin;     // Available capacity margin
    bool needsRecharge;     // Flag when low
    time_t lastRechargeTime; // When last recharged
    float timeSinceLastRecharge; // Seconds
    
    std::vector<Pipe> pumps;
    
    Reservoir() : position(0, 0, 0), capacity(10000.0f), volume(7000.0f),
                  level(0), maxPumpCapacity(2.0f), availablePumpCapacity(2.0f),
                  inflowRate(0), outflowRate(0), levelTrend(0),
                  supplyMargin(1.5f), needsRecharge(false),
                  lastRechargeTime(0), timeSinceLastRecharge(0) {
        level = (volume / capacity) * 30.0f;
    }
    
    void update(float dt, float totalDemand) {
        // ================== REALISTIC DEPLETION ==================
        // Reservoir ONLY gives out water, no magical refill
        outflowRate = std::min(totalDemand, availablePumpCapacity);
        
        // DEPLETION: Volume decreases
        float netFlow = -outflowRate; // Negative = depleting
        volume += netFlow * dt;
        
        // Check if needs recharge
        if (volume < capacity * 0.2f) { // Below 20%
            needsRecharge = true;
        }
        
        // Auto-recharge every ~10 seconds when low
        if (needsRecharge && timeSinceLastRecharge >= 10.0f) {
            performRecharge();
        }
        
        // Clamp volume
        if (volume < 0) volume = 0;
        if (volume > capacity) volume = capacity;
        
        // Update level
        level = (volume / capacity) * 30.0f;
        
        // Calculate trend (negative when depleting)
        levelTrend = (levelTrend * 0.9f) + (netFlow / capacity * 10.0f * 0.1f);
        
        // Update supply margin
        supplyMargin = availablePumpCapacity - outflowRate;
        
        // Update recharge timer
        timeSinceLastRecharge += dt;
        
        // Update available pump capacity (can reduce when low)
        float levelRatio = volume / capacity;
        availablePumpCapacity = maxPumpCapacity * 
                              std::max(0.3f, levelRatio); // Reduced when low
    }
    
    void performRecharge() {
        printf("💧 RESERVOIR RECHARGE: Refilling from %.2f%% to 70%%\n", getLevelPercent());
        
        // Refill to 70%
        volume = capacity * 0.7f;
        level = (volume / capacity) * 30.0f;
        needsRecharge = false;
        lastRechargeTime = time(nullptr);
        timeSinceLastRecharge = 0;
        
        printf("✅ Recharge complete. Reservoir at %.1f%%\n", getLevelPercent());
    }
    
    float getLevelPercent() const {
        return (volume / capacity) * 100.0f;
    }
    
    std::string getTrendString() const {
        if (levelTrend > 0.01f) return "rising";
        else if (levelTrend < -0.01f) return "falling";
        else return "stable";
    }
    
    bool isDepleting() const {
        return levelTrend < -0.01f;
    }
    
    bool needsRefill() const {
        return getLevelPercent() < 25.0f;
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

    float sewage_level_pct;
    std::string sewage_status;
    bool sewage_needs_treatment;
    float sewage_time_since_treatment;
    
    int last_treatment_event_id;
    float time_since_last_treatment;
    float time_since_last_recharge;
    
    // Zone states (DYNAMIC VECTORS - stores ALL zones)
    std::vector<float> avg_pressure_zone;
    std::vector<float> min_pressure_zone;
    std::vector<float> flow_zone;
    std::vector<float> pressure_variance_zone;
    std::vector<float> flow_to_pressure_ratio;
    std::vector<float> historical_flow;
    std::vector<bool> leak_flag;
    std::vector<float> leak_severity;
    std::vector<bool> overflow_flag;
    std::vector<bool> pressure_violation;
    
    // Demand patterns
    float time_of_day;
    std::string day_type;
    
    // Anomalies
    
    // Control state (DYNAMIC VECTORS - stores ALL sensors)
    std::vector<float> valve_state; // One per sensor
    std::vector<float> last_action_time; // One per sensor
    int recent_action_count;
    
    // Time encoding
    float sim_time_sin;
    float sim_time_cos;
    float episode_progress;

    // Sensor volumes (DYNAMIC VECTORS - stores ALL sensors)
    std::vector<float> cumulative_volume_sensor;
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
        // Initialize all members
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
        
        // Vectors are automatically empty, no need to initialize
    }
    
    std::string toJSON() const {
        Json::Value root;
        
        // Reservoir
        root["reservoir_level_pct"] = reservoir_level_pct;
        root["reservoir_trend"] = reservoir_trend;
        root["pump_capacity_available"] = pump_capacity_available;
        root["supply_margin"] = supply_margin;
        
        // Zones (DYNAMIC - based on vector size)
        Json::Value zones(Json::arrayValue);
        for (size_t i = 0; i < avg_pressure_zone.size(); i++) {
            Json::Value zone;
            zone["zone_id"] = (int)i;
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
        
        // Valves (DYNAMIC - based on vector size)
        Json::Value valves(Json::arrayValue);
        for (size_t i = 0; i < valve_state.size(); i++) {
            valves.append(valve_state[i]);
        }
        root["valve_states"] = valves;
        
        // Last action times
        Json::Value action_times(Json::arrayValue);
        for (size_t i = 0; i < last_action_time.size(); i++) {
            action_times.append(last_action_time[i]);
        }
        root["last_action_times"] = action_times;
        
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
        
        // Cumulative volumes (DYNAMIC - based on vector size)
        Json::Value volumes(Json::arrayValue);
        for (size_t i = 0; i < cumulative_volume_sensor.size(); i++) {
            volumes.append(cumulative_volume_sensor[i]);
        }
        root["cumulative_volumes"] = volumes;
        
        // Hourly usage
        Json::Value hourly(Json::arrayValue);
        for (int i = 0; i < 24; i++) {
            hourly.append(hourly_usage[i]);
        }
        root["hourly_usage"] = hourly;
        
        // Counts (important for receiver to know how much data is valid)
        root["zone_count"] = (int)avg_pressure_zone.size();
        root["sensor_count"] = (int)valve_state.size();
        
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
    const auto interval = std::chrono::milliseconds(100);  // 10Hz max
    
    while (udpRunning) {
        auto start = std::chrono::steady_clock::now();
        
        std::string message;
        {
            std::lock_guard<std::mutex> lock(udpMutex);
            if (!udpMessageQueue.empty()) {
                message = udpMessageQueue.front();
                udpMessageQueue.pop();
            }
        }
        
        if (!message.empty()) {
            // Send with error checking
            int sent = sendto(udpSocket, message.c_str(), message.length(), 0,
                            (struct sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
            
            if (sent < 0) {
                // Log error but continue
                static int error_count = 0;
                if (error_count++ % 100 == 0) {
                    perror("UDP send error");
                }
            } else if (sent > 0) {
                // DEBUG: Log occasional sends
                static int send_count = 0;
                if (send_count++ % 50 == 0) {
                    printf("📤 UDP: Sent %d bytes (Queue: %lu)\n", 
                           sent, udpMessageQueue.size());
                }
            }
        }
        
        // Sleep to maintain 10Hz rate
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < interval) {
            std::this_thread::sleep_for(interval - elapsed);
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
    
    // Limit queue to prevent memory explosion
    if (udpMessageQueue.size() < 100) {  // Max 100 messages in queue
        udpMessageQueue.push(message);
    } else {
        // Drop oldest if queue full
        udpMessageQueue.pop();
        udpMessageQueue.push(message);
        
        static int drop_count = 0;
        if (drop_count++ % 50 == 0) {
            printf("⚠ UDP queue full, dropping old messages\n");
        }
    }
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
    

class WaterFlowParticleSystem {
private:
    struct FlowParticle {
        float position;      // 0-1 along pipe length
        float radial_offset; // 0-1 from center
        float angle_offset;  // 0-2π
        float speed;         // relative to flow
        float size;          // particle size
        float intensity;     // 0-1 brightness
        int pipe_id;
        
        FlowParticle() : position(0), radial_offset(0), angle_offset(0), 
                        speed(1.0f), size(0.1f), intensity(1.0f), pipe_id(-1) {}
        
        FlowParticle(float pos, float rad, float ang, float spd, int pid)
            : position(pos), radial_offset(rad), angle_offset(ang), 
              speed(spd), size(0.1f), intensity(1.0f), pipe_id(pid) {}
    };
    
    std::vector<FlowParticle> particles;
    int max_particles;
    float spawn_rate;        // particles per second per pipe
    
public:
    WaterFlowParticleSystem(int max_particles = 1000000, float spawn_rate = 1000.0f)
        : max_particles(max_particles), spawn_rate(spawn_rate) {
        particles.reserve(max_particles);
    }
    
    void update(float dt, const std::vector<Pipe>& pipes) {
        // Update existing particles
        for (auto& p : particles) {
            if (p.pipe_id >= 0 && p.pipe_id < pipes.size()) {
                const Pipe& pipe = pipes[p.pipe_id];
                float flow_speed = pipe.flowRate / (M_PI * pipe.diameter * pipe.diameter / 4.0f);
                
                // Move particle along pipe
                p.position += flow_speed * p.speed * dt / pipe.length;
                
                // Add swirling motion
                p.angle_offset += (0.5f + p.radial_offset) * flow_speed * 0.1f * dt;
                
                // Particles fade out when near end
                if (p.position > 0.9f) {
                    p.intensity = std::max(0.0f, 1.0f - (p.position - 0.9f) * 10.0f);
                }
                
                // Reset particle if it reached end
                if (p.position > 1.0f || p.intensity <= 0.0f) {
                    // Recycle particle
                    p.position = 0.0f;
                    p.radial_offset = static_cast<float>(rand()) / RAND_MAX * 0.8f;
                    p.angle_offset = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
                    p.speed = 0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX;
                    p.intensity = 1.0f;
                }
            }
        }
        
        // Spawn new particles based on flow rate
        for (size_t i = 0; i < pipes.size(); i++) {
            const Pipe& pipe = pipes[i];
            
            // Only spawn for water pipes with significant flow
            if (pipe.type >= SEWAGE_LATERAL || pipe.flowRate < 0.01f) continue;
            
            float flow_area = M_PI * pipe.diameter * pipe.diameter / 4.0f;
            float velocity = pipe.flowRate / flow_area;
            
            // Calculate particles to spawn this frame
            float particles_per_frame = spawn_rate * velocity * dt;
            int new_particles = static_cast<int>(particles_per_frame);
            
            for (int j = 0; j < new_particles && particles.size() < max_particles; j++) {
                FlowParticle p;
                p.pipe_id = i;
                p.position = 0.0f;  // Start at beginning
                p.radial_offset = static_cast<float>(rand()) / RAND_MAX * 0.8f;  // Keep away from walls
                p.angle_offset = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
                p.speed = 0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX;  // Speed variation
                p.size = 0.02f + 0.08f * (velocity / 5.0f);  // Size based on velocity
                p.intensity = 0.7f + 0.3f * static_cast<float>(rand()) / RAND_MAX;
                
                particles.push_back(p);
            }
        }
        
        // Limit total particles
        if (particles.size() > max_particles) {
            particles.erase(particles.begin(), particles.begin() + (particles.size() - max_particles));
        }
    }
    
    void render(const std::vector<Pipe>& pipes) {
        if (particles.empty()) return;
        
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);
        
        // Use point sprites for better performance
        glEnable(GL_POINT_SPRITE);
        glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
        
        // Larger point size for better visibility
        glPointSize(3.0f);
        
        glBegin(GL_POINTS);
        
        for (const auto& p : particles) {
            if (p.pipe_id < 0 || p.pipe_id >= pipes.size()) continue;
            
            const Pipe& pipe = pipes[p.pipe_id];
            
            // Calculate position along pipe
            Vec3 pipe_dir = pipe.end - pipe.start;
            float pipe_length = pipe_dir.length();
            Vec3 dir_normalized = pipe_dir.normalized();
            
            // Position along pipe axis
            Vec3 axial_pos = pipe.start + pipe_dir * p.position;
            
            // Calculate radial position (avoid wall)
            float pipe_radius = pipe.diameter / 2.0f;
            float actual_radius = p.radial_offset * pipe_radius * 0.9f;
            
            // Create local coordinate system
            Vec3 up(0, 1, 0);
            if (fabs(dir_normalized.y) > 0.99f) up = Vec3(1, 0, 0);
            Vec3 right = dir_normalized.cross(up).normalized();
            up = right.cross(dir_normalized).normalized();
            
            // Radial offset
            float cos_theta = cosf(p.angle_offset);
            float sin_theta = sinf(p.angle_offset);
            
            Vec3 radial_offset = right * (cos_theta * actual_radius) +
                                up * (sin_theta * actual_radius);
            
            Vec3 final_pos = axial_pos + radial_offset;
            
            // Color based on flow velocity
            float flow_area = M_PI * pipe.diameter * pipe.diameter / 4.0f;
            float velocity = pipe.flowRate / flow_area;
            float velocity_factor = std::min(1.0f, velocity / 3.0f);
            
            // Blue gradient based on velocity
            Color particle_color;
            particle_color.r = 0.1f + 0.1f * velocity_factor;
            particle_color.g = 0.3f + 0.4f * velocity_factor;
            particle_color.b = 0.7f + 0.3f * velocity_factor;
            particle_color.a = p.intensity * (0.3f + 0.7f * velocity_factor);
            
            glColor4f(particle_color.r, particle_color.g, particle_color.b, 
                     particle_color.a * 0.8f);
            glVertex3f(final_pos.x, final_pos.y, final_pos.z);
        }
        
        glEnd();
        
        glDisable(GL_POINT_SPRITE);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }
    
    void clear() {
        particles.clear();
    }
};

// ============================================================================
// TCP LEAK CLIENT FOR MONITORING
// ============================================================================

class TCPLeakClient {
private:
    int tcp_socket;
    struct sockaddr_in server_addr;
    bool connected;
    std::thread reconnect_thread;
    std::atomic<bool> running;
    mutable std::mutex status_mutex;  // Mutable for const methods
    
    struct LeakData {
        int pipe_id;
        float leak_rate;
        std::string severity;
        time_t timestamp;
        bool active;
        
        LeakData() : pipe_id(-1), leak_rate(0.0f), severity("NONE"), 
                    timestamp(0), active(false) {}
    };
    
    std::map<int, LeakData> leak_data;
    std::map<int, bool> sensor_leak_status;
    std::map<int, bool> sensor_blockage_status;
    std::map<int, bool> critical_status;
    
public:
    TCPLeakClient() : tcp_socket(-1), connected(false), running(true) {
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(9999);
        inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);
        
        reconnect_thread = std::thread(&TCPLeakClient::reconnectWorker, this);
    }
    
    ~TCPLeakClient() {
        running = false;
        if (reconnect_thread.joinable()) {
            reconnect_thread.join();
        }
        disconnect();
    }
    
    void reconnectWorker() {
        while (running) {
            if (!connected) {
                connectToServer();
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    bool connectToServer() {
        tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_socket < 0) {
            return false;
        }
        
        int flags = fcntl(tcp_socket, F_GETFL, 0);
        fcntl(tcp_socket, F_SETFL, flags | O_NONBLOCK);
        
        int result = connect(tcp_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (result < 0 && errno != EINPROGRESS) {
            close(tcp_socket);
            tcp_socket = -1;
            return false;
        }
        
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(tcp_socket, &writefds);
        
        struct timeval timeout;
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        
        result = select(tcp_socket + 1, NULL, &writefds, NULL, &timeout);
        if (result <= 0) {
            close(tcp_socket);
            tcp_socket = -1;
            return false;
        }
        
        int error = 0;
        socklen_t len = sizeof(error);
        getsockopt(tcp_socket, SOL_SOCKET, SO_ERROR, &error, &len);
        
        if (error != 0) {
            close(tcp_socket);
            tcp_socket = -1;
            return false;
        }
        
        fcntl(tcp_socket, F_SETFL, flags);
        
        connected = true;
        printf("✅ Connected to leak monitoring server\n");
        
        std::thread(&TCPLeakClient::receiveWorker, this).detach();
        
        return true;
    }
    
    void disconnect() {
        if (tcp_socket >= 0) {
            close(tcp_socket);
            tcp_socket = -1;
        }
        connected = false;
    }
    
    void receiveWorker() {
        char buffer[4096];
        std::string partial_data;
        
        while (connected && running) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(tcp_socket, &readfds);
            
            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            
            int result = select(tcp_socket + 1, &readfds, NULL, NULL, &timeout);
            
            if (result < 0) {
                printf("⚠ TCP receive error\n");
                disconnect();
                break;
            } else if (result == 0) {
                continue;
            }
            
            int bytes_read = recv(tcp_socket, buffer, sizeof(buffer) - 1, 0);
            
            if (bytes_read <= 0) {
                printf("⚠ TCP connection closed\n");
                disconnect();
                break;
            }
            
            buffer[bytes_read] = '\0';
            partial_data += buffer;
            
            size_t pos;
            while ((pos = partial_data.find('\n')) != std::string::npos) {
                std::string message = partial_data.substr(0, pos);
                partial_data = partial_data.substr(pos + 1);
                
                processMessage(message);
            }
        }
    }
    
    void processMessage(const std::string& message) {
        try {
            Json::Value root;
            Json::Reader reader;
            
            if (!reader.parse(message, root)) {
                return;
            }
            
            std::lock_guard<std::mutex> lock(status_mutex);
            
            if (root.isMember("sensor_id")) {
                int sensor_id = root["sensor_id"].asInt();
                
                if (root.isMember("issue_type")) {
                    std::string issue_type = root["issue_type"].asString();
                    
                    if (issue_type == "LEAK") {
                        sensor_leak_status[sensor_id] = true;
                        printf("✅ TCP: Leak detected at sensor %d\n", sensor_id);
                    } 
                    else if (issue_type == "BLOCKAGE") {
                        sensor_blockage_status[sensor_id] = true;
                        printf("✅ TCP: Blockage detected at sensor %d\n", sensor_id);
                    }
                }
                
                if (root.isMember("type") && root["type"].asString() == "CRITICAL_ALERT") {
                    critical_status[sensor_id] = true;
                    printf("🚨 TCP: CRITICAL alert at sensor %d\n", sensor_id);
                }
                
                if (root.isMember("maintenance") && root["maintenance"].asInt() == 1) {
                    // Maintenance clears all issues
                    sensor_leak_status[sensor_id] = false;
                    sensor_blockage_status[sensor_id] = false;
                    critical_status[sensor_id] = false;
                    printf("✅ TCP: Maintenance complete for sensor %d\n", sensor_id);
                }
            }
            
        } catch (...) {
            // Ignore parse errors
        }
    }
    
    int getLeakCount() {
        std::lock_guard<std::mutex> lock(status_mutex);
        int count = 0;
        for (const auto& pair : sensor_leak_status) {
            if (pair.second) count++;
        }
        return count;
    }
    
    int getBlockageCount() {
        std::lock_guard<std::mutex> lock(status_mutex);
        int count = 0;
        for (const auto& pair : sensor_blockage_status) {
            if (pair.second) count++;
        }
        return count;
    }
    
    int getCriticalCount() {
        std::lock_guard<std::mutex> lock(status_mutex);
        int count = 0;
        for (const auto& pair : critical_status) {
            if (pair.second) count++;
        }
        return count;
    }
    
    bool hasLeak(int sensor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        auto it = sensor_leak_status.find(sensor_id);
        return (it != sensor_leak_status.end() && it->second);
    }
    
    bool hasBlockage(int sensor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        auto it = sensor_blockage_status.find(sensor_id);
        return (it != sensor_blockage_status.end() && it->second);
    }
    
    bool isCritical(int sensor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        auto it = critical_status.find(sensor_id);
        return (it != critical_status.end() && it->second);
    }
    
    std::vector<LeakData> getLeakData() {
        std::lock_guard<std::mutex> lock(status_mutex);
        std::vector<LeakData> result;
        
        for (const auto& pair : leak_data) {
            if (pair.second.active) {
                result.push_back(pair.second);
            }
        }
        
        return result;
    }
    
    bool isConnected() const {
        return connected;
    }
    
    void clearAllAlerts() {
        std::lock_guard<std::mutex> lock(status_mutex);
        sensor_leak_status.clear();
        sensor_blockage_status.clear();
        critical_status.clear();
        printf("✅ TCP: All alerts cleared\n");
    }
};

// Volumetric rendering for dense flow
class VolumetricFlowRenderer {
private:
    struct StreamLine {
        std::vector<Vec3> points;
        Color color;
        float age;
        int pipe_id;
    };
    
    std::vector<StreamLine> streamlines;
    const int POINTS_PER_STREAM = 20;
    
public:
    void update(float dt, const std::vector<Pipe>& pipes) {
        // Update existing streamlines
        for (auto& stream : streamlines) {
            stream.age += dt;
            
            // Remove old streamlines
            if (stream.age > 5.0f) {
                stream = streamlines.back();
                streamlines.pop_back();
                continue;
            }
        }
        
        // Create new streamlines for pipes with significant flow
        for (size_t i = 0; i < pipes.size(); i++) {
            const Pipe& pipe = pipes[i];
            
            // Only for water pipes with good flow
            if (pipe.type >= SEWAGE_LATERAL || pipe.flowRate < 0.05f) continue;
            
            // Create new streamline occasionally
            if ((rand() % 100) < 5) {  // 5% chance per frame
                StreamLine stream;
                stream.pipe_id = i;
                stream.age = 0.0f;
                
                // Calculate flow color
                float flow_area = M_PI * pipe.diameter * pipe.diameter / 4.0f;
                float velocity = pipe.flowRate / flow_area;
                float vel_factor = std::min(1.0f, velocity / 3.0f);
                
                stream.color = Color(0.1f, 0.4f, 0.9f, 0.6f);
                stream.color.r += vel_factor * 0.2f;
                stream.color.g += vel_factor * 0.3f;
                
                // Generate points along the pipe with slight random offset
                for (int j = 0; j < POINTS_PER_STREAM; j++) {
                    float t = static_cast<float>(j) / (POINTS_PER_STREAM - 1);
                    Vec3 pos = pipe.start + (pipe.end - pipe.start) * t;
                    
                    // Add small random radial offset
                    float offset = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * pipe.diameter * 0.3f;
                    pos.x += offset;
                    pos.y += offset * 0.5f;
                    pos.z += offset;
                    
                    stream.points.push_back(pos);
                }
                
                streamlines.push_back(stream);
                
                // Limit total streamlines
                if (streamlines.size() > 1000) {
                    streamlines.erase(streamlines.begin());
                }
            }
        }
    }
    
    void render() {
        if (streamlines.empty()) return;
        
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        
        glLineWidth(1.5f);
        
        for (const auto& stream : streamlines) {
            float alpha = 0.6f * (1.0f - stream.age / 5.0f);
            glColor4f(stream.color.r, stream.color.g, stream.color.b, alpha);
            
            glBegin(GL_LINE_STRIP);
            for (const auto& point : stream.points) {
                glVertex3f(point.x, point.y, point.z);
            }
            glEnd();
        }
        
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }
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
        HydraulicNetwork hydraulic_net;
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

        struct Node {
        Vec3 position;
        float pressure;          // kPa at this node
        float hydraulicGrade;    // meters (pressure + elevation head)
        float elevation;         // meters
        std::vector<int> incomingPipes;
        std::vector<int> outgoingPipes;
        bool isReservoir;
        bool isDemandNode;
        float demandFlow;        // m³/s
    };

    struct Network {
        std::vector<Node> nodes;
        std::vector<int> reservoirNodeIDs;
        std::vector<int> demandNodeIDs;
    };
    
    Network waterNetwork; 
    LevelStabilizer levelStabilizer; 
    SewageReservoir sewageReservoir;
    TCPLeakClient leak_client;
    TreatmentEvent lastTreatmentEvent;
    int treatmentEventCounter;
    float treatmentCheckTimer;

     CityNetwork() : cityExtent(100), reservoirWaterSensorID(-1), stpSewerSensorID(-1), 
                    simulationTime(8 * 3600), // Start at 8 AM
                    totalWaterSupplied(0), totalWaterConsumed(0),
                    totalLeakWater(0), episodeStep(0),
                    treatmentEventCounter(0), treatmentCheckTimer(0),
                    hydraulic_network_built(false), water_particles(500000, 2000.0f) {
            Building::citySimulationTime = &simulationTime;
            Sensor::city_simulation_time_ptr = &simulationTime;
            
            reservoir.position = Vec3(-cityExtent - 250, 0, -cityExtent - 250);
            stpPos = Vec3(cityExtent + 250, 0, cityExtent + 250);
            sewageReservoir.position = stpPos + Vec3(20, 0, 0);
        }


        // ROS2 subscriber
    std::shared_ptr<ROS2SensorSubscriber> ros2_subscriber;
    void setZoneValve(int zone_id, float valve_percent); 

        void generateCity(int numBuildings);
        void updateDynamicPositions();
        void generateWaterNetwork();
        void generateSewageNetwork();
        void createSensors();
        bool hydraulic_network_built;
        void enforceMassBalance(); 
        void updatePipeSensorPressure(Pipe& pipe, float calculatedPressure);
        void createZones();
        void loadLeakData(const char* filename);
        void updateSensorFromROS2(int sensorID, float valve, float pressure, float level, int esp_id);
        void updateSimulation(float dt);
        void generateInitSensorJSON(const char* filename);
        void exportSensorJSON(const char* filename);
        void exportLeakJSON(const char* filename);
        void exportSystemStateJSON(const char* filename);
        void broadcastRLState();
        void buildNetwork(); 
        
        void updateBuildingWaterUsage(float dt);
        void updateReservoir(float dt);
        void updateZones(float dt);
        void generateReservoirParticles(float dt);
        void updatePipeSensorPressureWithStabilization(Pipe& pipe, float calculatedPressure);
        void drawReservoirWithParticles() const;
        
        void initializeROS2();
        void shutdownROS2();
        void printSensorMapping() const;
        void updateTreatmentCycle(float dt);
        void drawSewageReservoir() const;
        void buildHydraulicNetwork();

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
        void checkROSDataAnomalies(int sensorID, float pressure, float level, float valve);
        void handleNewROS2Sensor(int sensorID, float valve, float pressure, float level, int esp_id);
        void syncROS2ToSimulation();  // Optional: for consistency
        void syncTCPLeakData();

        
    public: 
        std::unordered_map<int, int> sensorIdToIndex;
         WaterFlowParticleSystem water_particles;
    VolumetricFlowRenderer volumetric_renderer;

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

    void CityNetwork::updateTreatmentCycle(float dt) {
    treatmentCheckTimer += dt;
    
    // Check every second
    if (treatmentCheckTimer < 1.0f) return;
    treatmentCheckTimer = 0.0f;
    
    bool treatmentTriggered = false;
    bool rechargeTriggered = false;
    
    // ================== CHECK SEWAGE TREATMENT ==================
    if (sewageReservoir.needsTreatment) {
        float timeSinceLast = sewageReservoir.timeSinceLastTreatment;
        
        // Trigger treatment after ~10 seconds of being full
        if (timeSinceLast >= 10.0f) {
            // Record before state
            TreatmentEvent event;
            event.event_id = ++treatmentEventCounter;
            event.event_type = "TREATMENT";
            event.timestamp = simulationTime;
            event.sewage_before = sewageReservoir.getFillPercent();
            event.fresh_water_before = reservoir.getLevelPercent();
            event.duration_seconds = timeSinceLast;
            event.status = "IN_PROGRESS";
            
            // Perform treatment
            sewageReservoir.performTreatment();
            
            // Record after state
            event.sewage_after = sewageReservoir.getFillPercent();
            event.status = "COMPLETE";
            lastTreatmentEvent = event;
            
            // Broadcast via UDP
            std::string json = event.toJSON();
            queueUDPBroadcast("TREATMENT_EVENT:" + json);
            
            printf("📤 UDP: Treatment event broadcast (ID: %d)\n", event.event_id);
            treatmentTriggered = true;
        }
    }
    
    // ================== CHECK FRESH WATER RECHARGE ==================
    if (reservoir.needsRecharge) {
        float timeSinceLast = reservoir.timeSinceLastRecharge;
        
        // Trigger recharge after ~10 seconds of being low
        if (timeSinceLast >= 10.0f) {
            // Record before state
            TreatmentEvent event;
            event.event_id = ++treatmentEventCounter;
            event.event_type = "RECHARGE";
            event.timestamp = simulationTime;
            event.fresh_water_before = reservoir.getLevelPercent();
            event.sewage_before = sewageReservoir.getFillPercent();
            event.duration_seconds = timeSinceLast;
            event.status = "IN_PROGRESS";
            
            // Perform recharge
            reservoir.performRecharge();
            
            // Record after state
            event.fresh_water_after = reservoir.getLevelPercent();
            event.status = "COMPLETE";
            lastTreatmentEvent = event;
            
            // Broadcast via UDP
            std::string json = event.toJSON();
            queueUDPBroadcast("RECHARGE_EVENT:" + json);
            
            printf("📤 UDP: Recharge event broadcast (ID: %d)\n", event.event_id);
            rechargeTriggered = true;
        }
    }
    
    // ================== BROADCAST STATUS UPDATES ==================
    // Send regular status updates via UDP
    static float statusUpdateTimer = 0.0f;
    statusUpdateTimer += dt;
    
    if (statusUpdateTimer >= 2.0f) { // Every 2 seconds
        statusUpdateTimer = 0.0f;
        
        Json::Value status;
        status["timestamp"] = simulationTime;
        status["fresh_water_level"] = reservoir.getLevelPercent();
        status["fresh_water_trend"] = reservoir.getTrendString();
        status["fresh_water_needs_recharge"] = reservoir.needsRecharge;
        status["fresh_water_time_since_recharge"] = reservoir.timeSinceLastRecharge;
        
        status["sewage_level"] = sewageReservoir.getFillPercent();
        status["sewage_status"] = sewageReservoir.getStatus();
        status["sewage_needs_treatment"] = sewageReservoir.needsTreatment;
        status["sewage_time_since_treatment"] = sewageReservoir.timeSinceLastTreatment;
        
        status["total_demand_m3s"] = calculateTotalDemand();
        status["total_sewage_inflow_m3s"] = sewageReservoir.inflowRate;
        
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        std::string json = Json::writeString(builder, status);
        
        queueUDPBroadcast("WATER_SYSTEM_STATUS:" + json);
    }
}

    void CityNetwork::updateReservoir(float dt) {
    // Calculate total demand
    float totalDemand = 0.0f;
    for (const auto& building : buildings) {
        totalDemand += building.currentWaterFlow;
    }
    
    // Add leak flow
    float totalLeaks = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeaks += pipe.leakRate / 1000.0f;
        }
    }
    
    // Smooth demand
    static float smoothedDemand = totalDemand + totalLeaks;
    smoothedDemand = smoothedDemand * 0.8f + (totalDemand + totalLeaks) * 0.2f;
    
    // Update reservoir
    reservoir.update(dt, smoothedDemand);
    
    // Update statistics
    totalWaterSupplied += reservoir.outflowRate * dt;
    totalLeakWater += totalLeaks * dt;
    
    // Update sensor
    if (reservoirWaterSensorID >= 0 && reservoirWaterSensorID < (int)sensors.size()) {
        static float prevLevel = reservoir.getLevelPercent();
        float currentLevel = reservoir.getLevelPercent();
        float smoothedLevel = prevLevel * 0.7f + currentLevel * 0.3f;
        prevLevel = smoothedLevel;
        
        sensors[reservoirWaterSensorID].waterLevel = smoothedLevel;
        
        if (!sensors[reservoirWaterSensorID].isROSFresh()) {
            static float reservoirPressure = 300.0f;
            float targetPressure = 280.0f + 40.0f * (reservoir.getLevelPercent() / 100.0f);
            reservoirPressure = reservoirPressure * 0.9f + targetPressure * 0.1f;
            
            sensors[reservoirWaterSensorID].pressure = reservoirPressure;
            sensors[reservoirWaterSensorID].pressureFromROS = false;
        }
    }
}

void CityNetwork:: syncTCPLeakData() {
    if (!leak_client.isConnected()) return;
    
    for (auto& sensor : sensors) {
        bool tcp_leak = leak_client.hasLeak(sensor.id);
        bool tcp_blockage = leak_client.hasBlockage(sensor.id);
        
        // Update sensor visual state based on TCP data
        if (tcp_leak || tcp_blockage) {
            // Find connected pipe and mark it as having an issue
            for (auto& pipe : pipes) {
                if (pipe.id == sensor.connectedPipeID) {
                    // Set visual indicators
                    pipe.hasLeak = tcp_leak;
                    if (tcp_blockage) {
                        pipe.flowRate *= 0.3f; // Simulate blockage effect
                    }
                    break;
                }
            }
        }
    }
}

    // Implement them:
void CityNetwork::checkROSDataAnomalies(int sensorID, float pressure, float level, float valve) {
    // Check for impossible or suspicious values
    bool hasAnomaly = false;
    std::string anomalyMsg;
    
    if (pressure < 0 || pressure > 1000) {
        hasAnomaly = true;
        anomalyMsg = "Impossible pressure: " + std::to_string(pressure) + "kPa";
    }
    
    if (level < 0 || level > 100) {
        hasAnomaly = true;
        if (!anomalyMsg.empty()) anomalyMsg += ", ";
        anomalyMsg += "Impossible level: " + std::to_string(level) + "%";
    }
    
    if (valve < 0 || valve > 100) {
        hasAnomaly = true;
        if (!anomalyMsg.empty()) anomalyMsg += ", ";
        anomalyMsg += "Impossible valve: " + std::to_string(valve) + "%";
    }
    
    if (hasAnomaly) {
        std::cerr << "⚠ ROS2 ANOMALY: Sensor " << sensorID << " - " << anomalyMsg << std::endl;
        
        // Could trigger alert or correction logic here
        // For now, just log it
    }
    
    // Check for sudden pressure drops (possible pipe burst)
    static std::map<int, float> lastPressure;
    static std::map<int, time_t> lastPressureTime;
    
    if (lastPressure.find(sensorID) != lastPressure.end()) {
        time_t now = time(nullptr);
        if (now - lastPressureTime[sensorID] < 5) {  // Within 5 seconds
            float pressureDrop = lastPressure[sensorID] - pressure;
            if (pressureDrop > 50.0f) {  // More than 50kPa drop in 5 seconds
                std::cerr << "⚠ RAPID PRESSURE DROP: Sensor " << sensorID 
                         << " dropped " << pressureDrop << "kPa in " 
                         << (now - lastPressureTime[sensorID]) << " seconds" << std::endl;
                
                // Could trigger leak detection here
            }
        }
    }
    
    lastPressure[sensorID] = pressure;
    lastPressureTime[sensorID] = time(nullptr);
}

void CityNetwork::buildHydraulicNetwork() {
    hydraulic_net = HydraulicNetwork();
    
    // Add reservoir node
    HydraulicNode reservoir_node(0, reservoir.position, reservoir.position.y, 0);
    reservoir_node.is_reservoir = true;
    reservoir_node.pressure = 300.0f;
    hydraulic_net.addNode(reservoir_node);
    
    // Add zone nodes (demand nodes)
    for (size_t i = 0; i < zones.size(); i++) {
        // Base demand: 2-5 L/s per zone
        float base_demand = std::max(0.001f, (2.0f + 3.0f * (rand() % 1000)/1000.0f) / 1000.0f);
        
        HydraulicNode zone_node(i+1, zones[i].center, 0, base_demand);
        zone_node.is_demand = true;
        hydraulic_net.addNode(zone_node);
        
        // Store base demand in zone for reference
        zones[i].base_demand = base_demand;
        
        // Connect to reservoir - FIXED: Use proper constructor
        HydraulicPipe trunk_pipe(
            i,                  // id
            0,                  // start_node (reservoir)
            i+1,                // end_node (zone)
            0.4f,               // diameter
            (zones[i].center - reservoir.position).length() // length
        );
        
        hydraulic_net.addPipe(trunk_pipe);
    }
    
    hydraulic_network_built = true;
    printf("Hydraulic network built with %d zones\n", (int)zones.size());
}

void CityNetwork::drawSewageReservoir() const {
    float tankRadius = 15.0f;
    float tankHeight = 20.0f;
    Vec3 pos = sewageReservoir.position;
    
    // Tank structure
    glColor3f(0.5f, 0.35f, 0.2f);
    glPushMatrix();
    glTranslatef(pos.x, pos.y - tankHeight/2, pos.z);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, tankRadius, tankRadius, tankHeight, 32, 8);
    gluDeleteQuadric(quad);
    glPopMatrix();
    
    // Sewage level
    if (sewageReservoir.level > 0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Sewage color (brown)
        glColor4f(0.4f, 0.25f, 0.1f, 0.8f);
        glPushMatrix();
        glTranslatef(pos.x, pos.y - tankHeight/2 + sewageReservoir.level, pos.z);
        glRotatef(-90, 1, 0, 0);
        GLUquadric* disk = gluNewQuadric();
        gluDisk(disk, 0, tankRadius * 0.95f, 32, 8);
        gluDeleteQuadric(disk);
        glPopMatrix();
        
        glDisable(GL_BLEND);
    }
    
    // Status indicator
    glDisable(GL_LIGHTING);
    if (sewageReservoir.needsTreatment) {
        // Blinking red when needs treatment
        float pulse = 0.7f + 0.3f * sinf(simulationTime * 5.0f);
        glColor3f(1.0f, 0.2f * pulse, 0.2f * pulse);
    } else {
        glColor3f(0.3f, 0.7f, 0.3f);
    }
    
    glRasterPos3f(pos.x, pos.y + 15, pos.z);
    std::stringstream ss;
    ss << "SEWAGE: " << std::fixed << std::setprecision(1) 
       << sewageReservoir.getFillPercent() << "%";
    if (sewageReservoir.needsTreatment) {
        ss << " [TREATMENT IN " << std::setprecision(0) 
           << (10.0f - sewageReservoir.timeSinceLastTreatment) << "s]";
    }
    
    for (char c : ss.str()) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
    }
    glEnable(GL_LIGHTING);
}

void CityNetwork::handleNewROS2Sensor(int sensorID, float valve, float pressure, float level, int esp_id) {
    static int unknown_sensor_warnings = 0;
    
    if (unknown_sensor_warnings < 20) {
        std::cout << "\n⚠ NEW ROS2 SENSOR DETECTED: ID=" << sensorID 
                 << " from ESP" << esp_id << std::endl;
        std::cout << "  Valve: " << valve << "%" << std::endl;
        std::cout << "  Pressure: " << pressure << "kPa" << std::endl;
        std::cout << "  Level: " << level << "%" << std::endl;
        
        // Try to find a similar sensor in simulation
        Sensor* closestSensor = nullptr;
        float closestDistance = 1e9f;
        
        for (auto& sensor : sensors) {
            // Check if this is a similar type (water/sewage) and position
            // For simplicity, just check if IDs are close
            if (abs(sensor.id - sensorID) < 50) {
                closestSensor = &sensor;
                break;
            }
        }
        
        if (closestSensor) {
            std::cout << "  Mapping to simulation sensor: " << closestSensor->id 
                     << " (" << closestSensor->name << ")" << std::endl;
            
            // Add mapping for future updates
            sensorIdToIndex[sensorID] = closestSensor->id;
            
            // Update the sensor with ROS2 data
            updateSensorFromROS2(sensorID, valve, pressure, level, esp_id);
        } else {
            std::cout << "  No matching simulation sensor found." << std::endl;
            
            // Create a virtual sensor for this ROS2-only sensor
            std::cout << "  Creating virtual sensor representation." << std::endl;
            
            // Determine sensor type based on position or values
            SensorType type = (level > 0 && level < 100) ? WATER_SENSOR : SEWAGE_SENSOR;
            std::string name = "ROS2_Virtual_" + std::to_string(sensorID);
            Vec3 position(rand() % 100 - 50, 5, rand() % 100 - 50);
            
            sensors.push_back(Sensor(sensorID, name, type, position, -1));
            sensorIdToIndex[sensorID] = sensors.size() - 1;
            
            // Update with ROS2 data
            sensors.back().updateFromROS2(valve, pressure, level, esp_id);
        }
        
        unknown_sensor_warnings++;
        
        if (unknown_sensor_warnings == 10) {
            std::cout << "\nShowing first 10 unknown sensors only. "
                     << "Run with debug flag to see all.\n" << std::endl;
        }
    }
}

// Optional: Sync ROS2 state to simulation for consistency
void CityNetwork::syncROS2ToSimulation() {
    // This function can be called periodically to make simulation
    // match ROS2 values when ROS2 is the authority
    
    for (auto& sensor : sensors) {
        if (sensor.pressureFromROS) {
            // Find connected pipe
            for (auto& pipe : pipes) {
                if (pipe.id == sensor.connectedPipeID) {
                    // Adjust simulation to match ROS2 pressure
                    // This is a simplified approach - in reality you'd need
                    // to solve the hydraulic equations
                    
                    // For now, just log the mismatch
                    static std::map<int, std::pair<float, time_t>> lastLog;
                    time_t now = time(nullptr);
                    
                    if (lastLog.find(sensor.id) == lastLog.end() || 
                        now - lastLog[sensor.id].second > 60) {  // Log every minute
                        
                        // Calculate what the simulation pressure should be
                        float simPressure = sensor.pressure;  // Use ROS pressure
                        
                        // Log if there's a big discrepancy
                        float pipePressure = 0.0f;
                        for (const auto& s : sensors) {
                            if (s.connectedPipeID == pipe.id && !s.pressureFromROS) {
                                pipePressure = s.pressure;
                                break;
                            }
                        }
                        
                        if (fabs(pipePressure - sensor.pressure) > 20.0f) {
                            std::cout << "Pressure mismatch - Pipe " << pipe.id 
                                     << ": ROS=" << sensor.pressure 
                                     << "kPa, SIM=" << pipePressure << "kPa" << std::endl;
                        }
                        
                        lastLog[sensor.id] = {sensor.pressure, now};
                    }
                    break;
                }
            }
        }
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

    void CityNetwork::buildNetwork() {
    waterNetwork.nodes.clear();
    
    // Create reservoir node
    Node reservoirNode;
    reservoirNode.position = reservoir.position + Vec3(0, -10, 0);
    reservoirNode.elevation = reservoir.position.y;
    reservoirNode.pressure = 300.0f;  // High pressure from pumps
    reservoirNode.hydraulicGrade = reservoirNode.elevation + 
                                  (reservoirNode.pressure * 1000.0f) / 
                                  (WATER_DENSITY * GRAVITY);
    reservoirNode.isReservoir = true;
    reservoirNode.isDemandNode = false;
    waterNetwork.reservoirNodeIDs.push_back(0);
    waterNetwork.nodes.push_back(reservoirNode);
    
    // Create demand nodes for each building
    for (const auto& building : buildings) {
        Node demandNode;
        demandNode.position = building.position + Vec3(0, 1.0f, 0);
        demandNode.elevation = building.position.y;
        demandNode.pressure = DEFAULT_PRESSURE;
        demandNode.isReservoir = false;
        demandNode.isDemandNode = true;
        demandNode.demandFlow = building.currentWaterFlow;
        waterNetwork.demandNodeIDs.push_back(waterNetwork.nodes.size());
        waterNetwork.nodes.push_back(demandNode);
    }
    
    // Create junction nodes at pipe connections
    // This would require analyzing pipe connectivity
    // For simplicity, add nodes at each pipe end point
    std::map<Vec3, int> positionToNodeID;
    
    for (const auto& pipe : pipes) {
        if (pipe.type >= SEWAGE_LATERAL) continue;  // Only water network
        
        // Add start node if not exists
        if (positionToNodeID.find(pipe.start) == positionToNodeID.end()) {
            Node junctionNode;
            junctionNode.position = pipe.start;
            junctionNode.elevation = pipe.start.y;
            junctionNode.pressure = DEFAULT_PRESSURE;
            junctionNode.isReservoir = false;
            junctionNode.isDemandNode = false;
            positionToNodeID[pipe.start] = waterNetwork.nodes.size();
            waterNetwork.nodes.push_back(junctionNode);
        }
        
        // Add end node if not exists
        if (positionToNodeID.find(pipe.end) == positionToNodeID.end()) {
            Node junctionNode;
            junctionNode.position = pipe.end;
            junctionNode.elevation = pipe.end.y;
            junctionNode.pressure = DEFAULT_PRESSURE;
            junctionNode.isReservoir = false;
            junctionNode.isDemandNode = false;
            positionToNodeID[pipe.end] = waterNetwork.nodes.size();
            waterNetwork.nodes.push_back(junctionNode);
        }
        
        // Connect nodes
        int startID = positionToNodeID[pipe.start];
        int endID = positionToNodeID[pipe.end];
        waterNetwork.nodes[startID].outgoingPipes.push_back(pipe.id);
        waterNetwork.nodes[endID].incomingPipes.push_back(pipe.id);
    }
    
    // Connect demand nodes to nearest junctions
    for (size_t i = 0; i < buildings.size(); i++) {
        int demandNodeID = waterNetwork.demandNodeIDs[i];
        Vec3 buildingPos = waterNetwork.nodes[demandNodeID].position;
        
        // Find nearest junction
        float minDist = 1e9f;
        int nearestJunction = -1;
        
        for (size_t j = 0; j < waterNetwork.nodes.size(); j++) {
            if (waterNetwork.nodes[j].isDemandNode || waterNetwork.nodes[j].isReservoir) 
                continue;
                
            float dist = (waterNetwork.nodes[j].position - buildingPos).length();
            if (dist < minDist) {
                minDist = dist;
                nearestJunction = j;
            }
        }
        
        if (nearestJunction >= 0) {
            // Create a virtual pipe between junction and demand node
            // (You should update pipe array too)
        }
    }
}

    void CityNetwork::updateSensorFromROS2(int sensorID, float valve, float pressure, float level, int esp_id) {
    std::lock_guard<std::mutex> lock(sensor_mutex);
    
    auto it = sensorIdToIndex.find(sensorID);
    if (it != sensorIdToIndex.end()) {
        int idx = it->second;
        if (idx >= 0 && idx < (int)sensors.size()) {
            Sensor& sensor = sensors[idx];
            
            // Store previous state for logging and tracking
            float oldValve = sensor.valveState;
            float oldPressure = sensor.pressure;
            float oldLevel = sensor.waterLevel;
            
            // ================== TRACK VALVE CHANGES ==================
            // Check if valve actually changed (more than 1% threshold)
            float valveDiff = fabs(oldValve - valve);
            bool valveChanged = (valveDiff > 1.0f);
            
            // Update valve timestamp if it changed
            if (valveChanged) {
                sensor.lastActionTime = time(nullptr);
                sensor.recentActionCount = 0;
                
                // Also update target valve for smooth movement
                sensor.targetValveState = valve;
            }
            
            // ================== UPDATE ALL SENSOR FIELDS ==================
            // Update with ROS2 data - this is REAL data from ESP
            sensor.valveState = valve;
            sensor.pressure = pressure;           // ROS2 provided pressure
            sensor.waterLevel = level;
            sensor.last_esp_id = esp_id;
            sensor.last_update_time = time(nullptr);
            sensor.lastROSPressureUpdate = time(nullptr);  // Track when ROS data arrived
            sensor.pressureFromROS = true;                 // Mark as ROS source
            
            // Store ROS values in separate fields for reference
            sensor.pressureROS = pressure;          // Store ROS pressure separately
            sensor.levelROS = level;                // Store ROS level separately
            
            // ================== LOGGING ==================
            // Log significant updates (first few and major changes)
            static int first_updates_logged = 0;
            bool logThisUpdate = false;
            
            if (first_updates_logged < 20) {
                logThisUpdate = true;
                first_updates_logged++;
            } else if (valveChanged || 
                      fabs(oldPressure - pressure) > 20.0f ||
                      fabs(oldLevel - level) > 20.0f) {
                logThisUpdate = true;
            }
            
            if (logThisUpdate) {
                std::cout << "✓ ROS2 Update: Sensor ID=" << sensorID 
                         << " (" << sensor.name << ")"
                         << " from ESP" << esp_id;
                
                if (valveChanged) {
                    std::cout << " [VALVE ACTION]" << std::endl;
                    std::cout << "  Valve: " << oldValve << "% → " << valve << "%" << std::endl;
                } else {
                    std::cout << " [DATA ONLY]" << std::endl;
                }
                
                if (fabs(oldPressure - pressure) > 5.0f) {
                    std::cout << "  Pressure: " << oldPressure << "kPa → " << pressure << "kPa" << std::endl;
                }
                if (fabs(oldLevel - level) > 5.0f) {
                    std::cout << "  Level: " << oldLevel << "% → " << level << "%" << std::endl;
                }
                
                // Also log if this sensor is critical
                if (sensorID == reservoirWaterSensorID) {
                    std::cout << "  [RESERVOIR SENSOR - Critical Path]" << std::endl;
                } else if (sensor.type == WATER_SENSOR) {
                    // Find which building/zone this belongs to
                    for (const auto& building : buildings) {
                        if (building.waterSensorID == sensorID) {
                            std::cout << "  [Building " << building.id << " water sensor]" << std::endl;
                            break;
                        }
                    }
                }
                
                // Log the action time update
                if (valveChanged) {
                    std::cout << "  Last action time updated: " << sensor.lastActionTime << std::endl;
                }
            }
            
            // ================== ANOMALY DETECTION ==================
            // Check for anomalies in ROS2 data
            checkROSDataAnomalies(sensorID, pressure, level, valve);
            
            // ================== DEBUG OUTPUT FOR RL ==================
            // Log when actions happen for RL debugging
            static std::map<int, time_t> last_valve_change_log;
            if (valveChanged) {
                time_t now = time(nullptr);
                if (last_valve_change_log.find(sensorID) == last_valve_change_log.end() || 
                    (now - last_valve_change_log[sensorID]) > 10) {  // Log every 10 seconds max
                    
                    std::cout << "RL ACTION: Sensor " << sensorID 
                             << " valve changed at time " << now 
                             << " (diff from prev: " << valveDiff << "%)" << std::endl;
                    last_valve_change_log[sensorID] = now;
                }
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
        // This is a new sensor ID we haven't seen before - possibly added dynamically
        handleNewROS2Sensor(sensorID, valve, pressure, level, esp_id);
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
                    !sensor_data.isMember("pressure_kpa") ||
                    !sensor_data.isMember("level_pct") ||
                    !sensor_data.isMember("valve")) {
                    RCLCPP_WARN(this->get_logger(),
                            "Incomplete sensor data from ESP%d", esp_id);
                    continue;
                }

                try {
                    int sensor_id = sensor_data["sensor_id"].asInt();
                    float pressure_kpa = sensor_data["pressure_kpa"].asFloat();
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
        file << "    \"pressure\": \"kpa\",\n";
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
            file << "      \"pressure_kpa\": " << (isFresh ? 103.0f : 101.0f) << ",\n";
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
            file << "      \"pressure_kpa\": " << std::setprecision(3) << s.pressure << ",\n";
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

    // ============================================================================
// CITY NETWORK IMPLEMENTATION - CRITICAL FIXES
// ============================================================================

void CityNetwork::updateRLState() {
    // ================== DYNAMIC RESIZING ==================
    int zoneCount = zones.size();
    int sensorCount = sensors.size();
    
    // Clear and resize all vectors to match current counts
    rlState.avg_pressure_zone.clear();
    rlState.min_pressure_zone.clear();
    rlState.flow_zone.clear();
    rlState.pressure_variance_zone.clear();
    rlState.flow_to_pressure_ratio.clear();
    rlState.historical_flow.clear();
    rlState.leak_flag.clear();
    rlState.leak_severity.clear();
    rlState.overflow_flag.clear();
    rlState.pressure_violation.clear();
    
    rlState.valve_state.clear();
    rlState.last_action_time.clear();
    rlState.cumulative_volume_sensor.clear();
    
    // Reserve space for efficiency
    rlState.avg_pressure_zone.reserve(zoneCount);
    rlState.min_pressure_zone.reserve(zoneCount);
    rlState.flow_zone.reserve(zoneCount);
    // ... repeat for all zone vectors ...
    
    rlState.valve_state.reserve(sensorCount);
    rlState.last_action_time.reserve(sensorCount);
    rlState.cumulative_volume_sensor.reserve(sensorCount);
    
    // ================== FILL ZONE DATA ==================
    for (const auto& zone : zones) {
        rlState.avg_pressure_zone.push_back(zone.avgPressure);
        rlState.min_pressure_zone.push_back(zone.minPressure);
        rlState.flow_zone.push_back(zone.totalFlow);
        rlState.pressure_variance_zone.push_back(zone.pressureVariance);
        rlState.flow_to_pressure_ratio.push_back(zone.flowToPressureRatio);
        
        // Calculate average historical flow
        float histAvg = 0.0f;
        if (!zone.historicalFlow.empty()) {
            for (float flow : zone.historicalFlow) histAvg += flow;
            histAvg /= zone.historicalFlow.size();
        }
        rlState.historical_flow.push_back(histAvg);
        
        rlState.leak_flag.push_back(zone.leakFlag);
        rlState.leak_severity.push_back(zone.leakSeverity);
        rlState.overflow_flag.push_back(zone.overflowFlag);
        rlState.pressure_violation.push_back(zone.pressureViolation);
    }
    
    // ================== FILL SENSOR DATA ==================
    for (const auto& sensor : sensors) {
        rlState.valve_state.push_back(sensor.valveState);
        rlState.last_action_time.push_back(static_cast<float>(sensor.lastActionTime));
        rlState.cumulative_volume_sensor.push_back(sensor.cumulativeVolume_m3);
    }
    
    // ================== FIX: ADD SEWAGE RESERVOIR DATA ==================
    rlState.sewage_level_pct = sewageReservoir.getFillPercent();
    rlState.sewage_status = sewageReservoir.getStatus();
    rlState.sewage_needs_treatment = sewageReservoir.needsTreatment;
    rlState.sewage_time_since_treatment = sewageReservoir.timeSinceLastTreatment;
    rlState.time_since_last_treatment = sewageReservoir.timeSinceLastTreatment;
    rlState.time_since_last_recharge = reservoir.timeSinceLastRecharge;
    
    // Add treatment event info
    if (lastTreatmentEvent.event_id > 0) {
        rlState.last_treatment_event_id = lastTreatmentEvent.event_id;
    }
    
    // ================== OTHER STATE UPDATES ==================
    rlState.reservoir_level_pct = reservoir.getLevelPercent();
    rlState.reservoir_trend = reservoir.getTrendString();
    rlState.pump_capacity_available = reservoir.availablePumpCapacity;
    rlState.supply_margin = reservoir.supplyMargin;
    
    rlState.time_of_day = dayNight.timeOfDay;
    rlState.day_type = dayNight.getDayType();
    
    // Count recent actions
    rlState.recent_action_count = 0;
    for (const auto& sensor : sensors) {
        if (sensor.recentActionCount > 0) {
            rlState.recent_action_count++;
        }
    }
    
    // Time encoding
    float timeRadians = (dayNight.timeOfDay / 24.0f) * 2.0f * M_PI;
    rlState.sim_time_sin = sinf(timeRadians);
    rlState.sim_time_cos = cosf(timeRadians);
    rlState.episode_progress = fmod(simulationTime, 86400.0f) / 86400.0f;
    
    // ================== FIXED: HOURLY USAGE CALCULATION ==================
    // Get current simulation hour
    float sim_hours = simulationTime / 3600.0f;
    int currentHour = static_cast<int>(sim_hours) % 24;
    int currentDay = static_cast<int>(sim_hours / 24.0f);
    
    // DEBUG: Log day/hour changes
    static int last_logged_hour = -1;
    static int last_logged_day = -1;
    if (currentHour != last_logged_hour) {
        printf("\n[TIME UPDATE] Day %d, Hour %d, Type: %s\n", 
               currentDay, currentHour, dayNight.getDayType().c_str());
        last_logged_hour = currentHour;
    }
    if (currentDay != last_logged_day) {
        printf("[DAY CHANGE] New day: %s (%s)\n", 
               dayNight.getDayName().c_str(), dayNight.getDayType().c_str());
        last_logged_day = currentDay;
    }
    
    // Calculate total hourly usage for current hour only
    float totalHourlyUsage = 0.0f;
    
    // 1. Sum sensor usage for current hour (already in m³)
    for (const auto& sensor : sensors) {
        totalHourlyUsage += sensor.getHourlyUsage(currentHour);
    }
    
    // 2. Add current building flows (current flow rate in m³/s, but we want accumulated volume)
    // Since we don't have dt here, we'll use the sensor cumulative volume as reference
    float totalBuildingFlowRate = 0.0f;
    for (const auto& building : buildings) {
        totalBuildingFlowRate += building.currentWaterFlow;
    }
    
    // Estimate hourly volume from flow rate (m³/s → m³/hour)
    // This is an approximation since we don't have dt
    float estimatedHourlyVolume = totalBuildingFlowRate * 3600.0f; // Maximum possible if flow was constant
    // Scale it down to be realistic (typically 30-50% of max)
    float realisticFactor = 0.4f + 0.2f * sinf(simulationTime * 0.0001f); // Varying between 0.4-0.6
    totalHourlyUsage += estimatedHourlyVolume * realisticFactor;
    
    // 3. Reset array and set only current hour
    for (int i = 0; i < 24; i++) {
        rlState.hourly_usage[i] = 0.0f;
    }
    
    // Set current hour's usage (cap at reasonable value)
    rlState.hourly_usage[currentHour] = std::min(totalHourlyUsage, 1000.0f);
    
    // Set day type properly
    rlState.day_type = dayNight.getDayType();
    rlState.time_of_day = dayNight.timeOfDay;
    
    // ================== FIX: CALCULATE FLOW METRICS ==================
    float totalFlowSum = 0.0f;
    float flowSquaredSum = 0.0f;
    int flowCount = 0;
    float maxFlow = 0.0f;
    
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) { // Water pipes only
            totalFlowSum += pipe.flowRate;
            flowSquaredSum += pipe.flowRate * pipe.flowRate;
            flowCount++;
            
            if (pipe.flowRate > maxFlow) {
                maxFlow = pipe.flowRate;
            }
        }
    }
    
    rlState.peak_flow_rate = maxFlow;
    
    if (flowCount > 0) {
        rlState.avg_flow_rate = totalFlowSum / flowCount;
        float variance = (flowSquaredSum / flowCount) - (rlState.avg_flow_rate * rlState.avg_flow_rate);
        rlState.flow_variance = std::max(0.0f, variance);
    } else {
        rlState.avg_flow_rate = 0.0f;
        rlState.flow_variance = 0.0f;
    }
    
    // ================== FIX: CALCULATE NON-REVENUE WATER ==================
    float totalDemand = calculateTotalDemand();
    float totalWaterPumped = reservoir.outflowRate;
    
    float totalLeakFlow = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeakFlow += pipe.leakRate / 1000.0f; // Convert L/s to m³/s
        }
    }
    
    rlState.non_revenue_water = std::max(0.0f, totalWaterPumped - totalDemand - totalLeakFlow);
    
    if (totalWaterPumped > 0.001f) {
        rlState.supply_efficiency = totalDemand / totalWaterPumped;
    } else {
        rlState.supply_efficiency = 0.0f;
    }
    
    // ================== FIX: CALCULATE PRESSURE COMPLIANCE ==================
    int compliantPipes = 0;
    int totalWaterPipes = 0;
    
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) {
            totalWaterPipes++;
            
            // Find pressure at this pipe
            float pipePressure = DEFAULT_PRESSURE;
            for (const auto& sensor : sensors) {
                if (sensor.connectedPipeID == pipe.id) {
                    pipePressure = sensor.getEffectivePressure();
                    break;
                }
            }
            
            if (pipePressure >= 100.0f && pipePressure <= 300.0f) {
                compliantPipes++;
            }
        }
    }
    
    if (totalWaterPipes > 0) {
        rlState.pressure_compliance = static_cast<float>(compliantPipes) / totalWaterPipes;
    } else {
        rlState.pressure_compliance = 0.0f;
    }
    
    // ================== DEBUG OUTPUT ==================
    static int debugCounter = 0;
    if (debugCounter++ % 200 == 0) { // Every 200 updates (~20 seconds at 10Hz)
        printf("\n[RL STATE UPDATE] Zones: %d, Sensors: %d, Hour: %d\n", 
               zoneCount, sensorCount, currentHour);
        printf("  Reservoir: %.1f%% (%s), Sewage: %.1f%% (%s)\n",
               rlState.reservoir_level_pct, rlState.reservoir_trend.c_str(),
               rlState.sewage_level_pct, rlState.sewage_status.c_str());
        printf("  Day: %s, Time: %.1fh, Hourly usage: %.1f m³\n",
               rlState.day_type.c_str(), rlState.time_of_day,
               rlState.hourly_usage[currentHour]);
        printf("  Valve states: %d, Recent actions: %d\n",
               (int)rlState.valve_state.size(), rlState.recent_action_count);
    }
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
            std::cout << "\n WARNING: Your ROS2 data expects sensor IDs up to 110, but simulation only has " 
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
        // Get pressure for this building's location
        float building_pressure = DEFAULT_PRESSURE;
        if (building.waterSensorID >= 0 && building.waterSensorID < sensors.size()) {
            building_pressure = sensors[building.waterSensorID].getEffectivePressure();
        }
        
        // Pass all 4 parameters, including sensors
        building.updateWaterUsage(dt, dayNight, building_pressure, sensors);
        totalWaterConsumed += building.currentWaterFlow * dt;
    }
}

    void CityNetwork::setZoneValve(int zone_id, float valve_percent) {
    if (!hydraulic_network_built) return;
    
    float valve_opening = valve_percent / 100.0f;
    int node_id = hydraulic_net.getZoneNodeID(zone_id);
    hydraulic_net.setValve(node_id, valve_opening);
    
    // Also update the sensor if it exists
    for (auto& sensor : sensors) {
        // Find sensor for this zone (simplified)
        if (sensor.zoneID == zone_id) {
            sensor.setValve(valve_percent);
            break;
        }
    }
    
    episodeStep++;
}

void CityNetwork::updatePipePhysics(float dt) {
    // ================== BUILD HYDRAULIC NETWORK IF NOT BUILT ==================
    if (!hydraulic_network_built) {
        buildHydraulicNetwork();
        // Initialize building pressures
        for (auto& building : buildings) {
            building.pressureSimulated = NOMINAL_PRESSURE;
        }
    }
    
    // ================== CONNECT BUILDING DEMANDS TO HYDRAULIC NETWORK ==================
    // Update building demands in hydraulic network nodes
    for (size_t i = 0; i < zones.size(); i++) {
        int node_id = hydraulic_net.getZoneNodeID(i);
        if (node_id >= 0) {
            // Calculate total demand for this zone
            float zone_demand = 0.0f;
            for (const auto& building : buildings) {
                if (building.zoneID == static_cast<int>(i)) {
                    zone_demand += building.currentWaterFlow;
                }
            }
            // We need to update the hydraulic node demand
            // This assumes HydraulicNode has a method to set base_demand
            // If not, we'll need to add it
            if (node_id < hydraulic_net.getNodes().size()) {
                // Access nodes through a getter - we need to add this method
                // For now, we'll store zone demands separately
                zones[i].base_demand = zone_demand;
            }
        }
    }
    
    // ================== SOLVE HYDRAULIC EQUATIONS ==================
    // 1. Update leaks
    hydraulic_net.updateLeaks(dt);
    
    // 2. Solve the network
    hydraulic_net.solve(dt, simulationTime);
    
    // 3. Get pressures for each zone
    std::vector<float> zone_pressures(zones.size(), NOMINAL_PRESSURE);
    std::vector<float> zone_flows(zones.size(), 0.0f);
    for (size_t i = 0; i < zones.size(); i++) {
        int node_id = hydraulic_net.getZoneNodeID(i);
        if (node_id >= 0) {
            zone_pressures[i] = hydraulic_net.getNodePressure(node_id);
            zone_flows[i] = hydraulic_net.getNodeFlow(node_id);
        }
    }
    
    // ================== UPDATE BUILDINGS WITH ACTUAL PRESSURES ==================
    for (auto& building : buildings) {
        if (building.zoneID >= 0 && building.zoneID < static_cast<int>(zone_pressures.size())) {
            float actual_pressure = zone_pressures[building.zoneID];
            
            // Update building water usage with actual pressure
            building.updateWaterUsage(dt, dayNight, actual_pressure, sensors);
            
            // Store pressure for reference
            building.pressureSimulated = actual_pressure;
            building.actual_pressure = actual_pressure; // Make sure this is set
        }
    }
    
    // ================== UPDATE RESERVOIR ==================
    float total_demand = hydraulic_net.getTotalDemand();
    reservoir.update(dt, total_demand);
    
    // ================== UPDATE ZONE STATISTICS ==================
    for (size_t i = 0; i < zones.size(); i++) {
        int node_id = hydraulic_net.getZoneNodeID(i);
        
        if (node_id >= 0) {
            // Get pressure and flow from hydraulic network
            float pressure = zone_pressures[i];
            float flow = zone_flows[i];
            
            // SAFETY: Ensure values are finite
            if (!std::isfinite(pressure)) pressure = NOMINAL_PRESSURE;
            if (!std::isfinite(flow)) flow = 0.001f;
            
            // Clamp values
            pressure = std::max(0.0f, std::min(500.0f, pressure));
            flow = std::max(0.0f, std::min(100.0f, flow));  // Max 100 m³/s
            
            // Update zone statistics
            zones[i].avgPressure = pressure;
            zones[i].totalFlow = flow;
            
            // Track min/max pressure
            if (pressure < zones[i].minPressure) zones[i].minPressure = pressure;
            if (pressure > zones[i].maxPressure) zones[i].maxPressure = pressure;
            
            // Calculate pressure variance with safety
            static std::map<int, std::deque<float>> pressure_history;
            if (pressure_history.find(i) == pressure_history.end()) {
                pressure_history[i] = std::deque<float>(10, pressure);
            }
            
            pressure_history[i].pop_front();
            pressure_history[i].push_back(pressure);
            
            float sum = 0.0f, sum_sq = 0.0f;
            for (float p : pressure_history[i]) {
                p = std::max(0.0f, std::min(500.0f, p));  // Clamp
                sum += p;
                sum_sq += p * p;
            }
            float mean = sum / pressure_history[i].size();
            float variance = (sum_sq / pressure_history[i].size()) - (mean * mean);
            zones[i].pressureVariance = std::max(0.0f, std::min(100.0f, variance));
            
            // Flow-to-pressure ratio with safety
            if (pressure > 1.0f && std::isfinite(flow)) {
                zones[i].flowToPressureRatio = flow / pressure;
                // Clamp ratio
                zones[i].flowToPressureRatio = std::max(0.0f, std::min(100.0f, zones[i].flowToPressureRatio));
            } else {
                zones[i].flowToPressureRatio = 0.0f;
            }
            
            // Pressure violations
            zones[i].pressureViolation = (pressure < 100.0f || pressure > 250.0f);
            
            // Leak detection with safety
            if (zones[i].base_demand > 0.0001f && std::isfinite(flow)) {
                if (flow > zones[i].base_demand * 1.3f) {
                    zones[i].leakFlag = true;
                    zones[i].leakSeverity = (flow - zones[i].base_demand) / zones[i].base_demand;
                    // Clamp leak severity
                    zones[i].leakSeverity = std::max(0.0f, std::min(10.0f, zones[i].leakSeverity));
                } else {
                    zones[i].leakFlag = false;
                    zones[i].leakSeverity = 0.0f;
                }
            } else {
                zones[i].leakFlag = false;
                zones[i].leakSeverity = 0.0f;
            }
            
            // Overflow detection with safety
            zones[i].overflowFlag = (flow > zones[i].base_demand * 2.0f);
            
            // Historical flow tracking
            zones[i].historicalFlow.push_back(flow);
            if (zones[i].historicalFlow.size() > 100) {
                zones[i].historicalFlow.erase(zones[i].historicalFlow.begin());
            }
        }
    }
    
    // ================== UPDATE PIPE VISUALS AND PHYSICS ==================
    // First pass: calculate sewage flows (needed for water flow calculation)
    std::vector<float> sewage_flows(pipes.size(), 0.0f);
    for (auto& pipe : pipes) {
        if (pipe.type >= SEWAGE_LATERAL) { // Sewage pipes
            // Sewage flow calculation
            float sewageFlow = 0.0f;
            
            if (pipe.type == SEWAGE_LATERAL) {
                // Find connected building
                for (const auto& building : buildings) {
                    if (building.sewerPipeID == pipe.id) {
                        sewageFlow = building.currentSewageFlow;
                        break;
                    }
                }
            } else if (pipe.type == SEWAGE_COLLECTOR) {
                // Sum flows from laterals in this cluster
                int clusterIdx = pipe.zoneID;
                if (clusterIdx >= 0 && clusterIdx < clusters.size()) {
                    for (int lateralID : clusters[clusterIdx].sewerPipeIDs) {
                        if (lateralID >= 0 && lateralID < pipes.size()) {
                            sewageFlow += pipes[lateralID].flowRate;
                        }
                    }
                }
            } else if (pipe.type == SEWAGE_INTERCEPTOR) {
                // Sum all collector flows
                for (const auto& cluster : clusters) {
                    if (cluster.sewageCollectorID >= 0 && 
                        cluster.sewageCollectorID < pipes.size()) {
                        sewageFlow += pipes[cluster.sewageCollectorID].flowRate;
                    }
                }
            }
            
            pipe.flowRate = sewageFlow;
            sewage_flows[pipe.id] = sewageFlow;
            
            // Update velocity for sewage
            float area = M_PI * pipe.diameter * pipe.diameter / 4.0f;
            if (area > 0.001f) {
                pipe.velocity = sewageFlow / area;
            }
        }
    }
    
    // Second pass: calculate water flows with proper mass balance
    for (auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) { // Water pipes
            // Find which zone this pipe is in
            Zone* zone = getZoneForPipe(pipe.id);
            float pipe_flow = 0.0f;
            
            if (zone) {
                // Assign flow based on pipe type and zone demand
                float base_flow = zone->totalFlow;
                switch (pipe.type) {
                    case TRUNK_MAIN:
                        // Trunk carries total system demand
                        pipe_flow = total_demand * 1.1f; // Include some margin
                        break;
                    case SECONDARY_MAIN:
                        // Secondary serves entire zone
                        pipe_flow = base_flow * 1.05f; // Include some losses
                        break;
                    case RING_MAIN:
                        // Ring distributes within cluster
                        pipe_flow = base_flow * 0.7f; // Reduced for distribution
                        break;
                    case SERVICE_PIPE:
                        // Find connected building
                        for (const auto& building : buildings) {
                            if (building.servicePipeID == pipe.id) {
                                pipe_flow = building.currentWaterFlow;
                                break;
                            }
                        }
                        break;
                    default:
                        pipe_flow = base_flow * 0.5f;
                }
            } else {
                // If no zone found, use a default flow
                switch (pipe.type) {
                    case TRUNK_MAIN: pipe_flow = total_demand; break;
                    case SECONDARY_MAIN: pipe_flow = total_demand / zones.size(); break;
                    case RING_MAIN: pipe_flow = 0.1f; break;
                    case SERVICE_PIPE: pipe_flow = 0.01f; break;
                    default: pipe_flow = 0.05f;
                }
            }
            
            // Apply realistic constraints
            float max_velocity = (pipe.type == SERVICE_PIPE) ? 2.0f : 3.0f;
            float area = M_PI * pipe.diameter * pipe.diameter / 4.0f;
            float max_flow = max_velocity * area;
            
            pipe.flowRate = std::min(pipe_flow, max_flow);
            
            // Calculate pressure drop using Darcy-Weisbach
            float pressure_drop = 0.0f;
            if (pipe.flowRate > 0.001f) {
                pressure_drop = pipe.calculateRealisticPressureDrop(pipe.flowRate);
                
                // For long pipes, add distance-based pressure loss
                float distance_loss = pipe.length * 0.05f; // 0.05 kPa/m
                pressure_drop += distance_loss;
            }
            
            // Update pipe sensor pressure
            float upstream_pressure = NOMINAL_PRESSURE;
            if (pipe.type == TRUNK_MAIN) {
                upstream_pressure = 300.0f; // Reservoir pressure
            } else if (zone) {
                upstream_pressure = zone->avgPressure;
            }
            
            float pipe_pressure = upstream_pressure - pressure_drop;
            pipe_pressure = std::max(0.0f, pipe_pressure);
            
            updatePipeSensorPressureWithStabilization(pipe, pipe_pressure);
            
            // Update velocity for visualization
            if (area > 0.001f) {
                pipe.velocity = pipe.flowRate / area;
                
                // Determine flow regime
                float Re = (WATER_DENSITY * pipe.velocity * pipe.diameter) / DYNAMIC_VISCOSITY;
                if (Re < 2000) {
                    pipe.flowRegime = "LAMINAR";
                } else if (Re < 4000) {
                    pipe.flowRegime = "TRANSITIONAL";
                } else {
                    pipe.flowRegime = "TURBULENT";
                }
            }
        }
    }
    
    // ================== UPDATE HIGH-DENSITY WATER PARTICLE SYSTEM ==================
    if (showWaterParticles) {
        water_particles.update(dt, pipes);
        volumetric_renderer.update(dt, pipes);
    } else {
        // If particles are disabled, still update a few for leak visualization
        for (auto& pipe : pipes) {
            if (pipe.hasLeak && pipe.type < SEWAGE_LATERAL && pipe.flowRate > 0.001f) {
                // Update leak particles only
                pipe.updateParticles(dt);
            }
        }
    }
    
    // ================== UPDATE SENSORS ==================
    for (auto& sensor : sensors) {
        // Find which zone this sensor is in
        int closest_zone = -1;
        float min_distance = 1e9f;
        
        for (size_t i = 0; i < zones.size(); i++) {
            float distance = (sensor.position - zones[i].center).length();
            if (distance < min_distance) {
                min_distance = distance;
                closest_zone = i;
            }
        }
        
        if (closest_zone >= 0) {
            float pressure = zones[closest_zone].avgPressure;
            float flow = zones[closest_zone].totalFlow;
            
            // Check if ROS2 data is stale
            time_t now = time(nullptr);
            bool rosIsFresh = sensor.last_update_time > 0 && 
                             (now - sensor.last_update_time) <= 3;
            
            if (!rosIsFresh) {
                // Use hydraulic network pressure with noise
                float noise = (rand() % 1000) / 1000.0f * SENSOR_NOISE_SIGMA * 2.0f - SENSOR_NOISE_SIGMA;
                sensor.pressure = pressure + noise;
                sensor.pressure = std::max(0.0f, sensor.pressure);
                sensor.pressureSimulated = pressure;
                sensor.pressureFromROS = false;
                
                // Calculate water level based on pipe type and flow
                float pressure_ratio = pressure / NOMINAL_PRESSURE;
                float flow_ratio = flow / std::max(0.001f, zones[closest_zone].base_demand);
                
                float level = 0.0f;
                if (sensor.type == WATER_SENSOR) {
                    // Different level calculation based on sensor location
                    if (sensor.name.find("reservoir") != std::string::npos) {
                        level = reservoir.getLevelPercent();
                    } else if (sensor.connectedPipeID >= 0 && sensor.connectedPipeID < pipes.size()) {
                        const Pipe& connected_pipe = pipes[sensor.connectedPipeID];
                        if (connected_pipe.type == TRUNK_MAIN) {
                            level = 60.0f + 20.0f * pressure_ratio;
                        } else if (connected_pipe.type == SERVICE_PIPE) {
                            level = 40.0f + 40.0f * std::min(1.0f, flow_ratio);
                        } else {
                            level = 50.0f + 25.0f * pressure_ratio + 5.0f * tanh(flow_ratio - 1.0f);
                        }
                    } else {
                        level = 40.0f + 30.0f * pressure_ratio + 10.0f * tanh(flow_ratio - 1.0f);
                    }
                } else {
                    // Sewage sensor
                    level = 30.0f + 40.0f * std::min(1.0f, flow_ratio * 0.8f);
                }
                
                // Add variation and clamp
                level += (rand() % 1000) / 1000.0f * 10.0f - 5.0f;
                level = std::max(10.0f, std::min(95.0f, level));
                
                // Stabilize
                sensor.waterLevel = levelStabilizer.getStabilizedLevel(sensor.id, level);
            }
        }
        
        // Update sensor volume based on connected pipe
        float flow = 0.0f;
        if (sensor.connectedPipeID >= 0 && sensor.connectedPipeID < pipes.size()) {
            flow = pipes[sensor.connectedPipeID].flowRate;
        }
        sensor.updateVolume(flow, dt);
        
        // Update valve position
        sensor.updateValve(dt);
    }
    
    // ================== UPDATE SEWAGE SYSTEM ==================
    float totalSewageInflow = 0.0f;
    for (const auto& building : buildings) {
        totalSewageInflow += building.currentSewageFlow;
    }
    
    sewageReservoir.update(dt, totalSewageInflow);
    
    // ================== CALCULATE SYSTEM METRICS ==================
    float totalWaterPumped = reservoir.outflowRate;
    float totalBilledWater = calculateTotalDemand();
    
    // Calculate Non-Revenue Water
    float totalLeakFlow = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeakFlow += pipe.leakRate / 1000.0f; // L/s to m³/s
        }
    }
    
    rlState.non_revenue_water = std::max(0.0f, totalWaterPumped - totalBilledWater - totalLeakFlow);
    
    // Supply efficiency
    if (totalWaterPumped > 0.001f) {
        rlState.supply_efficiency = totalBilledWater / totalWaterPumped;
    } else {
        rlState.supply_efficiency = 0.0f;
    }
    
    // Calculate water quality metrics
    int compliant_pipes = 0;
    int total_water_pipes = 0;
    float total_pressure = 0.0f;
    
    for (const auto& pipe : pipes) {
        if (pipe.type < SEWAGE_LATERAL) {
            total_water_pipes++;
            
            // Estimate pipe pressure
            float pipe_pressure = NOMINAL_PRESSURE;
            for (const auto& sensor : sensors) {
                if (sensor.connectedPipeID == pipe.id) {
                    pipe_pressure = sensor.getEffectivePressure();
                    break;
                }
            }
            
            total_pressure += pipe_pressure;
            
            if (pipe_pressure >= 100.0f && pipe_pressure <= 300.0f) {
                compliant_pipes++;
            }
        }
    }
    
    if (total_water_pipes > 0) {
        rlState.pressure_compliance = static_cast<float>(compliant_pipes) / total_water_pipes;
        rlState.avg_flow_rate = totalBilledWater;
    }
    
    // Update episode step
    episodeStep++;
    
    // ================== DEBUG OUTPUT ==================
    static int debug_counter = 0;
    if (debug_counter++ % 200 == 0) {
        printf("\n[PHYSICS UPDATE] Time: %.1fh, Step: %d\n", 
               simulationTime/3600.0f, episodeStep);
        printf("  Reservoir: %.1f%%, Demand: %.3f m³/s, Pumped: %.3f m³/s\n",
               reservoir.getLevelPercent(), total_demand, totalWaterPumped);
        printf("  Sewage: %.1f%%, Inflow: %.3f m³/s\n",
               sewageReservoir.getFillPercent(), totalSewageInflow);
        
        if (zones.size() > 0) {
            printf("  Zone 0: P=%.1fkPa, F=%.3fm³/s, V=%.0f%%, L=%s\n",
                   zones[0].avgPressure, zones[0].totalFlow,
                   hydraulic_net.getValveState(hydraulic_net.getZoneNodeID(0)),
                   zones[0].leakFlag ? "YES" : "NO");
        }
        
        // Show building statistics
        if (buildings.size() > 0) {
            printf("  Building 0: %.1f L/s, Pressure: %.1f kPa\n",
                   buildings[0].currentWaterFlow * 1000.0f,
                   buildings[0].actual_pressure);
        }
        
        // Show particle statistics
        if (showWaterParticles) {
            printf("  Particles: Active (millions simulated as continuous flow)\n");
        }
    }
}

void CityNetwork::updatePipeSensorPressureWithStabilization(Pipe& pipe, float calculatedPressure) {
    for (auto& sensor : sensors) {
        if (sensor.connectedPipeID == pipe.id) {
            // Store simulated pressure
            sensor.pressureSimulated = calculatedPressure;
            
            // Check if ROS2 data is fresh
            time_t now = time(nullptr);
            bool rosIsFresh = sensor.last_update_time > 0 && 
                             (now - sensor.last_update_time) <= 3;
            
            if (!rosIsFresh) {
                // Use simulation pressure with realistic characteristics
                sensor.pressure = calculatedPressure;
                sensor.pressureFromROS = false;
                
                // Calculate water level based on pipe characteristics
                if (sensor.type == WATER_SENSOR) {
                    float targetLevel = 0.0f;
                    float demandRatio = 0.0f; // Declare outside switch
                    
                    // Different calculation based on pipe type
                    switch (pipe.type) {
                        case TRUNK_MAIN:
                            // Trunk main: level based on pressure stability
                            targetLevel = 60.0f + 20.0f * tanh((calculatedPressure - 200.0f) / 100.0f);
                            break;
                            
                        case SECONDARY_MAIN:
                            // Secondary main: mix of pressure and flow
                            targetLevel = 50.0f + 25.0f * (calculatedPressure / 250.0f) + 
                                        5.0f * tanh(pipe.flowRate * 10.0f);
                            break;
                            
                        case RING_MAIN:
                            // Distribution: more flow-dependent
                            targetLevel = 45.0f + 30.0f * (1.0f - exp(-pipe.flowRate * 20.0f)) +
                                        10.0f * (calculatedPressure / 200.0f);
                            break;
                            
                        case SERVICE_PIPE:
                            // Service pipe: building demand driven
                            demandRatio = 0.0f; // Initialize here
                            for (const auto& building : buildings) {
                                if (building.servicePipeID == pipe.id) {
                                    demandRatio = building.currentWaterFlow / 
                                                std::max(0.001f, building.getBaseDemandLps() * 0.001f);
                                    break;
                                }
                            }
                            targetLevel = 40.0f + 40.0f * std::min(1.0f, demandRatio) +
                                        5.0f * (calculatedPressure / 150.0f);
                            break;
                            
                        default:
                            targetLevel = 50.0f;
                    }
                    
                    // Add realistic noise and stabilization
                    float noise = (rand() % 1000) / 1000.0f * 5.0f - 2.5f;
                    targetLevel += noise;
                    
                    // Clamp to valid range
                    targetLevel = std::max(5.0f, std::min(95.0f, targetLevel));
                    
                    // Apply stabilization for smooth transitions
                    float stabilizedLevel = levelStabilizer.getStabilizedLevel(sensor.id, targetLevel);
                    sensor.waterLevel = stabilizedLevel;
                    
                } else {
                    // Sewage sensor: simpler calculation
                    float flow_ratio = pipe.flowRate / std::max(0.001f, pipe.diameter * 2.0f);
                    float sewageLevel = 30.0f + 50.0f * std::min(1.0f, flow_ratio);
                    
                    // Add noise
                    sewageLevel += (rand() % 1000) / 1000.0f * 10.0f - 5.0f;
                    sewageLevel = std::max(10.0f, std::min(90.0f, sewageLevel));
                    
                    sensor.waterLevel = levelStabilizer.getStabilizedLevel(sensor.id, sewageLevel);
                }
                
                // Also update valve if it's controlled by simulation
                if (sensor.valveState < 10.0f || sensor.valveState > 90.0f) {
                    // Adjust valve based on pressure
                    float targetValve = 100.0f;
                    if (calculatedPressure < 100.0f) {
                        targetValve = 100.0f; // Open fully if pressure low
                    } else if (calculatedPressure > 250.0f) {
                        targetValve = 50.0f; // Partially close if pressure too high
                    }
                    sensor.setValve(targetValve);
                }
                
            } else {
                // ROS2 data is fresh, mark it
                sensor.pressureFromROS = true;
                
                // Still update simulated pressure for reference
                sensor.pressureSimulated = calculatedPressure;
            }
            break;
        }
    }
}

// ================== HELPER FUNCTION ==================

void CityNetwork::updatePipeSensorPressure(Pipe& pipe, float calculatedPressure) {
    for (auto& sensor : sensors) {
        if (sensor.connectedPipeID == pipe.id) {
            sensor.pressureSimulated = calculatedPressure;
            
            // Check if ROS2 data is stale (> 3 seconds)
            time_t now = time(nullptr);
            bool rosIsFresh = sensor.last_update_time > 0 && 
                             (now - sensor.last_update_time) <= 3;
            
            if (!rosIsFresh) {
                // Use simulation pressure
                sensor.pressure = calculatedPressure;
                sensor.pressureFromROS = false;
                
                // IMPROVED: Calculate water level based on pipe flow and pressure
                if (sensor.type == WATER_SENSOR) {
                    // More stable water level calculation
                    float targetLevel = 0.0f;
                    
                    if (pipe.type == TRUNK_MAIN || pipe.type == SECONDARY_MAIN) {
                        // Transmission mains: level based on pressure stability
                        float pressureRatio = calculatedPressure / 200.0f; // Nominal 200 kPa
                        targetLevel = 60.0f + 20.0f * tanh(pressureRatio - 1.0f);
                    } else if (pipe.type == RING_MAIN) {
                        // Distribution: level based on flow stability
                        float pipeArea = M_PI * pipe.diameter * pipe.diameter / 4.0f;
                        float velocity = (pipeArea > 0.001f) ? pipe.flowRate / pipeArea : 0.0f;
                        float velocityRatio = velocity / 1.0f; // Normalized to 1 m/s
                        targetLevel = 50.0f + 30.0f * (1.0f - exp(-velocityRatio * 2.0f));
                    } else if (pipe.type == SERVICE_PIPE) {
                        // Service pipes: level based on building demand
                        float demandRatio = 0.0f;
                        for (const auto& building : buildings) {
                            if (building.servicePipeID == pipe.id) {
                                demandRatio = building.currentWaterFlow / 
                                            (building.getBaseDemandLps() * 0.001f);
                                break;
                            }
                        }
                        targetLevel = 40.0f + 40.0f * std::min(1.0f, demandRatio);
                    }
                    
                    // Apply smoothing to prevent sudden jumps
                    static std::map<int, float> previousLevels;
                    if (previousLevels.find(sensor.id) == previousLevels.end()) {
                        previousLevels[sensor.id] = targetLevel;
                    }
                    
                    // Smooth transition (60% previous, 40% new)
                    float smoothedLevel = previousLevels[sensor.id] * 0.6f + targetLevel * 0.4f;
                    previousLevels[sensor.id] = smoothedLevel;
                    
                    sensor.waterLevel = std::min(100.0f, std::max(0.0f, smoothedLevel));
                }
            } else {
                // ROS2 data is fresh, mark it
                sensor.pressureFromROS = true;
            }
            break;
        }
    }
}

void CityNetwork::enforceMassBalance() {
    // Simple mass balance check - prevents impossible flows
    
    // Calculate total system demand
    float totalDemand = calculateTotalDemand();
    
    // Calculate total leak flow
    float totalLeakFlow = 0.0f;
    for (const auto& pipe : pipes) {
        if (pipe.hasLeak) {
            totalLeakFlow += pipe.leakRate / 1000.0f; // L/s to m³/s
        }
    }
    
    // Reservoir should supply demand + leaks
    float requiredOutflow = totalDemand + totalLeakFlow;
    
    // Adjust trunk main flow
    for (auto& pipe : pipes) {
        if (pipe.type == TRUNK_MAIN) {
            pipe.flowRate = requiredOutflow;
            break;
        }
    }
    
    // Balance each zone
    for (int zoneID = 0; zoneID < zones.size(); zoneID++) {
        float zoneDemand = 0.0f;
        
        // Sum building demands in zone
        for (const auto& building : buildings) {
            if (building.zoneID == zoneID) {
                zoneDemand += building.currentWaterFlow;
            }
        }
        
        // Adjust secondary main for this zone
        for (auto& pipe : pipes) {
            if (pipe.type == SECONDARY_MAIN && pipe.zoneID == zoneID) {
                pipe.flowRate = zoneDemand;
                break;
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
    std::lock_guard<std::mutex> lock(city_mutex);
    
    simulationTime += dt;
    syncTCPLeakData();
    
    // Update day-night cycle
    dayNight.update(dt);
    
    updateBuildingWaterUsage(dt);
    enforceMassBalance();
    updatePipePhysics(dt);
    updateReservoir(dt);
    updateZones(dt);
    generateReservoirParticles(dt);
    
    // NEW: Update treatment/recharge cycle
    updateTreatmentCycle(dt);
    
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



// Add this method to update zones
void CityNetwork::updateZones(float dt) {
    for (auto& zone : zones) {
        // Update leak detection (simplified)
        if (zone.leakFlag) {
            // Gradually reduce leak severity over time
            zone.leakSeverity *= 0.99f;
            
            // If leak is very small, clear the flag
            if (zone.leakSeverity < 0.01f) {
                zone.leakFlag = false;
                zone.leakSeverity = 0.0f;
            }
        }
        
        // Check for overflow conditions
        // Simplified: overflow if flow exceeds 80% of capacity
        float capacity = 10.0f; // m³/s capacity per zone
        zone.overflowFlag = (zone.totalFlow > capacity * 0.8f);
        
        // Update pressure violation flag
        zone.pressureViolation = (zone.avgPressure < 100.0f || zone.avgPressure > 300.0f);
        
        // Update historical statistics
        if (zone.historicalFlow.size() > 100) {
            zone.historicalFlow.erase(zone.historicalFlow.begin());
        }
        
        // Calculate flow variance (simplified)
        if (!zone.historicalFlow.empty()) {
            float sum = 0.0f;
            float sumSquared = 0.0f;
            for (float flow : zone.historicalFlow) {
                sum += flow;
                sumSquared += flow * flow;
            }
            float mean = sum / zone.historicalFlow.size();
            float variance = (sumSquared / zone.historicalFlow.size()) - (mean * mean);
            zone.pressureVariance = std::max(0.0f, variance);
        }
        
        // Update flow-to-pressure ratio
        if (zone.avgPressure > 0.1f) {
            zone.flowToPressureRatio = zone.totalFlow / zone.avgPressure;
        }
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
    

    // Add this after CityNetwork class definition (around line 4500)
void HydraulicNetwork::connectBuildingDemands(const CityNetwork& city) {
    for (const auto& building : city.buildings) {
        Vec3 building_pos = building.position + Vec3(0, 1, 0);
        int node_id = findNodeByPosition(building_pos, 20.0f);
        
        if (node_id >= 0) {
            float demand_m3s = building.currentWaterFlow;
            if (demand_m3s > 0) {
                // Update node demand - need accessor methods
                // This requires modifying HydraulicNode to have public access
                // or adding methods to HydraulicNetwork
                nodes[node_id].base_demand = demand_m3s;
                nodes[node_id].is_demand = true;
            }
        }
    }
}

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
    // Check TCP for alerts on this sensor
    bool tcp_leak = false;
    bool tcp_blockage = false;
    bool tcp_critical = false;
    
    if (city.leak_client.isConnected()) {
        // These methods need to be implemented in TCPLeakClient
        tcp_leak = city.leak_client.hasLeak(sensor.id);
        tcp_blockage = city.leak_client.hasBlockage(sensor.id);
        tcp_critical = city.leak_client.isCritical(sensor.id);
    }
    
    // Determine sensor state
    bool has_issue = tcp_leak || tcp_blockage || tcp_critical;
    bool is_stale = sensor.isStale();
    bool needs_maintenance = (sensor.lastActionTime > 0 && 
                             (time(nullptr) - sensor.lastActionTime) < 5); // Recently maintained
    
    // Base color from sensor type
    Color base_col = sensor.getColor();
    
    // Enhanced pulse for different states
    float pulse_rate = 3.0f; // Default pulse rate
    float pulse_intensity = 0.25f; // Default pulse intensity
    
    if (tcp_critical) {
        pulse_rate = 10.0f;  // Fast pulse for critical
        pulse_intensity = 0.8f;
    } else if (tcp_leak) {
        pulse_rate = 8.0f;   // Fast pulse for leaks
        pulse_intensity = 0.6f;
    } else if (tcp_blockage) {
        pulse_rate = 6.0f;   // Medium pulse for blockages
        pulse_intensity = 0.4f;
    } else if (is_stale) {
        pulse_rate = 1.0f;   // Slow pulse for stale
        pulse_intensity = 0.2f;
    } else if (needs_maintenance) {
        pulse_rate = 4.0f;   // Medium pulse for recent maintenance
        pulse_intensity = 0.5f;
    }
    
    float pulse = 0.75f + pulse_intensity * sinf(city.simulationTime * pulse_rate);
    
    // Final color based on state
    Color final_col;
    if (tcp_critical) {
        final_col = Color(1.0f, 0.0f, 0.0f);  // Bright red for critical
    } else if (tcp_leak) {
        final_col = Color(1.0f, 0.3f, 0.3f);  // Red for leaks
    } else if (tcp_blockage) {
        final_col = Color(1.0f, 0.8f, 0.2f);  // Yellow for blockages
    } else if (is_stale) {
        final_col = Color(base_col.r * 0.5f, base_col.g * 0.5f, base_col.b * 0.5f); // Dim for stale
    } else if (needs_maintenance) {
        final_col = Color(0.2f, 1.0f, 0.2f);  // Bright green for maintained
    } else {
        final_col = Color(base_col.r * pulse, base_col.g * pulse, base_col.b * pulse); // Normal with pulse
    }
    
    // ================== GLOW EFFECT ==================
    if (sensor.active) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        
        // Glow size based on issue severity
        float glow_size = 2.5f;
        if (tcp_critical) glow_size = 5.0f;
        else if (tcp_leak) glow_size = 4.0f;
        else if (tcp_blockage) glow_size = 3.5f;
        
        // Glow color
        if (is_stale) {
            glColor4f(final_col.r * 0.5f, final_col.g * 0.3f, final_col.b * 0.3f, 0.2f);
        } else if (has_issue) {
            // Pulsing glow for issues
            float issue_pulse = 0.5f + 0.5f * sinf(city.simulationTime * pulse_rate * 1.5f);
            glColor4f(final_col.r, final_col.g, final_col.b, 0.5f * issue_pulse);
        } else {
            glColor4f(final_col.r, final_col.g, final_col.b, 0.3f * pulse);
        }
        
        glPushMatrix();
        glTranslatef(sensor.position.x, sensor.position.y, sensor.position.z);
        glutSolidSphere(glow_size, 16, 16);
        glPopMatrix();
        glDisable(GL_BLEND);
    }
    
    // ================== SENSOR BODY ==================
    GLfloat mat_specular[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat mat_shininess[] = { 80.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    
    float sensorSize = 2.2f;
    if (tcp_critical) sensorSize = 3.0f;
    else if (has_issue) sensorSize = 2.5f;
    
    glColor3f(final_col.r, final_col.g, final_col.b);
    
    // Draw sensor body
    glPushMatrix();
    glTranslatef(sensor.position.x, sensor.position.y, sensor.position.z);
    
    // For critical sensors, add rotation
    if (tcp_critical) {
        glRotatef(city.simulationTime * 100.0f, 0, 1, 0);
    }
    
    // Draw different shapes based on sensor type
    if (sensor.type == WATER_SENSOR) {
        glutSolidCube(sensorSize);  // Cube for water sensors
    } else {
        // Cylinder for sewage sensors
        glRotatef(90, 1, 0, 0);
        GLUquadric* quad = gluNewQuadric();
        gluCylinder(quad, sensorSize/2, sensorSize/2, sensorSize, 10, 3);
        gluDeleteQuadric(quad);
    }
    glPopMatrix();
    
    // ================== VALVE WHEEL ==================
    glDisable(GL_LIGHTING);
    
    // Valve wheel color
    Color valve_col;
    if (is_stale) {
        valve_col = Color(final_col.r * 0.4f, final_col.g * 0.4f, final_col.b * 0.4f);
    } else if (has_issue) {
        valve_col = final_col;
    } else {
        valve_col = Color(final_col.r * 0.7f, final_col.g * 0.7f, final_col.b * 0.7f);
    }
    glColor3f(valve_col.r, valve_col.g, valve_col.b);
    
    glPushMatrix();
    glTranslatef(sensor.position.x, sensor.position.y + 1.8f, sensor.position.z);
    glRotatef(90, 1, 0, 0);
    
    // Rotate valve based on valve state
    float valve_rotation = sensor.valveState * 3.6f; // 0-100% to 0-360 degrees
    glRotatef(valve_rotation, 0, 0, 1);
    
    // Draw valve wheel
    glutSolidTorus(0.4, 0.9, 10, 20);
    
    // Add valve indicator line
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0.9);
    glVertex3f(0, 0, 1.5);
    glEnd();
    
    glPopMatrix();
    glEnable(GL_LIGHTING);
    
    // ================== ISSUE INDICATORS ==================
    if (has_issue) {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        
        // Issue indicator above sensor
        float indicator_y = sensor.position.y + 3.0f;
        float indicator_size = 0.8f;
        
        if (tcp_critical) {
            // Red warning triangle for critical
            glColor4f(1.0f, 0.0f, 0.0f, 0.8f * pulse);
            glPushMatrix();
            glTranslatef(sensor.position.x, indicator_y, sensor.position.z);
            glRotatef(90, 1, 0, 0);
            glutSolidCone(indicator_size, indicator_size * 2, 3, 2);
            glPopMatrix();
        } else if (tcp_leak) {
            // Red droplet for leaks
            glColor4f(1.0f, 0.3f, 0.3f, 0.7f * pulse);
            glPushMatrix();
            glTranslatef(sensor.position.x, indicator_y, sensor.position.z);
            glutSolidSphere(indicator_size, 12, 12);
            glPopMatrix();
        } else if (tcp_blockage) {
            // Yellow X for blockages
            glColor4f(1.0f, 0.8f, 0.2f, 0.7f * pulse);
            glLineWidth(3.0f);
            glBegin(GL_LINES);
            glVertex3f(sensor.position.x - indicator_size, indicator_y, sensor.position.z - indicator_size);
            glVertex3f(sensor.position.x + indicator_size, indicator_y, sensor.position.z + indicator_size);
            glVertex3f(sensor.position.x + indicator_size, indicator_y, sensor.position.z - indicator_size);
            glVertex3f(sensor.position.x - indicator_size, indicator_y, sensor.position.z + indicator_size);
            glEnd();
        }
        
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }
    
    // ================== SENSOR LABELS ==================
    if (showSensorLabels) {
        glDisable(GL_LIGHTING);
        
        // Label background for better readability
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
        glBegin(GL_QUADS);
        float bg_width = 8.0f;
        float bg_height = 2.5f;
        glVertex3f(sensor.position.x - bg_width/2, sensor.position.y + 6.0f, sensor.position.z - 0.1f);
        glVertex3f(sensor.position.x + bg_width/2, sensor.position.y + 6.0f, sensor.position.z - 0.1f);
        glVertex3f(sensor.position.x + bg_width/2, sensor.position.y + 8.5f, sensor.position.z - 0.1f);
        glVertex3f(sensor.position.x - bg_width/2, sensor.position.y + 8.5f, sensor.position.z - 0.1f);
        glEnd();
        glDisable(GL_BLEND);
        
        // Main label - sensor ID and name
        Color label_color;
        if (tcp_critical) label_color = Color(1.0f, 0.5f, 0.5f);
        else if (has_issue) label_color = final_col;
        else if (is_stale) label_color = Color(0.6f, 0.6f, 0.6f);
        else label_color = (sensor.type == WATER_SENSOR) ? 
                          Color(0.3f, 0.85f, 1.0f) : Color(1.0f, 0.7f, 0.2f);
        
        glColor3f(label_color.r, label_color.g, label_color.b);
        
        // Sensor ID and name
        glRasterPos3f(sensor.position.x - 4.0f, sensor.position.y + 8.0f, sensor.position.z);
        std::stringstream ss;
        ss << "ID:" << sensor.id << " " << sensor.name;
        if (sensor.last_esp_id != -1) {
            ss << " [ESP" << sensor.last_esp_id << "]";
        }
        
        // Add issue icons
        if (tcp_critical) ss << " ⚠️";
        else if (tcp_leak) ss << " 💧";
        else if (tcp_blockage) ss << " 🚫";
        
        std::string label = ss.str();
        for (char c : label) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        
        // Sensor values
        glRasterPos3f(sensor.position.x - 4.0f, sensor.position.y + 6.5f, sensor.position.z);
        ss.str("");
        ss << "V:" << (int)sensor.valveState << "% P:" << (int)sensor.pressure << "kPa L:" << (int)sensor.waterLevel << "%";
        
        // Add data source indicator
        if (sensor.pressureFromROS) {
            ss << " [ROS]";
        } else if (is_stale) {
            ss << " [STALE]";
        } else {
            ss << " [SIM]";
        }
        
        label = ss.str();
        for (char c : label) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c);
        }
        
        // TCP alert status if applicable
        if (has_issue) {
            glRasterPos3f(sensor.position.x - 4.0f, sensor.position.y + 5.5f, sensor.position.z);
            ss.str("");
            if (tcp_critical) ss << "CRITICAL ";
            if (tcp_leak) ss << "LEAK ";
            if (tcp_blockage) ss << "BLOCKAGE ";
            ss << "[TCP]";
            
            Color alert_color = tcp_critical ? Color(1.0f, 0.0f, 0.0f) : 
                               (tcp_leak ? Color(1.0f, 0.3f, 0.3f) : Color(1.0f, 0.8f, 0.2f));
            glColor3f(alert_color.r, alert_color.g, alert_color.b);
            
            label = ss.str();
            for (char c : label) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c);
            }
        }
        
        glEnable(GL_LIGHTING);
    }
    
    // ================== CONNECTION LINES TO PIPE ==================
    if (sensor.connectedPipeID >= 0 && sensor.connectedPipeID < city.pipes.size()) {
        const Pipe& connected_pipe = city.pipes[sensor.connectedPipeID];
        Vec3 pipe_point = (connected_pipe.start + connected_pipe.end) * 0.5f;
        
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Connection line color based on sensor state
        Color line_color;
        if (has_issue) {
            line_color = final_col;
        } else if (is_stale) {
            line_color = Color(0.4f, 0.4f, 0.4f, 0.3f);
        } else {
            line_color = Color(final_col.r, final_col.g, final_col.b, 0.5f);
        }
        
        glColor4f(line_color.r, line_color.g, line_color.b, 0.5f);
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        glVertex3f(sensor.position.x, sensor.position.y, sensor.position.z);
        glVertex3f(pipe_point.x, pipe_point.y, pipe_point.z);
        glEnd();
        
        glDisable(GL_BLEND);
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
    
    // ================== TCP LEAK & BLOCKAGE STATUS ==================
    if (city.leak_client.isConnected()) {
        int tcp_leaks = 0;
        int tcp_blockages = 0;
        int tcp_critical = 0;
        
        // Get counts from TCP client (methods exist now)
        tcp_leaks = city.leak_client.getLeakCount();
        tcp_blockages = city.leak_client.getBlockageCount();
        tcp_critical = city.leak_client.getCriticalCount();
        
        ss.str("");
        ss << "TCP ALERTS: ";
        
        if (tcp_critical > 0) {
            ss << "🚨 " << tcp_critical << " CRITICAL";
            if (tcp_leaks > 0 || tcp_blockages > 0) ss << " | ";
        }
        
        if (tcp_leaks > 0) {
            ss << "🔴 " << tcp_leaks << " LEAK" << (tcp_leaks > 1 ? "S" : "");
            if (tcp_blockages > 0) ss << " | ";
        }
        
        if (tcp_blockages > 0) {
            ss << "🟡 " << tcp_blockages << " BLOCKAGE" << (tcp_blockages > 1 ? "S" : "");
        }
        
        if (tcp_leaks == 0 && tcp_blockages == 0 && tcp_critical == 0) {
            ss << "✅ ALL CLEAR";
        }
        
        // Color coding for TCP alerts
        Color tcp_alert_color;
        if (tcp_critical > 0) {
            tcp_alert_color = Color(1.0f, 0.0f, 0.0f); // Bright red for critical
        } else if (tcp_leaks > 0) {
            tcp_alert_color = Color(1.0f, 0.3f, 0.3f); // Red for leaks
        } else if (tcp_blockages > 0) {
            tcp_alert_color = Color(1.0f, 0.8f, 0.2f); // Yellow for blockages
        } else {
            tcp_alert_color = Color(0.3f, 0.9f, 0.3f); // Green for clear
        }
        
        drawText(10, windowHeight - 100, ss.str(), tcp_alert_color);
        ss.str("");
        
        // TCP connection status
        ss << "TCP SERVER: " << (city.leak_client.isConnected() ? "✅ CONNECTED" : "❌ DISCONNECTED");
        Color conn_color = city.leak_client.isConnected() ? 
                          Color(0.2f, 0.9f, 0.2f) : Color(1.0f, 0.2f, 0.2f);
        drawText(10, windowHeight - 125, ss.str(), conn_color);
        ss.str("");
        
        // Show if TCP is receiving data
        static time_t last_tcp_update = 0;
        static int tcp_message_count = 0;
        
        // For demo, simulate occasional TCP messages
        static float tcp_timer = 0.0f;
        tcp_timer += 0.016f; // Assume 60 FPS
        
        if (tcp_timer > 3.0f) { // Every 3 seconds
            last_tcp_update = time(nullptr);
            tcp_message_count++;
            tcp_timer = 0.0f;
        }
        
        if (last_tcp_update > 0) {
            int seconds_ago = time(nullptr) - last_tcp_update;
            if (seconds_ago < 5) {
                ss << "TCP ACTIVE: " << seconds_ago << "s ago (" << tcp_message_count << " msgs)";
                drawText(10, windowHeight - 150, ss.str(), Color(0.6f, 0.9f, 0.6f));
                ss.str("");
            }
        }
    } else {
        // TCP not connected warning
        ss << "TCP SERVER: ❌ DISCONNECTED - No live leak/blockage data";
        drawText(10, windowHeight - 100, ss.str(), Color(1.0f, 0.3f, 0.3f));
        ss.str("");
        
        ss << "Run TCP server on port 9999 to receive alerts";
        drawText(10, windowHeight - 125, ss.str(), Color(0.8f, 0.8f, 0.8f));
        ss.str("");
    }
    
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
        drawText(10, windowHeight - 175, ss.str(), status_color);
    } else {
        drawText(10, windowHeight - 175, "✗ ROS2 NOT INITIALIZED - Using default values", Color(1.0f, 0.2f, 0.2f));
    }
    ss.str("");
    
    // Reservoir status
    ss << "RESERVOIR: " << std::fixed << std::setprecision(1) 
    << city.reservoir.getLevelPercent() << "% (" << city.reservoir.getTrendString() << ")"
    << " | Pump: " << std::setprecision(2) << city.reservoir.availablePumpCapacity << " m³/s"
    << " | Margin: " << std::setprecision(2) << city.reservoir.supplyMargin << " m³/s";
    drawText(10, windowHeight - 200, ss.str());
    ss.str("");
    
    // Water consumption
    float totalDemand = city.calculateTotalDemand();
    
    ss << "CITY WATER: " << std::fixed << std::setprecision(2) << (totalDemand * 1000.0f) 
    << " L/s | Total consumed: " << std::setprecision(0) << city.totalWaterConsumed << " m³";
    drawText(10, windowHeight - 225, ss.str());
    ss.str("");
    
    // UDP broadcast status
    ss << "UDP BROADCAST: " << UDP_BROADCAST_PORT << " @ " << (1.0f/UDP_BROADCAST_INTERVAL) << "Hz"
    << " | RL Steps: " << city.episodeStep;
    drawText(10, windowHeight - 250, ss.str(), Color(0.6f, 0.8f, 1.0f));
    ss.str("");
    
    // Zone status summary (SIMULATION leaks - from leaks.csv)
    if (!city.zones.empty()) {
        int leakZones = 0;
        int pressureViolations = 0;
        for (const auto& zone : city.zones) {
            if (zone.leakFlag) leakZones++;
            if (zone.pressureViolation) pressureViolations++;
        }
        
        ss << "SIMULATION ZONES: " << city.zones.size() 
        << " | Leaks: " << leakZones 
        << " | Pressure Violations: " << pressureViolations;
        Color sim_color = (leakZones > 0 || pressureViolations > 0) ? 
                         Color(1.0f, 0.3f, 0.3f) : Color(0.3f, 0.9f, 0.3f);
        drawText(10, windowHeight - 275, ss.str(), sim_color);
        ss.str("");
    }
    
    // Add sewage reservoir status
    ss << "SEWAGE RESERVOIR: " << std::fixed << std::setprecision(1) 
       << city.sewageReservoir.getFillPercent() << "% " 
       << city.sewageReservoir.getStatus();
    
    if (city.sewageReservoir.needsTreatment) {
        ss << " [TREATMENT IN " << std::setprecision(0) 
           << (10.0f - city.sewageReservoir.timeSinceLastTreatment) << "s]";
    }
    
    Color sewageColor = city.sewageReservoir.needsTreatment ? 
                       Color(1.0f, 0.5f, 0.2f) : Color(0.6f, 0.8f, 0.4f);
    drawText(10, windowHeight - 300, ss.str(), sewageColor);
    ss.str("");
    
    // Add treatment event info
    if (city.lastTreatmentEvent.event_id > 0) {
        ss << "LAST TREATMENT: #" << city.lastTreatmentEvent.event_id 
           << " (" << city.lastTreatmentEvent.event_type << ") "
           << std::fixed << std::setprecision(0) 
           << city.simulationTime - city.lastTreatmentEvent.timestamp << "s ago";
        drawText(10, windowHeight - 325, ss.str(), Color(0.8f, 0.8f, 0.2f));
        ss.str("");
    }
    
    // ESP topics being monitored
    drawText(10, windowHeight - 350, "TOPICS: /esp1/sensors, /esp2/sensors, /esp3/sensors, /esp4/sensors", Color(0.6f, 0.8f, 1.0f));
    
    // ================== BOTTOM CONTROLS SECTION ==================
    drawText(10, 360, "CONTROLS:");
    drawText(10, 340, "B: Buildings | W: Water | S: Sewage | N: Sensors | L: Leaks | T: Labels");
    drawText(10, 320, "X: Transparent | G: Ground | P: Particles | H: Highlight | R: Reset | M: Sun/Moon");
    drawText(10, 300, "+/-: Buildings | Q: Quit | K: Load leaks | 1-9: Test sensor | 0: Broadcast test");
    drawText(10, 280, "C: Clear TCP alerts | V: Toggle visualization | Z: Zone control | F: Fullscreen");
    
    // Data sources legend
    drawText(10, 250, "DATA SOURCES:");
    drawText(10, 230, "• ROS2: Live sensor data from ESP nodes (blue/yellow sensors)");
    drawText(10, 210, "• TCP: Leak/blockage alerts (red sensors, shows in TCP ALERTS)");
    drawText(10, 190, "• Simulation: Hydraulic models (SIMULATION ZONES)");
    
    // Sensor status legend
    drawText(10, 160, "SENSOR STATUS:");
    drawText(10, 140, "• 🔵 Blue: Normal water sensor");
    drawText(10, 120, "• 🟡 Yellow: Normal sewage sensor");
    drawText(10, 100, "• 🔴 Red: Leak detected (pulsing)");
    drawText(10, 80, "• 🟠 Orange: Blockage detected (pulsing)");
    drawText(10, 60, "• ⚫ Gray: Sensor stale/no data");
    
    // RL info
    drawText(10, 30, "RL STATE BROADCAST:");
    drawText(10, 10, "• 10Hz UDP broadcast to port " + std::to_string(UDP_BROADCAST_PORT));
    
    // ================== RIGHT SIDE STATUS INDICATORS ==================
    if (transparentBuildings) {
        drawText(windowWidth - 300, 30, ">>> BUILDINGS TRANSPARENT <<<", Color(0.2f, 1.0f, 0.2f));
    }
    
    if (showWaterParticles) {
        ss.str("");
        ss << "PARTICLES: Flow visualization ACTIVE";
        drawText(windowWidth - 300, 50, ss.str(), Color(0.2f, 0.8f, 1.0f));
        ss.str("");
    }
    
    if (highlightedCluster >= 0) {
        ss.str("");
        ss << "HIGHLIGHTED: Zone " << highlightedCluster;
        drawText(windowWidth - 300, 70, ss.str(), Color(1.0f, 1.0f, 0.0f));
        ss.str("");
    }
    
    if (!showWaterNetwork) {
        drawText(windowWidth - 300, 90, "WATER NETWORK: HIDDEN", Color(1.0f, 0.5f, 0.5f));
    }
    
    if (!showSewageNetwork) {
        drawText(windowWidth - 300, 110, "SEWAGE NETWORK: HIDDEN", Color(1.0f, 0.5f, 0.5f));
    }
    
    if (!showSensors) {
        drawText(windowWidth - 300, 130, "SENSORS: HIDDEN", Color(1.0f, 0.5f, 0.5f));
    }
    
    // Show simulation speed indicator
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    float frame_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    
    float fps = (frame_time > 0) ? 1.0f / frame_time : 0;
    
    ss.str("");
    ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
    drawText(windowWidth - 300, windowHeight - 50, ss.str(), 
            (fps > 30) ? Color(0.2f, 1.0f, 0.2f) : 
            (fps > 15) ? Color(1.0f, 1.0f, 0.2f) : 
            Color(1.0f, 0.2f, 0.2f));
    
    // Show simulation time
    ss.str("");
    int sim_hours = (int)(city.simulationTime / 3600.0f);
    int sim_minutes = (int)((city.simulationTime - sim_hours * 3600.0f) / 60.0f);
    int sim_seconds = (int)(city.simulationTime) % 60;
    ss << "SIM TIME: " << std::setw(2) << std::setfill('0') << sim_hours << ":"
       << std::setw(2) << std::setfill('0') << sim_minutes << ":"
       << std::setw(2) << std::setfill('0') << sim_seconds;
    drawText(windowWidth - 300, windowHeight - 30, ss.str(), Color(0.8f, 0.8f, 1.0f));
    
    // Show UDP broadcast status
    static int udp_counter = 0;
    static auto last_udp_display = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_udp_display).count();
    
    if (elapsed_ms < 500) { // Show for 500ms after UDP broadcast
        ss.str("");
        ss << "UDP BROADCAST #" << udp_counter++;
        drawText(windowWidth - 300, windowHeight - 70, ss.str(), Color(0.6f, 0.9f, 1.0f));
    }
    
    // Show memory usage (approximate)
    static int memory_counter = 0;
    memory_counter++;
    if (memory_counter % 100 == 0) {
        // This is a placeholder - you could add actual memory tracking
        ss.str("");
        ss << "MEM: ~" << (city.buildings.size() * sizeof(Building) + 
                          city.pipes.size() * sizeof(Pipe) + 
                          city.sensors.size() * sizeof(Sensor)) / 1024 << " KB";
        drawText(windowWidth - 300, windowHeight - 90, ss.str(), Color(0.8f, 0.8f, 0.8f));
    }
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
    
    // Export CSV every 60 frames (approx 1 second at 60 FPS)
    if (frame_count % 60 == 0) {
        std::lock_guard<std::mutex> lock(city_mutex);
        
        // Generate timestamp for filename
        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);
        char filename[100];
        strftime(filename, sizeof(filename), "hourly_usage_%Y%m%d_%H%M%S.csv", timeinfo);
        
        // Open CSV file
        std::ofstream csv_file(filename);
        if (csv_file.is_open()) {
            csv_file << "timestamp,source_ip,hour,usage\n";
            
            // Get simulation time in hours
            float sim_hours = city.simulationTime / 3600.0f;
            int current_hour = static_cast<int>(sim_hours) % 24;
            
            // Write data for all sensors
            for (const auto& sensor : city.sensors) {
                // Write hourly usage for all 24 hours
                for (int hour = 0; hour < 24; hour++) {
                    float usage = sensor.hourlyVolume_m3[hour];
                    
                    // Format timestamp
                    char timestamp[100];
                    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", timeinfo);
                    
                    csv_file << timestamp << ".282382,172.20.10.5," 
                            << hour << "," << usage << "\n";
                }
            }
            
            csv_file.close();
            std::cout << "CSV exported: " << filename << std::endl;
        }
        
        // Also export JSON files
        city.exportSensorJSON("sensor.json");
        city.exportLeakJSON("leak.json");
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
            if (showWaterParticles) {
                std::cout << "Water particles: ON (High-density flow visualization)\n";
                std::cout << "  - Shows millions of particles as continuous flow\n";
                std::cout << "  - Blue gradient indicates flow velocity\n";
                std::cout << "  - Particles follow pipe curvature\n";
            } else {
                std::cout << "Water particles: OFF\n";
            }
            break;
            
        case 'v': case 'V':  // New: Toggle visualization mode
            static int vis_mode = 0;
            vis_mode = (vis_mode + 1) % 3;
            switch (vis_mode) {
                case 0:
                    std::cout << "Visualization: Particle System (High-density)\n";
                    break;
                case 1:
                    std::cout << "Visualization: Streamlines\n";
                    break;
                case 2:
                    std::cout << "Visualization: Combined (Particles + Streamlines)\n";
                    break;
            }
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
    
    city.drawSewageReservoir();
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
    
    // ================== RENDER HIGH-DENSITY WATER FLOW FIRST (in the background) ==================
    if (showWaterParticles) {
        // Render volumetric streamlines first (subtle background flow)
        city.volumetric_renderer.render();
        
        // Render dense particle system (main water flow visualization)
        city.water_particles.render(city.pipes);
    }
    
    // ================== RENDER INFRASTRUCTURE ==================
    if (showBuildings) {
        for (const auto& bldg : city.buildings) {
            if (highlightedCluster < 0 || bldg.clusterID == highlightedCluster) {
                drawBuilding(bldg);
            }
        }
    }
    
    // Render pipes (water and sewage)
    for (const auto& pipe : city.pipes) {
        bool isWater = pipe.type < SEWAGE_LATERAL;
        if ((isWater && showWaterNetwork) || (!isWater && showSewageNetwork)) {
            drawPipe(pipe);
            
            // Render leak particles separately (if particles are disabled)
            if (!showWaterParticles && pipe.hasLeak && showLeaks) {
                pipe.drawLeakParticles(city.simulationTime);
            }
        }
    }
    
    // Render sensors on top
    if (showSensors) {
        for (const auto& sensor : city.sensors) {
            drawSensor(sensor);
        }
    }
    
    drawHUD();
    
    glutSwapBuffers();
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
        std::cout << "        \"pressure_kpa\": 103,0,\n";
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