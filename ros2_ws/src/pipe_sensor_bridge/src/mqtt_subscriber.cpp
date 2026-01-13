#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sys/stat.h>  
#include <algorithm>  
#include <unistd.h>    

using json = nlohmann::json;
using std::placeholders::_1;

std::map<int, std::map<int, json>> esp_chunk_buffers;
std::map<int, int> esp_expected_chunks;
rclcpp::Node* g_node = nullptr;

// Log file management
const size_t MAX_LOG_FILE_SIZE = 50 * 1024 * 1024; 
const int MAX_LOG_ENTRIES_PER_FILE = 1000;  
std::map<int, int> esp_entry_count;

// Store publishers as member pointers
std::map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> esp_publishers;

// File streams for logging
std::map<int, std::ofstream> esp_log_files;
std::string log_directory = "sensor_logs";
std::string package_path = "/home/arka/aqua_sentinel/ros2_ws/src/pipe_sensor_bridge";

// Initialize first_entry for each ESP
std::map<int, bool> esp_first_entry;

// Dynamic configuration 
int sensors_per_esp_ = 75;  
int max_esp_count_ = 4;   
std::map<int, std::pair<int, int>> esp_sensor_ranges;  

// ========== HELPER FUNCTION ==========
// Get ESP ID from sensor ID using dynamic configuration
int getEspIdFromSensorId(int sensor_id) {
    if (sensor_id < 0) {
        return -1;
    }
    
    // Method 1: Check if we have explicit ranges defined
    if (!esp_sensor_ranges.empty()) {
        for (const auto& [esp_id, range] : esp_sensor_ranges) {
            if (sensor_id >= range.first && sensor_id <= range.second) {
                return esp_id;
            }
        }
    }
    
    // Method 2: Calculate based on sensors_per_esp (fallback)
    int esp_id = (sensor_id / sensors_per_esp_) + 1;
    
    // Safety check - assign to last ESP if exceeds max
    if (esp_id > max_esp_count_) {
        esp_id = max_esp_count_;
        if (g_node) {
            RCLCPP_WARN(g_node->get_logger(), 
                "Sensor ID %d exceeds max ESP count, assigning to ESP%d", 
                sensor_id, max_esp_count_);
        }
    }
    
    return esp_id;
}

// Load configuration from parameters
void load_configuration_from_params(rclcpp::Node* node) {
    // Get parameters
    node->get_parameter("sensors_per_esp", sensors_per_esp_);
    node->get_parameter("max_esp_count", max_esp_count_);
    
    RCLCPP_INFO(node->get_logger(), 
        "Configuration loaded: %d ESPs, %d sensors per ESP", 
        max_esp_count_, sensors_per_esp_);
}

// Load configuration from received MQTT config message
void load_configuration_from_mqtt(const json& config_data) {
    if (config_data.contains("sensor_range_start") && 
        config_data.contains("sensor_range_end") &&
        config_data.contains("esp_id")) {
        
        int esp_id = config_data["esp_id"];
        int start = config_data["sensor_range_start"];
        int end = config_data["sensor_range_end"];
        
        esp_sensor_ranges[esp_id] = {start, end};
        
        if (g_node) {
            RCLCPP_INFO(g_node->get_logger(), 
                "ESP%d sensor range: %d-%d", esp_id, start, end);
        }
    }
    
    // Update max_esp_count if provided
    if (config_data.contains("total_esps")) {
        max_esp_count_ = config_data["total_esps"];
    }
    
    // Update sensors_per_esp if provided
    if (config_data.contains("sensors_per_esp")) {
        sensors_per_esp_ = config_data["sensors_per_esp"];
    }
}

// Get current working directory 
std::string get_current_dir() {
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        return std::string(cwd);
    }
    return "";
}

// Create sensor_logs directory
bool create_log_directory() {
    // Use the fixed path
    std::string full_path = package_path + "/" + log_directory;
    
    RCLCPP_INFO(g_node->get_logger(), "Creating log directory: %s", full_path.c_str());

    if (mkdir(full_path.c_str(), 0777) != 0) {
        if (errno != EEXIST) {
            if (g_node) {
                RCLCPP_ERROR(g_node->get_logger(), 
                    "Failed to create log directory '%s': %s", 
                    full_path.c_str(), strerror(errno));
            }
            return false;
        }
    }
    
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), " Log directory: %s", full_path.c_str());
    }
    return true;
}

bool file_exists_and_has_content(const std::string& filename) {
    std::ifstream file(filename, std::ios::ate); 
    if (!file.is_open()) {
        return false;
    }
    
    std::streampos file_size = file.tellg();
    file.close();
    return file_size > 0;
}

// Initialize ESP log files
void initialize_esp_log_files() {
    RCLCPP_INFO(g_node->get_logger(), "Initializing log files in: %s", package_path.c_str());
    
    if (!create_log_directory()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to create log directory");
        return;
    }
    
    std::string full_log_dir = package_path + "/" + log_directory;
    
    // Initialize log files for each ESP based on max_esp_count_
    for (int i = 1; i <= max_esp_count_; i++) {
        std::string filename = full_log_dir + "/esp" + std::to_string(i) + "_sensors.json";
        
        // Close if already open
        if (esp_log_files[i].is_open()) {
            esp_log_files[i].close();
        }
        
        // Check file size
        bool should_clear = false;
        std::ifstream check_file(filename, std::ios::ate);
        if (check_file.is_open()) {
            size_t file_size = check_file.tellg();
            check_file.close();
            
            if (file_size > MAX_LOG_FILE_SIZE) {
                should_clear = true;
                if (g_node) {
                    RCLCPP_WARN(g_node->get_logger(), 
                        "ESP%d log file exceeds %zuMB, clearing...", 
                        i, MAX_LOG_FILE_SIZE / (1024*1024));
                }
            }
        }

        if (should_clear) {
            esp_log_files[i].open(filename, std::ios::out | std::ios::trunc);
            esp_log_files[i] << "[\n";
            esp_first_entry[i] = true;
            esp_entry_count[i] = 0;
        } else {
            bool file_has_content = file_exists_and_has_content(filename);
            esp_log_files[i].open(filename, std::ios::out | std::ios::app);
            
            if (!file_has_content) {
                esp_log_files[i] << "[\n";
                esp_first_entry[i] = true;
                esp_entry_count[i] = 0;
            } else {
                esp_first_entry[i] = false;
                esp_entry_count[i] = 0; 
            }
        }
        
        if (!esp_log_files[i].is_open()) {
            if (g_node) {
                RCLCPP_ERROR(g_node->get_logger(), 
                    "Failed to open log file: %s", filename.c_str());
            }
            continue;
        }
        
        esp_log_files[i].flush();
        
        if (g_node) {
            RCLCPP_INFO(g_node->get_logger(), 
                " Log file ready: esp%d_sensors.json", i);
        }
    }
}

// Log sensor data to ESP's file
void log_esp_sensor_data(int esp_id, const json& sensor_data) {
    if (!esp_log_files[esp_id].is_open()) {
        if (g_node) {
            RCLCPP_WARN(g_node->get_logger(), "Log file for ESP%d not open!", esp_id);
        }
        return;
    }

    esp_entry_count[esp_id]++;
    if (esp_entry_count[esp_id] > MAX_LOG_ENTRIES_PER_FILE) {
        if (g_node) {
            RCLCPP_INFO(g_node->get_logger(), 
                "ESP%d: Rotating log file (%d entries)", 
                esp_id, MAX_LOG_ENTRIES_PER_FILE);
        }
  
        esp_log_files[esp_id] << "\n]";
        esp_log_files[esp_id].close();
        
        std::string full_log_dir = package_path + "/" + log_directory;
        std::string filename = full_log_dir + "/esp" + std::to_string(esp_id) + "_sensors.json";
        
        esp_log_files[esp_id].open(filename, std::ios::out | std::ios::trunc);
        esp_log_files[esp_id] << "[\n";
        esp_first_entry[esp_id] = true;
        esp_entry_count[esp_id] = 0;
    }
    
    try {
        if (!esp_first_entry[esp_id]) {
            esp_log_files[esp_id] << ",\n";
        } else {
            esp_first_entry[esp_id] = false;
        }
        
        json log_entry;
        log_entry["timestamp"] = sensor_data["timestamp"];
        log_entry["esp_id"] = esp_id;
        log_entry["sensors"] = sensor_data["sensors"];
        
        esp_log_files[esp_id] << log_entry.dump(2);
        esp_log_files[esp_id].flush();
        
    } catch (const std::exception &e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), 
                "Failed to log ESP%d data: %s", esp_id, e.what());
        }
    }
}

// Log individual sensor data
void log_individual_sensor(int esp_id, const json& sensor_data) {
    if (!esp_log_files[esp_id].is_open()) {
        return;
    }
    
    try {
        if (!esp_first_entry[esp_id]) {
            esp_log_files[esp_id] << ",\n";
        } else {
            esp_first_entry[esp_id] = false;
        }
        
        json log_entry;
        log_entry["timestamp"] = static_cast<int>(time(nullptr));
        log_entry["esp_id"] = esp_id;
        log_entry["sensors"] = json::array({sensor_data});
        
        esp_log_files[esp_id] << log_entry.dump(2);
        esp_log_files[esp_id].flush();
        
    } catch (const std::exception &e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), 
                "Failed to log individual sensor data: %s", e.what());
        }
    }
}

void close_all_log_files() {
    for (int i = 1; i <= max_esp_count_; i++) {
        if (esp_log_files[i].is_open()) {
            esp_log_files[i] << "\n]";
            esp_log_files[i].close();
            
            if (g_node) {
                RCLCPP_INFO(g_node->get_logger(), "Closed log file for ESP%d", i);
            }
        }
    }
}

// Handle individual sensor messages
void handle_individual_sensor_message(int esp_id, const json& sensor_data) {
    if (g_node && esp_publishers.count(esp_id) > 0) {
        json sensor_frame;
        sensor_frame["timestamp"] = static_cast<int>(time(nullptr));
        sensor_frame["esp_id"] = esp_id;
        sensor_frame["sensors"] = json::array({sensor_data});
        
        log_individual_sensor(esp_id, sensor_data);
        
        std_msgs::msg::String ros_msg;
        ros_msg.data = sensor_frame.dump();
        esp_publishers[esp_id]->publish(ros_msg);
        
        RCLCPP_DEBUG(g_node->get_logger(), 
            "📡 ESP%d: Individual sensor %d → ROS", 
            esp_id, sensor_data.value("sensor_id", -1));
    }
}

void handle_system_summary(int esp_id, const json& summary_data) {
    if (g_node) {
        std_msgs::msg::String ros_msg;
        
        json summary_frame = summary_data;
        summary_frame["esp_id"] = esp_id;
        summary_frame["timestamp"] = static_cast<int>(time(nullptr));
        
        ros_msg.data = summary_frame.dump();
        
        static std::map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> summary_publishers;
        
        if (summary_publishers.find(esp_id) == summary_publishers.end()) {
            summary_publishers[esp_id] = g_node->create_publisher<std_msgs::msg::String>(
                "/esp" + std::to_string(esp_id) + "/summary", 10);
        }
        
        summary_publishers[esp_id]->publish(ros_msg);
        
        RCLCPP_DEBUG(g_node->get_logger(), 
            "ESP%d: System summary → ROS", esp_id);
    }
}

// Handle configuration messages from InitConfigPublisher
void handle_configuration_message(int esp_id, const json& config_data) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), 
            "Received configuration for ESP%d", esp_id);
        
        // Load configuration from the message
        load_configuration_from_mqtt(config_data);
        
        // Log the configuration
        RCLCPP_INFO(g_node->get_logger(), 
            "ESP%d: Sensors %d-%d (%d sensors)", 
            esp_id, 
            config_data.value("sensor_range_start", -1),
            config_data.value("sensor_range_end", -1),
            config_data.value("sensor_count", 0));
    }
}

void on_message(struct mosquitto *, void *, const struct mosquitto_message *msg)
{
    std::string topic = msg->topic;
    std::string payload((char*)msg->payload, msg->payloadlen);
    
    // Determine which ESP this is from
    int esp_id = -1;
    
    // Check for different topic patterns
    if (topic.find("pipes/sensors1/") != std::string::npos) esp_id = 1;
    else if (topic.find("pipes/sensors2/") != std::string::npos) esp_id = 2;
    else if (topic.find("pipes/sensors3/") != std::string::npos) esp_id = 3;
    else if (topic.find("pipes/sensors4/") != std::string::npos) esp_id = 4;
    else if (topic.find("pipes/sensors") != std::string::npos) {
        // Check if it's a higher numbered ESP (5+)
        size_t pos = topic.find("pipes/sensors");
        if (pos != std::string::npos) {
            std::string esp_num_str = topic.substr(pos + 13);
            esp_num_str = esp_num_str.substr(0, esp_num_str.find('/'));
            try {
                esp_id = std::stoi(esp_num_str);
            } catch (...) {
                return;
            }
        }
    } else return;
    
    if (g_node) {
        RCLCPP_DEBUG(g_node->get_logger(), 
            "Received MQTT from ESP%d, topic: %s", esp_id, topic.c_str());
    }
    
    try {
        auto j = json::parse(payload);
        
        // Check what type of message this is
        if (topic.find("/all") != std::string::npos) {
            // Batched sensor data
            if (!j.contains("sensors")) {
                if (g_node) {
                    RCLCPP_WARN(g_node->get_logger(), 
                        "ESP%d: Message missing 'sensors' field", esp_id);
                }
                return;
            }
            
            int chunk = j.value("chunk", -1);
            int total = j.value("total_chunks", -1);
            
            if (chunk < 0 || total <= 0) {
                if (g_node) {
                    RCLCPP_WARN(g_node->get_logger(), 
                        "ESP%d: Invalid chunk info (chunk=%d, total=%d)", esp_id, chunk, total);
                }
                return;
            }
            
            esp_expected_chunks[esp_id] = total;
            esp_chunk_buffers[esp_id][chunk] = j["sensors"];
            
            if (g_node) {
                RCLCPP_INFO(g_node->get_logger(), 
                    "ESP%d: Chunk %d/%d (%lu sensors)", 
                    esp_id, chunk + 1, total, j["sensors"].size());
            }
            
            if ((int)esp_chunk_buffers[esp_id].size() < esp_expected_chunks[esp_id]) {
                return;
            }
            
            // Full frame received
            if (g_node && esp_publishers.count(esp_id) > 0) {
                // Combine all chunks
                json full_frame;
                full_frame["timestamp"] = j["timestamp"];
                full_frame["esp_id"] = esp_id;
                full_frame["sensors"] = json::array();
                
                for (int i = 0; i < esp_expected_chunks[esp_id]; i++) {
                    if (esp_chunk_buffers[esp_id].count(i) == 0) {
                        RCLCPP_ERROR(g_node->get_logger(), 
                            "ESP%d: Missing chunk %d!", esp_id, i);
                        esp_chunk_buffers[esp_id].clear();
                        esp_expected_chunks[esp_id] = -1;
                        return;
                    }
                    for (const auto &s : esp_chunk_buffers[esp_id][i]) {
                        // Extract essential fields
                        json essential_sensor;
                        essential_sensor["sensor_id"] = s.value("sensor_id", -1);
                        essential_sensor["pipe_id"] = s.value("pipe_id", -1);
                        essential_sensor["pipe_type"] = s.value("type", "fresh") == "fresh" ? 0 : 1;
                        essential_sensor["pressure_bar"] = s.value("pressure_bar", 0.0);
                        essential_sensor["level_pct"] = s.value("level_pct", 0.0);
                        essential_sensor["valve"] = s.value("valve", 1);
                        
                        full_frame["sensors"].push_back(essential_sensor);
                    }
                }
                
                // LOG THE COMPLETE SENSOR DATA
                log_esp_sensor_data(esp_id, full_frame);
                
                // Publish to ROS topic
                std_msgs::msg::String ros_msg;
                ros_msg.data = full_frame.dump();
                esp_publishers[esp_id]->publish(ros_msg);
                
                RCLCPP_INFO(g_node->get_logger(), 
                    "ESP%d: %lu sensors → ROS + Log file", 
                    esp_id, full_frame["sensors"].size());
            }
            
            esp_chunk_buffers[esp_id].clear();
            esp_expected_chunks[esp_id] = -1;
            
        } else if (topic.find("system/summary") != std::string::npos) {
            // System summary message
            handle_system_summary(esp_id, j);
            
        } else if (topic.find("pipes/config/init") != std::string::npos) {
            // Configuration message from InitConfigPublisher
            handle_configuration_message(esp_id, j);
            
        } else if (topic.find("pipes/sensors") != std::string::npos) {
            // Individual sensor message 
            // Check if it's a numeric sensor ID 
            std::string sensor_part = topic.substr(topic.find_last_of('/') + 1);

            try {
                std::stoi(sensor_part);
                handle_individual_sensor_message(esp_id, j);
            } catch (...) {
                RCLCPP_DEBUG(g_node->get_logger(), 
                    "Non-sensor topic: %s", topic.c_str());
            }
        }
        
    } catch (const std::exception &e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), 
                "JSON error on ESP%d: %s", esp_id, e.what());
        }
    }
}

class ROSMQTTBridge : public rclcpp::Node {
private:
    mosquitto *mosq_;
    int sensors_per_esp_;
    int max_esp_count_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr valve_control_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr maintenance_request_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr maintenance_command_pub_;
    rclcpp::TimerBase::SharedPtr mqtt_timer_;
    
public:
    ROSMQTTBridge() : Node("mqtt_subscriber_bridge"), 
                      sensors_per_esp_(75), 
                      max_esp_count_(4) {
        g_node = this;
        
        // Declare parameters
        this->declare_parameter<int>("sensors_per_esp", 75);
        this->declare_parameter<int>("max_esp_count", 4);
        this->declare_parameter<std::string>("package_path", 
            "/home/arka/aqua_sentinel/ros2_ws/src/pipe_sensor_bridge");
        
        // Get parameters
        this->get_parameter("sensors_per_esp", sensors_per_esp_);
        this->get_parameter("max_esp_count", max_esp_count_);
        this->get_parameter("package_path", package_path);
        
        // Update global variables
        ::sensors_per_esp_ = sensors_per_esp_;
        ::max_esp_count_ = max_esp_count_;
        
        RCLCPP_INFO(this->get_logger(), "ROS2-MQTT Bridge starting...");
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  - Max ESP count: %d", max_esp_count_);
        RCLCPP_INFO(this->get_logger(), "  - Sensors per ESP: %d", sensors_per_esp_);
        RCLCPP_INFO(this->get_logger(), "  - Package path: %s", package_path.c_str());
        
        // Initialize the JSON files in sensor_logs folder
        initialize_esp_log_files();
        
        // Create publishers for each ESP based on max_esp_count
        for (int i = 1; i <= max_esp_count_; i++) {
            esp_publishers[i] = this->create_publisher<std_msgs::msg::String>(
                "/esp" + std::to_string(i) + "/sensors", 10);
            RCLCPP_INFO(this->get_logger(), "Created publisher for /esp%d/sensors", i);
        }
        
        mosquitto_lib_init();
        
        mosq_ = mosquitto_new("ros2_mqtt_bridge", true, nullptr);
        if (!mosq_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create MQTT client");
            return;
        }
        
        mosquitto_message_callback_set(mosq_, on_message);
        
        if (mosquitto_connect(mosq_, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "MQTT connection failed");
            return;
        }
        
        // ========== SUBSCRIBE TO ALL NECESSARY TOPICS ==========
        
        // Subscribe to configuration topics
        for (int i = 1; i <= max_esp_count_; i++) {
            std::string config_topic = "pipes/config/init" + std::to_string(i);
            mosquitto_subscribe(mosq_, nullptr, config_topic.c_str(), 0);
        }
        
        // Subscribe to sensor data topics (dynamic based on max_esp_count)
        for (int i = 1; i <= max_esp_count_; i++) {
            // Batched sensor data
            std::string batch_topic = "pipes/sensors" + std::to_string(i) + "/all/#";
            mosquitto_subscribe(mosq_, nullptr, batch_topic.c_str(), 0);
            
            // Individual sensor data
            std::string individual_topic = "pipes/sensors" + std::to_string(i) + "/+";
            mosquitto_subscribe(mosq_, nullptr, individual_topic.c_str(), 0);
            
            // System summary data
            std::string summary_topic = "pipes/system/summary" + std::to_string(i);
            mosquitto_subscribe(mosq_, nullptr, summary_topic.c_str(), 0);
        }

        RCLCPP_INFO(this->get_logger(), " Subscribed to topics for %d ESPs", max_esp_count_);
        
        // ROS2 subscribers
        valve_control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/valve_control", 10,
            std::bind(&ROSMQTTBridge::valveControlCallback, this, _1));
        
        maintenance_request_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/maintenance/request", 10,
            std::bind(&ROSMQTTBridge::maintenanceRequestCallback, this, _1));
        
        maintenance_command_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/maintenance/command", 10);
        
        // Timer for MQTT processing
        mqtt_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ROSMQTTBridge::mqttLoop, this));
        
        RCLCPP_INFO(this->get_logger(), " ROS2-MQTT Bridge initialized");
        RCLCPP_INFO(this->get_logger(), " Logging to: %s/sensor_logs/", package_path.c_str());
        RCLCPP_INFO(this->get_logger(), " Dynamic sensor mapping enabled");
    }
    
    ~ROSMQTTBridge() {
        // Close all JSON files properly
        close_all_log_files();
        
        g_node = nullptr;
        esp_publishers.clear();
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }
    
    void valveControlCallback(const std_msgs::msg::String::SharedPtr msg) {  
        try {
            std::string data = msg->data;
            size_t pos = data.find(':');
            
            if (pos == std::string::npos) {
                RCLCPP_WARN(this->get_logger(), 
                    "Invalid format. Expected: sensor_id:valve_state");
                return;
            }
            
            int sensor_id = std::stoi(data.substr(0, pos));
            int valve_state = std::stoi(data.substr(pos + 1));
            
            // Use the helper function for consistent ESP mapping
            int esp_id = getEspIdFromSensorId(sensor_id);
            
            if (esp_id < 1 || esp_id > max_esp_count_) {
                RCLCPP_WARN(this->get_logger(), 
                    "Sensor ID %d invalid (no ESP assigned)", sensor_id);
                return;
            }
            
            json valve_cmd;
            valve_cmd["sensor_id"] = sensor_id;
            valve_cmd["valve"] = valve_state;
            valve_cmd["timestamp"] = this->now().seconds();
            
            // Publish to ESP-specific valve control topic
            std::string topic = "pipes/control/valve" + std::to_string(esp_id);
            std::string payload = valve_cmd.dump();
            
            mosquitto_publish(mosq_, nullptr, topic.c_str(), 
                payload.size(), payload.c_str(), 1, false);
            
            RCLCPP_INFO(this->get_logger(), 
                "🔧 Valve control → ESP%d: Sensor %d: %s", 
                esp_id, sensor_id, valve_state ? "OPEN" : "CLOSED");
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }
    
    void maintenanceRequestCallback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            auto j = json::parse(msg->data);
            
            int esp_id = -1;
            
            // Check if esp_id is provided
            if (j.contains("esp_id")) {
                esp_id = j["esp_id"];
            }
            // If only sensor_id is provided, auto-detect ESP
            else if (j.contains("sensor_id")) {
                int sensor_id = j["sensor_id"];
                esp_id = getEspIdFromSensorId(sensor_id);
                
                if (esp_id < 1 || esp_id > max_esp_count_) {
                    RCLCPP_WARN(this->get_logger(), 
                        "Sensor ID %d invalid (no ESP assigned)", sensor_id);
                    return;
                }
                
                // Add esp_id to the request
                j["esp_id"] = esp_id;
                j["action"] = "reset_sensor_to_initial";
                
                RCLCPP_INFO(this->get_logger(), 
                    "Auto-detected: Sensor %d is on ESP%d", sensor_id, esp_id);
            }
            else {
                RCLCPP_WARN(this->get_logger(), 
                    "Must have either 'esp_id' or 'sensor_id'");
                return;
            }
            
            // Validate ESP ID
            if (esp_id < 1 || esp_id > max_esp_count_) {
                RCLCPP_WARN(this->get_logger(), "Invalid ESP ID: %d", esp_id);
                return;
            }
            
            // Check action
            if (!j.contains("action")) {
                RCLCPP_WARN(this->get_logger(), "Must have 'action' field");
                return;
            }
            
            std::string action = j["action"];
            
            json maint_cmd;
            maint_cmd["timestamp"] = this->now().seconds();
            
            if (action == "reset_to_initial") {
                if (!j.contains("sensor_ids")) {
                    RCLCPP_WARN(this->get_logger(), "Requires 'sensor_ids' array");
                    return;
                }
                maint_cmd["reset_to_initial"] = j["sensor_ids"];
            } else if (action == "reset_sensor_to_initial") {
                if (!j.contains("sensor_id")) {
                    RCLCPP_WARN(this->get_logger(), "Requires 'sensor_id'");
                    return;
                }
                maint_cmd["reset_sensor_to_initial"] = j["sensor_id"];
            } else if (action == "reset_all_to_initial") {
                maint_cmd["reset_all_to_initial"] = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown action: %s", action.c_str());
                return;
            }
            
            std::string topic = "pipes/maintenance/esp" + std::to_string(esp_id);
            std::string payload = maint_cmd.dump();
            
            mosquitto_publish(mosq_, nullptr, topic.c_str(), 
                payload.size(), payload.c_str(), 1, false);
            
            RCLCPP_INFO(this->get_logger(), "Maintenance sent to ESP%d", esp_id);
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }
    
    void mqttLoop() {
        mosquitto_loop(mosq_, 10, 1);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROSMQTTBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}