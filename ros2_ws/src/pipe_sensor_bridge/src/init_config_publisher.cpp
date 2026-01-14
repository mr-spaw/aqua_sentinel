#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <cmath>

using json = nlohmann::json;

class InitConfigPublisher : public rclcpp::Node {
private:
    mosquitto *mosq_;
    std::string json_path_;
    int esp_count_;
    int sensors_per_esp_;
    rclcpp::TimerBase::SharedPtr timer_;
    
public:
    InitConfigPublisher() : Node("init_config_publisher") {
        // Declare and get parameters
        this->declare_parameter<std::string>("json_path", 
            "/home/arka/aqua_sentinel/simulator/init_sensor.json");
        this->declare_parameter<int>("esp_count", 4);
        this->declare_parameter<int>("sensors_per_esp", 0); 
        
        this->get_parameter("json_path", json_path_);
        this->get_parameter("esp_count", esp_count_);
        this->get_parameter("sensors_per_esp", sensors_per_esp_);
        
        // Validate
        if (esp_count_ <= 0) {
            RCLCPP_FATAL(this->get_logger(), "ESP count must be positive");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize MQTT
        mosquitto_lib_init();
        mosq_ = mosquitto_new("ros2_init_publisher", true, nullptr);
        
        if (!mosq_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create MQTT client");
            return;
        }
        
        if (mosquitto_connect(mosq_, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "MQTT connection failed");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "ROS2 Init Config Publisher initialized");
        RCLCPP_INFO(this->get_logger(), "ESP count: %d", esp_count_);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() { 
                this->publishInitConfig();
                timer_.reset();
            }
        );
    }
    
    ~InitConfigPublisher() {
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }
    
    bool publishInitConfig() {
        std::ifstream f(json_path_);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open init_sensor.json at: %s", 
                json_path_.c_str());
            return false;
        }
        
        json sim_json;
        f >> sim_json;
        f.close();
        
        if (!sim_json.contains("sensors")) {
            RCLCPP_ERROR(this->get_logger(), "JSON does not contain 'sensors' array");
            return false;
        }
        
        auto all_sensors = sim_json["sensors"];
        int total_sensors = all_sensors.size();
        
        RCLCPP_INFO(this->get_logger(), "Loaded %d sensors from JSON", total_sensors);
        
        // Auto-calculate sensors per ESP 
        int actual_sensors_per_esp = sensors_per_esp_;
        if (actual_sensors_per_esp <= 0) {
            actual_sensors_per_esp = (int)std::ceil((double)total_sensors / esp_count_);
        }
        
        RCLCPP_INFO(this->get_logger(), "Distributing %d sensors across %d ESPs (~%d per ESP)",
            total_sensors, esp_count_, actual_sensors_per_esp);
        
        // Calculate distribution
        int remaining_sensors = total_sensors;
        int current_index = 0;
        
        for (int esp_id = 1; esp_id <= esp_count_; esp_id++) {
            // Calculate sensors 
            int sensors_this_esp = (esp_id == esp_count_) ? 
                remaining_sensors : 
                std::min(actual_sensors_per_esp, remaining_sensors);
            
            if (sensors_this_esp <= 0) {
                RCLCPP_WARN(this->get_logger(), 
                    "ESP%d: No sensors to assign", esp_id);
                continue;
            }
            
            int end_idx = current_index + sensors_this_esp;
            
            json esp_config;
            esp_config["timestamp"] = sim_json["timestamp"];
            esp_config["dt"] = 1.0;
            esp_config["esp_id"] = esp_id;
            esp_config["sensor_range_start"] = current_index;
            esp_config["sensor_range_end"] = end_idx - 1;
            esp_config["sensor_count"] = sensors_this_esp;
            esp_config["total_sensors_in_json"] = total_sensors;
            
            json sensors = json::array();
            
            for (int i = current_index; i < end_idx; i++) {
                auto &s = all_sensors[i];
                
                json sensor_obj;
                sensor_obj["sensor_id"] = s["sensor_id"];
                sensor_obj["pipe_id"] = s["pipe_id"];
                sensor_obj["type"] = (s["pipe_type"].get<int>() == 0) ? "fresh" : "sewage";
                sensor_obj["pressure_kpa"] = s["pressure_kpa"];
                sensor_obj["level_pct"] = s["level_pct"];
                sensor_obj["valve"] = s.contains("valve") ? s["valve"].get<int>() : 1;
                
                sensors.push_back(sensor_obj);
            }
            
            esp_config["sensors"] = sensors;
            
            std::string topic = "pipes/config/init" + std::to_string(esp_id);
            std::string payload = esp_config.dump();
            
            mosquitto_publish(
                mosq_,
                nullptr,
                topic.c_str(),
                payload.size(),
                payload.c_str(),
                1,
                true
            );
            
            mosquitto_loop(mosq_, 200, 1);
            
            RCLCPP_INFO(this->get_logger(), 
                "ESP%d: Published %d sensors (IDs %d-%d) to topic: %s", 
                esp_id, sensors_this_esp, current_index, end_idx - 1, topic.c_str());
            
            current_index = end_idx;
            remaining_sensors -= sensors_this_esp;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            " All configurations published! Total sensors assigned: %d/%d", 
            current_index, total_sensors);
            
        if (current_index < total_sensors) {
            RCLCPP_WARN(this->get_logger(), 
                " Note: %d sensors were not assigned (increase ESP count)", 
                total_sensors - current_index);
        }
        
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitConfigPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}