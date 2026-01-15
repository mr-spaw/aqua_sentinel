#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <csignal>

namespace fs = std::filesystem;

// ============================================================================
// CONFIGURATION
// ============================================================================
const int UDP_LISTEN_PORT = 8888;
const int BUFFER_SIZE = 65536;
std::atomic<bool> running(true);
std::mutex data_mutex;

// Output directory
const std::string OUTPUT_DIR = "water_system_data";
const std::string CSV_DIR = OUTPUT_DIR + "/csv";
const std::string LOGS_DIR = OUTPUT_DIR + "/logs";

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct RLState {
    // Timestamp
    std::chrono::system_clock::time_point received_time;
    uint64_t timestamp_ms;
    
    // Reservoir
    float reservoir_level_pct;
    std::string reservoir_trend;
    float pump_capacity_available;
    float supply_margin;
    
    // Sewage
    float sewage_level_pct;
    std::string sewage_status;
    bool sewage_needs_treatment;
    float sewage_time_since_treatment;
    
    // Event info
    int last_treatment_event_id;
    float time_since_last_treatment;
    float time_since_last_recharge;
    
    // Zone data
    struct ZoneData {
        int zone_id;
        float avg_pressure;
        float min_pressure;
        float flow;
        float pressure_variance;
        float flow_to_pressure_ratio;
        float historical_flow;
        bool leak_flag;
        float leak_severity;
        bool overflow_flag;
        bool pressure_violation;
    };
    std::vector<ZoneData> zones;
    
    // Time
    float time_of_day;
    std::string day_type;
    int recent_action_count;
    float sim_time_sin;
    float sim_time_cos;
    float episode_progress;
    
    // Valves
    std::vector<float> valve_states;
    std::vector<float> last_action_times;
    
    // Volumes
    std::vector<float> cumulative_volumes;
    float hourly_usage[24];
    
    // Metrics
    float non_revenue_water;
    float supply_efficiency;
    float peak_flow_rate;
    float avg_flow_rate;
    float flow_variance;
    float pressure_compliance;
    float service_continuity;
    float response_time;
    
    // Counts
    int zone_count;
    int sensor_count;
    
    // Source
    std::string source_ip;
    
    RLState() {
        memset(hourly_usage, 0, sizeof(hourly_usage));
        timestamp_ms = 0;
        reservoir_level_pct = 0.0f;
        pump_capacity_available = 0.0f;
        supply_margin = 0.0f;
        sewage_level_pct = 0.0f;
        sewage_needs_treatment = false;
        sewage_time_since_treatment = 0.0f;
        last_treatment_event_id = 0;
        time_since_last_treatment = 0.0f;
        time_since_last_recharge = 0.0f;
        time_of_day = 0.0f;
        recent_action_count = 0;
        sim_time_sin = 0.0f;
        sim_time_cos = 0.0f;
        episode_progress = 0.0f;
        non_revenue_water = 0.0f;
        supply_efficiency = 0.0f;
        peak_flow_rate = 0.0f;
        avg_flow_rate = 0.0f;
        flow_variance = 0.0f;
        pressure_compliance = 0.0f;
        service_continuity = 0.0f;
        response_time = 0.0f;
        zone_count = 0;
        sensor_count = 0;
    }
};

struct TreatmentEvent {
    uint64_t timestamp_ms;
    int event_id;
    std::string event_type;
    float sim_timestamp;
    float fresh_water_before;
    float fresh_water_after;
    float sewage_before;
    float sewage_after;
    float duration_seconds;
    std::string status;
    
    TreatmentEvent() : timestamp_ms(0), event_id(0), sim_timestamp(0),
                      fresh_water_before(0), fresh_water_after(0),
                      sewage_before(0), sewage_after(0),
                      duration_seconds(0) {}
};

struct SystemStatus {
    uint64_t timestamp_ms;
    float sim_timestamp;
    float fresh_water_level;
    std::string fresh_water_trend;
    bool fresh_water_needs_recharge;
    float fresh_water_time_since_recharge;
    float sewage_level;
    std::string sewage_status;
    bool sewage_needs_treatment;
    float sewage_time_since_treatment;
    float total_demand_m3s;
    float total_sewage_inflow_m3s;
    
    SystemStatus() : timestamp_ms(0), sim_timestamp(0),
                    fresh_water_level(0), fresh_water_needs_recharge(false),
                    fresh_water_time_since_recharge(0), sewage_level(0),
                    sewage_needs_treatment(false), sewage_time_since_treatment(0),
                    total_demand_m3s(0), total_sewage_inflow_m3s(0) {}
};

// ============================================================================
// CSV LOGGER CLASS
// ============================================================================
class CSVLogger {
private:
    // File streams
    std::ofstream rl_state_csv;
    std::ofstream reservoir_csv;
    std::ofstream sewage_csv;
    std::ofstream zones_csv;
    std::ofstream valves_csv;
    std::ofstream sensors_csv;
    std::ofstream events_csv;
    std::ofstream status_csv;
    std::ofstream hourly_usage_csv;
    std::ofstream metrics_csv;
    
    // Statistics
    std::ofstream summary_txt;
    
    // Data buffers
    std::vector<RLState> rl_state_buffer;
    std::vector<TreatmentEvent> event_buffer;
    std::vector<SystemStatus> status_buffer;
    
    // Flush interval (seconds)
    int flush_interval;
    time_t last_flush_time;
    
public:
    CSVLogger() {
        // Create directories
        fs::create_directories(CSV_DIR);
        fs::create_directories(LOGS_DIR);
        
        // Generate timestamp for filenames
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* tm = std::localtime(&now_time);
        char timestamp[100];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm);
        std::string prefix = std::string(timestamp);
        
        // Open all CSV files
        rl_state_csv.open(CSV_DIR + "/" + prefix + "_rl_state.csv");
        reservoir_csv.open(CSV_DIR + "/" + prefix + "_reservoir.csv");
        sewage_csv.open(CSV_DIR + "/" + prefix + "_sewage.csv");
        zones_csv.open(CSV_DIR + "/" + prefix + "_zones.csv");
        valves_csv.open(CSV_DIR + "/" + prefix + "_valves.csv");
        sensors_csv.open(CSV_DIR + "/" + prefix + "_sensors.csv");
        events_csv.open(CSV_DIR + "/" + prefix + "_events.csv");
        status_csv.open(CSV_DIR + "/" + prefix + "_status.csv");
        hourly_usage_csv.open(CSV_DIR + "/" + prefix + "_hourly_usage.csv");
        metrics_csv.open(CSV_DIR + "/" + prefix + "_metrics.csv");
        
        // Open summary file
        summary_txt.open(LOGS_DIR + "/" + prefix + "_summary.txt");
        
        // Write CSV headers
        writeHeaders();
        
        flush_interval = 10; // Flush every 10 seconds
        last_flush_time = time(nullptr);
        
        std::cout << "CSV Logger initialized. Files in: " << OUTPUT_DIR << std::endl;
        
        // Write summary header
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm);
        summary_txt << "Water System Data Collection" << std::endl;
        summary_txt << "Started: " << time_str << std::endl;
        summary_txt << "Output Directory: " << OUTPUT_DIR << std::endl;
        summary_txt << "CSV Files: " << CSV_DIR << std::endl;
        summary_txt << "==========================================" << std::endl;
    }
    
    ~CSVLogger() {
        // Flush remaining data
        flushAllData();
        
        // Close all files
        rl_state_csv.close();
        reservoir_csv.close();
        sewage_csv.close();
        zones_csv.close();
        valves_csv.close();
        sensors_csv.close();
        events_csv.close();
        status_csv.close();
        hourly_usage_csv.close();
        metrics_csv.close();
        
        // Write final summary
        auto end_time = std::chrono::system_clock::now();
        std::time_t end_time_t = std::chrono::system_clock::to_time_t(end_time);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&end_time_t));
        
        summary_txt << "\nCollection ended: " << time_str << std::endl;
        summary_txt << "Total RL States: " << rl_state_buffer.size() << std::endl;
        summary_txt << "Total Events: " << event_buffer.size() << std::endl;
        summary_txt << "Total Status Updates: " << status_buffer.size() << std::endl;
        summary_txt.close();
        
        std::cout << "CSV Logger shutdown complete." << std::endl;
    }
    
private:
    void writeHeaders() {
        // RL State CSV (summary of everything)
        rl_state_csv << "timestamp_ms,source_ip,sim_time_of_day,day_type,"
                    << "reservoir_level_pct,reservoir_trend,pump_capacity_available,supply_margin,"
                    << "sewage_level_pct,sewage_status,sewage_needs_treatment,sewage_time_since_treatment,"
                    << "zone_count,sensor_count,non_revenue_water,supply_efficiency,"
                    << "peak_flow_rate,avg_flow_rate,flow_variance,pressure_compliance,"
                    << "service_continuity,response_time,recent_action_count,episode_progress\n";
        
        // Reservoir CSV (detailed reservoir data)
        reservoir_csv << "timestamp_ms,level_pct,trend,pump_capacity_available,supply_margin,"
                     << "needs_recharge,time_since_recharge\n";
        
        // Sewage CSV (detailed sewage data)
        sewage_csv << "timestamp_ms,level_pct,status,needs_treatment,time_since_treatment,"
                  << "last_event_id,time_since_last_event\n";
        
        // Zones CSV (one row per zone per update)
        zones_csv << "timestamp_ms,zone_id,avg_pressure,min_pressure,flow,pressure_variance,"
                 << "flow_to_pressure_ratio,historical_flow,leak_flag,leak_severity,"
                 << "overflow_flag,pressure_violation\n";
        
        // Valves CSV (one row per valve per update)
        valves_csv << "timestamp_ms,sensor_id,valve_position,last_action_time\n";
        
        // Sensors CSV (sensor volumes)
        sensors_csv << "timestamp_ms,sensor_id,cumulative_volume\n";
        
        // Events CSV
        events_csv << "timestamp_ms,event_id,event_type,sim_timestamp,fresh_water_before,"
                  << "fresh_water_after,sewage_before,sewage_after,duration_seconds,status\n";
        
        // Status CSV
        status_csv << "timestamp_ms,sim_timestamp,fresh_water_level,fresh_water_trend,"
                  << "fresh_water_needs_recharge,fresh_water_time_since_recharge,"
                  << "sewage_level,sewage_status,sewage_needs_treatment,"
                  << "sewage_time_since_treatment,total_demand_m3s,total_sewage_inflow_m3s\n";
        
        // Hourly Usage CSV
        hourly_usage_csv << "timestamp_ms,hour_0,hour_1,hour_2,hour_3,hour_4,hour_5,hour_6,"
                        << "hour_7,hour_8,hour_9,hour_10,hour_11,hour_12,hour_13,hour_14,"
                        << "hour_15,hour_16,hour_17,hour_18,hour_19,hour_20,hour_21,"
                        << "hour_22,hour_23\n";
        
        // Metrics CSV
        metrics_csv << "timestamp_ms,non_revenue_water,supply_efficiency,peak_flow_rate,"
                   << "avg_flow_rate,flow_variance,pressure_compliance,service_continuity,response_time\n";
        
        std::cout << "CSV headers written." << std::endl;
    }
    
public:
    void logRLState(const RLState& state) {
        std::lock_guard<std::mutex> lock(data_mutex);
        rl_state_buffer.push_back(state);
        
        // Write immediately to avoid data loss
        writeRLStateToCSV(state);
        
        // Check if we need to flush
        time_t current_time = time(nullptr);
        if (current_time - last_flush_time >= flush_interval) {
            flushAllData();
            last_flush_time = current_time;
        }
    }
    
    void logEvent(const TreatmentEvent& event) {
        std::lock_guard<std::mutex> lock(data_mutex);
        event_buffer.push_back(event);
        writeEventToCSV(event);
    }
    
    void logStatus(const SystemStatus& status) {
        std::lock_guard<std::mutex> lock(data_mutex);
        status_buffer.push_back(status);
        writeStatusToCSV(status);
    }
    
private:
    void writeRLStateToCSV(const RLState& state) {
        if (!rl_state_csv.is_open()) return;
        
        // Main RL State
        rl_state_csv << state.timestamp_ms << ","
                    << state.source_ip << ","
                    << std::fixed << std::setprecision(2) << state.time_of_day << ","
                    << state.day_type << ","
                    << state.reservoir_level_pct << ","
                    << state.reservoir_trend << ","
                    << state.pump_capacity_available << ","
                    << state.supply_margin << ","
                    << state.sewage_level_pct << ","
                    << state.sewage_status << ","
                    << (state.sewage_needs_treatment ? "true" : "false") << ","
                    << state.sewage_time_since_treatment << ","
                    << state.zone_count << ","
                    << state.sensor_count << ","
                    << state.non_revenue_water << ","
                    << state.supply_efficiency << ","
                    << state.peak_flow_rate << ","
                    << state.avg_flow_rate << ","
                    << state.flow_variance << ","
                    << state.pressure_compliance << ","
                    << state.service_continuity << ","
                    << state.response_time << ","
                    << state.recent_action_count << ","
                    << state.episode_progress << "\n";
        
        // Reservoir
        reservoir_csv << state.timestamp_ms << ","
                     << state.reservoir_level_pct << ","
                     << state.reservoir_trend << ","
                     << state.pump_capacity_available << ","
                     << state.supply_margin << ","
                     << "false,"  // needs_recharge (would need separate field)
                     << "0.0\n";  // time_since_recharge
        
        // Sewage
        sewage_csv << state.timestamp_ms << ","
                  << state.sewage_level_pct << ","
                  << state.sewage_status << ","
                  << (state.sewage_needs_treatment ? "true" : "false") << ","
                  << state.sewage_time_since_treatment << ","
                  << state.last_treatment_event_id << ","
                  << state.time_since_last_treatment << "\n";
        
        // Zones
        for (const auto& zone : state.zones) {
            zones_csv << state.timestamp_ms << ","
                     << zone.zone_id << ","
                     << zone.avg_pressure << ","
                     << zone.min_pressure << ","
                     << zone.flow << ","
                     << zone.pressure_variance << ","
                     << zone.flow_to_pressure_ratio << ","
                     << zone.historical_flow << ","
                     << (zone.leak_flag ? "true" : "false") << ","
                     << zone.leak_severity << ","
                     << (zone.overflow_flag ? "true" : "false") << ","
                     << (zone.pressure_violation ? "true" : "false") << "\n";
        }
        
        // Valves
        for (size_t i = 0; i < state.valve_states.size(); ++i) {
            valves_csv << state.timestamp_ms << ","
                      << i << ","
                      << state.valve_states[i] << ",";
            
            if (i < state.last_action_times.size()) {
                valves_csv << state.last_action_times[i];
            } else {
                valves_csv << "0.0";
            }
            valves_csv << "\n";
        }
        
        // Sensors (cumulative volumes)
        for (size_t i = 0; i < state.cumulative_volumes.size(); ++i) {
            sensors_csv << state.timestamp_ms << ","
                       << i << ","
                       << state.cumulative_volumes[i] << "\n";
        }
        
        // Hourly Usage
        hourly_usage_csv << state.timestamp_ms;
        for (int i = 0; i < 24; ++i) {
            hourly_usage_csv << "," << state.hourly_usage[i];
        }
        hourly_usage_csv << "\n";
        
        // Metrics
        metrics_csv << state.timestamp_ms << ","
                   << state.non_revenue_water << ","
                   << state.supply_efficiency << ","
                   << state.peak_flow_rate << ","
                   << state.avg_flow_rate << ","
                   << state.flow_variance << ","
                   << state.pressure_compliance << ","
                   << state.service_continuity << ","
                   << state.response_time << "\n";
    }
    
    void writeEventToCSV(const TreatmentEvent& event) {
        if (!events_csv.is_open()) return;
        
        events_csv << event.timestamp_ms << ","
                  << event.event_id << ","
                  << event.event_type << ","
                  << std::fixed << std::setprecision(3) << event.sim_timestamp << ","
                  << event.fresh_water_before << ","
                  << event.fresh_water_after << ","
                  << event.sewage_before << ","
                  << event.sewage_after << ","
                  << event.duration_seconds << ","
                  << event.status << "\n";
        
        events_csv.flush();
    }
    
    void writeStatusToCSV(const SystemStatus& status) {
        if (!status_csv.is_open()) return;
        
        status_csv << status.timestamp_ms << ","
                  << std::fixed << std::setprecision(3) << status.sim_timestamp << ","
                  << status.fresh_water_level << ","
                  << status.fresh_water_trend << ","
                  << (status.fresh_water_needs_recharge ? "true" : "false") << ","
                  << status.fresh_water_time_since_recharge << ","
                  << status.sewage_level << ","
                  << status.sewage_status << ","
                  << (status.sewage_needs_treatment ? "true" : "false") << ","
                  << status.sewage_time_since_treatment << ","
                  << status.total_demand_m3s << ","
                  << status.total_sewage_inflow_m3s << "\n";
        
        status_csv.flush();
    }
    
public:
    void flushAllData() {
        std::lock_guard<std::mutex> lock(data_mutex);
        
        rl_state_csv.flush();
        reservoir_csv.flush();
        sewage_csv.flush();
        zones_csv.flush();
        valves_csv.flush();
        sensors_csv.flush();
        events_csv.flush();
        status_csv.flush();
        hourly_usage_csv.flush();
        metrics_csv.flush();
        
        std::cout << "CSV files flushed to disk. Buffer sizes: "
                  << "RL States=" << rl_state_buffer.size()
                  << ", Events=" << event_buffer.size()
                  << ", Status=" << status_buffer.size() << std::endl;
    }
    
    void printStatistics() {
        std::lock_guard<std::mutex> lock(data_mutex);
        
        std::cout << "\n=== CSV LOGGER STATISTICS ===" << std::endl;
        std::cout << "RL States logged: " << rl_state_buffer.size() << std::endl;
        std::cout << "Events logged: " << event_buffer.size() << std::endl;
        std::cout << "Status updates logged: " << status_buffer.size() << std::endl;
        
        if (!rl_state_buffer.empty()) {
            auto& latest = rl_state_buffer.back();
            std::cout << "Latest data: " << std::endl;
            std::cout << "  Reservoir: " << latest.reservoir_level_pct << "%" << std::endl;
            std::cout << "  Sewage: " << latest.sewage_level_pct << "%" << std::endl;
            std::cout << "  Zones: " << latest.zone_count << std::endl;
            std::cout << "  Sensors: " << latest.sensor_count << std::endl;
        }
        
        // Write statistics to summary file
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time));
        
        summary_txt << "\n=== Statistics at " << time_str << " ===" << std::endl;
        summary_txt << "RL States: " << rl_state_buffer.size() << std::endl;
        summary_txt << "Events: " << event_buffer.size() << std::endl;
        summary_txt << "Status Updates: " << status_buffer.size() << std::endl;
        summary_txt.flush();
    }
};

// ============================================================================
// SIMPLE JSON PARSER
// ============================================================================
class SimpleJSONParser {
public:
    static std::map<std::string, std::string> parseObject(const std::string& json) {
        std::map<std::string, std::string> result;
        
        // Remove whitespace
        std::string clean_json;
        for (char c : json) {
            if (!std::isspace(c)) {
                clean_json += c;
            }
        }
        
        // Simple parsing
        size_t pos = clean_json.find('{');
        if (pos == std::string::npos) return result;
        
        while (pos < clean_json.size()) {
            // Find key
            size_t key_start = clean_json.find('"', pos);
            if (key_start == std::string::npos) break;
            size_t key_end = clean_json.find('"', key_start + 1);
            if (key_end == std::string::npos) break;
            
            std::string key = clean_json.substr(key_start + 1, key_end - key_start - 1);
            
            // Find value
            size_t value_start = clean_json.find(':', key_end);
            if (value_start == std::string::npos) break;
            
            size_t value_end = std::string::npos;
            
            // Check if value is a string
            if (clean_json[value_start + 1] == '"') {
                value_start += 1;
                value_end = clean_json.find('"', value_start + 1);
                if (value_end == std::string::npos) break;
                
                std::string value = clean_json.substr(value_start + 1, value_end - value_start - 1);
                result[key] = value;
                
            } else {
                // Number, boolean, or null
                value_start += 1;
                value_end = clean_json.find_first_of(",}", value_start);
                if (value_end == std::string::npos) break;
                
                std::string value = clean_json.substr(value_start, value_end - value_start);
                result[key] = value;
            }
            
            pos = value_end + 1;
        }
        
        return result;
    }
    
    static std::vector<std::string> parseArray(const std::string& json_str) {
        std::vector<std::string> result;
        
        std::string json = json_str;
        // Find array start
        size_t start = json.find('[');
        if (start == std::string::npos) return result;
        
        size_t end = json.find(']', start);
        if (end == std::string::npos) return result;
        
        std::string array_content = json.substr(start + 1, end - start - 1);
        
        // Simple split by comma
        size_t pos = 0;
        while (pos < array_content.size()) {
            size_t comma = array_content.find(',', pos);
            if (comma == std::string::npos) {
                std::string item = array_content.substr(pos);
                if (!item.empty()) result.push_back(item);
                break;
            }
            std::string item = array_content.substr(pos, comma - pos);
            result.push_back(item);
            pos = comma + 1;
        }
        
        return result;
    }
    
    static std::vector<std::map<std::string, std::string>> parseObjectArray(const std::string& json_str) {
        std::vector<std::map<std::string, std::string>> result;
        
        std::string json = json_str;
        size_t start = json.find('[');
        if (start == std::string::npos) return result;
        
        // Find each object
        size_t pos = start;
        while (pos < json.size()) {
            size_t obj_start = json.find('{', pos);
            if (obj_start == std::string::npos) break;
            
            size_t obj_end = json.find('}', obj_start);
            if (obj_end == std::string::npos) break;
            
            std::string obj_str = json.substr(obj_start, obj_end - obj_start + 1);
            result.push_back(parseObject(obj_str));
            
            pos = obj_end + 1;
        }
        
        return result;
    }
    
    static float toFloat(const std::string& str, float default_val = 0.0f) {
        try {
            if (str.empty()) return default_val;
            return std::stof(str);
        } catch (...) {
            return default_val;
        }
    }
    
    static int toInt(const std::string& str, int default_val = 0) {
        try {
            if (str.empty()) return default_val;
            return std::stoi(str);
        } catch (...) {
            return default_val;
        }
    }
    
    static bool toBool(const std::string& str, bool default_val = false) {
        if (str == "true" || str == "1" || str == "True" || str == "TRUE") return true;
        if (str == "false" || str == "0" || str == "False" || str == "FALSE") return false;
        return default_val;
    }
};

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================
CSVLogger csv_logger;
std::queue<std::string> message_queue;
std::mutex queue_mutex;

// Statistics
struct Statistics {
    int total_messages = 0;
    int rl_states = 0;
    int events = 0;
    int status = 0;
    int errors = 0;
    std::chrono::system_clock::time_point start_time;
    
    Statistics() {
        start_time = std::chrono::system_clock::now();
    }
    
    void print() {
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        
        std::cout << "\n=== UDP LISTENER STATISTICS ===" << std::endl;
        std::cout << "Uptime: " << duration.count() << " seconds" << std::endl;
        std::cout << "Total messages: " << total_messages << std::endl;
        std::cout << "RL States: " << rl_states << std::endl;
        std::cout << "Events: " << events << std::endl;
        std::cout << "Status: " << status << std::endl;
        std::cout << "Errors: " << errors << std::endl;
        if (duration.count() > 0) {
            std::cout << "Rate: " << std::fixed << std::setprecision(1) 
                     << (total_messages / (float)duration.count()) << " msg/sec" << std::endl;
        }
        std::cout << "===============================" << std::endl;
    }
};

Statistics stats;

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================
RLState parseRLState(const std::map<std::string, std::string>& data, const std::string& source_ip) {
    RLState state;
    
    // Timestamp
    state.received_time = std::chrono::system_clock::now();
    state.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        state.received_time.time_since_epoch()).count();
    state.source_ip = source_ip;
    
    // Basic fields
    if (data.find("reservoir_level_pct") != data.end())
        state.reservoir_level_pct = SimpleJSONParser::toFloat(data.at("reservoir_level_pct"));
    if (data.find("reservoir_trend") != data.end())
        state.reservoir_trend = data.at("reservoir_trend");
    if (data.find("pump_capacity_available") != data.end())
        state.pump_capacity_available = SimpleJSONParser::toFloat(data.at("pump_capacity_available"));
    if (data.find("supply_margin") != data.end())
        state.supply_margin = SimpleJSONParser::toFloat(data.at("supply_margin"));
    
    // Sewage
    if (data.find("sewage_level_pct") != data.end())
        state.sewage_level_pct = SimpleJSONParser::toFloat(data.at("sewage_level_pct"));
    if (data.find("sewage_status") != data.end())
        state.sewage_status = data.at("sewage_status");
    if (data.find("sewage_needs_treatment") != data.end())
        state.sewage_needs_treatment = SimpleJSONParser::toBool(data.at("sewage_needs_treatment"));
    if (data.find("sewage_time_since_treatment") != data.end())
        state.sewage_time_since_treatment = SimpleJSONParser::toFloat(data.at("sewage_time_since_treatment"));
    
    // Event info
    if (data.find("last_treatment_event_id") != data.end())
        state.last_treatment_event_id = SimpleJSONParser::toInt(data.at("last_treatment_event_id"));
    if (data.find("time_since_last_treatment") != data.end())
        state.time_since_last_treatment = SimpleJSONParser::toFloat(data.at("time_since_last_treatment"));
    if (data.find("time_since_last_recharge") != data.end())
        state.time_since_last_recharge = SimpleJSONParser::toFloat(data.at("time_since_last_recharge"));
    
    // Time
    if (data.find("time_of_day") != data.end())
        state.time_of_day = SimpleJSONParser::toFloat(data.at("time_of_day"));
    if (data.find("day_type") != data.end())
        state.day_type = data.at("day_type");
    if (data.find("recent_action_count") != data.end())
        state.recent_action_count = SimpleJSONParser::toInt(data.at("recent_action_count"));
    if (data.find("sim_time_sin") != data.end())
        state.sim_time_sin = SimpleJSONParser::toFloat(data.at("sim_time_sin"));
    if (data.find("sim_time_cos") != data.end())
        state.sim_time_cos = SimpleJSONParser::toFloat(data.at("sim_time_cos"));
    if (data.find("episode_progress") != data.end())
        state.episode_progress = SimpleJSONParser::toFloat(data.at("episode_progress"));
    
    // Metrics
    if (data.find("non_revenue_water") != data.end())
        state.non_revenue_water = SimpleJSONParser::toFloat(data.at("non_revenue_water"));
    if (data.find("supply_efficiency") != data.end())
        state.supply_efficiency = SimpleJSONParser::toFloat(data.at("supply_efficiency"));
    if (data.find("peak_flow_rate") != data.end())
        state.peak_flow_rate = SimpleJSONParser::toFloat(data.at("peak_flow_rate"));
    if (data.find("avg_flow_rate") != data.end())
        state.avg_flow_rate = SimpleJSONParser::toFloat(data.at("avg_flow_rate"));
    if (data.find("flow_variance") != data.end())
        state.flow_variance = SimpleJSONParser::toFloat(data.at("flow_variance"));
    if (data.find("pressure_compliance") != data.end())
        state.pressure_compliance = SimpleJSONParser::toFloat(data.at("pressure_compliance"));
    if (data.find("service_continuity") != data.end())
        state.service_continuity = SimpleJSONParser::toFloat(data.at("service_continuity"));
    if (data.find("response_time") != data.end())
        state.response_time = SimpleJSONParser::toFloat(data.at("response_time"));
    
    // Counts
    if (data.find("zone_count") != data.end())
        state.zone_count = SimpleJSONParser::toInt(data.at("zone_count"));
    if (data.find("sensor_count") != data.end())
        state.sensor_count = SimpleJSONParser::toInt(data.at("sensor_count"));
    
    return state;
}

TreatmentEvent parseEvent(const std::map<std::string, std::string>& data) {
    TreatmentEvent event;
    
    // Use current time for timestamp
    auto now = std::chrono::system_clock::now();
    event.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    if (data.find("event_id") != data.end())
        event.event_id = SimpleJSONParser::toInt(data.at("event_id"));
    if (data.find("event_type") != data.end())
        event.event_type = data.at("event_type");
    if (data.find("timestamp") != data.end())
        event.sim_timestamp = SimpleJSONParser::toFloat(data.at("timestamp"));
    if (data.find("fresh_water_before") != data.end())
        event.fresh_water_before = SimpleJSONParser::toFloat(data.at("fresh_water_before"));
    if (data.find("fresh_water_after") != data.end())
        event.fresh_water_after = SimpleJSONParser::toFloat(data.at("fresh_water_after"));
    if (data.find("sewage_before") != data.end())
        event.sewage_before = SimpleJSONParser::toFloat(data.at("sewage_before"));
    if (data.find("sewage_after") != data.end())
        event.sewage_after = SimpleJSONParser::toFloat(data.at("sewage_after"));
    if (data.find("duration_seconds") != data.end())
        event.duration_seconds = SimpleJSONParser::toFloat(data.at("duration_seconds"));
    if (data.find("status") != data.end())
        event.status = data.at("status");
    
    return event;
}

SystemStatus parseStatus(const std::map<std::string, std::string>& data) {
    SystemStatus status;
    
    // Use current time for timestamp
    auto now = std::chrono::system_clock::now();
    status.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    if (data.find("timestamp") != data.end())
        status.sim_timestamp = SimpleJSONParser::toFloat(data.at("timestamp"));
    if (data.find("fresh_water_level") != data.end())
        status.fresh_water_level = SimpleJSONParser::toFloat(data.at("fresh_water_level"));
    if (data.find("fresh_water_trend") != data.end())
        status.fresh_water_trend = data.at("fresh_water_trend");
    if (data.find("fresh_water_needs_recharge") != data.end())
        status.fresh_water_needs_recharge = SimpleJSONParser::toBool(data.at("fresh_water_needs_recharge"));
    if (data.find("fresh_water_time_since_recharge") != data.end())
        status.fresh_water_time_since_recharge = SimpleJSONParser::toFloat(data.at("fresh_water_time_since_recharge"));
    if (data.find("sewage_level") != data.end())
        status.sewage_level = SimpleJSONParser::toFloat(data.at("sewage_level"));
    if (data.find("sewage_status") != data.end())
        status.sewage_status = data.at("sewage_status");
    if (data.find("sewage_needs_treatment") != data.end())
        status.sewage_needs_treatment = SimpleJSONParser::toBool(data.at("sewage_needs_treatment"));
    if (data.find("sewage_time_since_treatment") != data.end())
        status.sewage_time_since_treatment = SimpleJSONParser::toFloat(data.at("sewage_time_since_treatment"));
    if (data.find("total_demand_m3s") != data.end())
        status.total_demand_m3s = SimpleJSONParser::toFloat(data.at("total_demand_m3s"));
    if (data.find("total_sewage_inflow_m3s") != data.end())
        status.total_sewage_inflow_m3s = SimpleJSONParser::toFloat(data.at("total_sewage_inflow_m3s"));
    
    return status;
}

void processMessage(const std::string& message, const std::string& source_ip) {
    stats.total_messages++;
    
    try {
        // Check message type
        if (message.find("\"reservoir_level_pct\"") != std::string::npos) {
            // RL State message
            auto data = SimpleJSONParser::parseObject(message);
            RLState state = parseRLState(data, source_ip);
            csv_logger.logRLState(state);
            stats.rl_states++;
            
            // Display every 10th message
            static int counter = 0;
            if (counter++ % 10 == 0) {
                std::cout << "📡 RL State: Reservoir=" << state.reservoir_level_pct 
                         << "%, Sewage=" << state.sewage_level_pct << "%" << std::endl;
            }
            
        } else if (message.find("TREATMENT_EVENT:") != std::string::npos || 
                  message.find("RECHARGE_EVENT:") != std::string::npos) {
            // Event message
            size_t colon_pos = message.find(':');
            if (colon_pos != std::string::npos) {
                std::string json_str = message.substr(colon_pos + 1);
                auto data = SimpleJSONParser::parseObject(json_str);
                TreatmentEvent event = parseEvent(data);
                csv_logger.logEvent(event);
                stats.events++;
                
                std::cout << "🔔 Event: " << event.event_type << " #" << event.event_id 
                         << " (" << event.status << ")" << std::endl;
            }
            
        } else if (message.find("WATER_SYSTEM_STATUS:") != std::string::npos) {
            // Status message
            size_t colon_pos = message.find(':');
            if (colon_pos != std::string::npos) {
                std::string json_str = message.substr(colon_pos + 1);
                auto data = SimpleJSONParser::parseObject(json_str);
                SystemStatus status = parseStatus(data);
                csv_logger.logStatus(status);
                stats.status++;
            }
            
        } else {
            // Unknown message type
            stats.errors++;
            if (stats.errors < 5) {
                std::cout << "❓ Unknown message: " << message.substr(0, 100) << "..." << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error processing message: " << e.what() << std::endl;
        stats.errors++;
    }
}

// ============================================================================
// UDP LISTENER
// ============================================================================
int createUDPSocket() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "ERROR: Failed to create UDP socket" << std::endl;
        return -1;
    }
    
    int reuse = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    return sockfd;
}

bool bindSocket(int sockfd) {
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_LISTEN_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "ERROR: Failed to bind to port " << UDP_LISTEN_PORT << std::endl;
        return false;
    }
    
    std::cout << "Listening on UDP port " << UDP_LISTEN_PORT << std::endl;
    return true;
}

void udpListenerThread() {
    int sockfd = createUDPSocket();
    if (sockfd < 0) return;
    
    if (!bindSocket(sockfd)) {
        close(sockfd);
        return;
    }
    
    char buffer[BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    std::cout << "UDP listener started. Data will be saved to CSV files." << std::endl;
    
    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        int activity = select(sockfd + 1, &readfds, NULL, NULL, &timeout);
        
        if (activity > 0 && FD_ISSET(sockfd, &readfds)) {
            ssize_t bytes_received = recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0,
                                            (struct sockaddr*)&client_addr, &addr_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                std::string message(buffer);
                std::string source_ip = inet_ntoa(client_addr.sin_addr);
                
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    message_queue.push(message);
                    // Store IP for processing
                    message_queue.push(source_ip);
                }
            }
        }
        
        // Process queued messages
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            while (message_queue.size() >= 2) { // Message + IP
                std::string message = message_queue.front();
                message_queue.pop();
                std::string source_ip = message_queue.front();
                message_queue.pop();
                
                processMessage(message, source_ip);
            }
        }
    }
    
    close(sockfd);
    std::cout << "UDP listener stopped" << std::endl;
}

// ============================================================================
// COMMAND LINE INTERFACE
// ============================================================================
void commandLineInterface() {
    std::cout << "\nWater System Data Collector - Commands:" << std::endl;
    std::cout << "  stats    - Show statistics" << std::endl;
    std::cout << "  flush    - Force flush data to disk" << std::endl;
    std::cout << "  files    - Show created files" << std::endl;
    std::cout << "  exit     - Exit program" << std::endl;
    std::cout << "\n> ";
    
    std::string command;
    while (running) {
        if (std::cin >> command) {
            if (command == "stats") {
                stats.print();
                csv_logger.printStatistics();
            } else if (command == "flush") {
                csv_logger.flushAllData();
                std::cout << "Data flushed to disk" << std::endl;
            } else if (command == "files") {
                std::cout << "\nOutput directory: " << OUTPUT_DIR << std::endl;
                std::cout << "CSV files in: " << CSV_DIR << std::endl;
                std::cout << "Logs in: " << LOGS_DIR << std::endl;
                
                // List files
                try {
                    std::cout << "\nCSV Files:" << std::endl;
                    for (const auto& entry : fs::directory_iterator(CSV_DIR)) {
                        if (entry.is_regular_file()) {
                            std::cout << "  " << entry.path().filename() << std::endl;
                        }
                    }
                } catch (...) {
                    std::cout << "Could not list directory" << std::endl;
                }
            } else if (command == "exit") {
                running = false;
                std::cout << "Exiting..." << std::endl;
                break;
            } else {
                std::cout << "Unknown command. Try: stats, flush, files, exit" << std::endl;
            }
            std::cout << "\n> ";
        }
    }
}

// ============================================================================
// SIGNAL HANDLER
// ============================================================================
void signalHandler(int signum) {
    std::cout << "\nSignal " << signum << " received. Shutting down..." << std::endl;
    running = false;
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "  WATER SYSTEM DATA COLLECTOR - CSV LOGGER" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "UDP Port: " << UDP_LISTEN_PORT << std::endl;
    std::cout << "Output Directory: " << OUTPUT_DIR << std::endl;
    std::cout << "CSV Files: " << CSV_DIR << std::endl;
    std::cout << "Logs: " << LOGS_DIR << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "Listening for data... (Ctrl+C to exit)" << std::endl;
    
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Start UDP listener thread
    std::thread udp_thread(udpListenerThread);
    
    // Start command line interface
    commandLineInterface();
    
    // Wait for UDP thread to finish
    if (udp_thread.joinable()) {
        udp_thread.join();
    }
    
    // Final statistics
    stats.print();
    csv_logger.printStatistics();
    
    std::cout << "\nData collection complete." << std::endl;
    std::cout << "All CSV files saved in: " << OUTPUT_DIR << std::endl;
    
    return 0;
}