#ifndef CEB7C70B_8DBC_47CE_8E56_3EE4C8BC04E4
#define CEB7C70B_8DBC_47CE_8E56_3EE4C8BC04E4
// g++ publish_init.cpp -o publish_init -lmosquitto

#include <mosquitto.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main() {
    mosquitto_lib_init();

    mosquitto *mosq = mosquitto_new("init_publisher", true, nullptr);
    if (!mosq) {
        std::cerr << "Failed to create MQTT client\n";
        return 1;
    }

    if (mosquitto_connect(mosq, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "MQTT connection failed\n";
        return 1;
    }

    std::ifstream f("/home/arka/aqua_sentinel/simulator/init_sensor.json");
    if (!f.is_open()) {
        std::cerr << "Cannot open init_sensor.json\n";
        return 1;
    }

    json sim_json;
    f >> sim_json;

    json out;
    out["timestamp"] = sim_json["timestamp"];
    out["dt"] = 1.0;

    json sensors = json::array();

    for (auto &s : sim_json["sensors"]) {
        json so;
        so["sensor_id"] = s["sensor_id"];
        so["pipe_id"]   = s["pipe_id"];

        so["type"] = (s["pipe_type"].get<int>() == 0) ? "fresh" : "sewage";

        so["pressure_bar"] = s["pressure_bar"];
        so["level_pct"]    = s["level_pct"];

        so["valve"] = s.contains("valve") ? s["valve"].get<int>() : 1;


        sensors.push_back(so);
    }

    out["sensors"] = sensors;

    std::string payload = out.dump();

    mosquitto_publish(
        mosq,
        nullptr,
        "pipes/config/init1",
        payload.size(),
        payload.c_str(),
        1,     // QoS 1
        true   // retained
    );

    mosquitto_loop(mosq, 200, 1);

    std::cout << "Published init config for "
              << sensors.size()
              << " sensors\n";

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
}


#endif /* CEB7C70B_8DBC_47CE_8E56_3EE4C8BC04E4 */
