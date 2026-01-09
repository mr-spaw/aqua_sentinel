#ifndef ABBA0285_EB4A_4018_89A7_CCA104D23C51
#define ABBA0285_EB4A_4018_89A7_CCA104D23C51
    //g++ mqtt_subscriber.cpp -o mqtt_rx -lmosquitto

    #include <mosquitto.h>
    #include <iostream>
    #include <string>
    #include <map>
    #include <vector>
    #include <nlohmann/json.hpp>
    #include <iomanip>


    using json = nlohmann::json;

    // Store chunks temporarily
    std::map<int, json> chunk_buffer;
    int expected_chunks = -1;

    void on_message(struct mosquitto *, void *, const struct mosquitto_message *msg)
    {
        std::string topic = msg->topic;
        std::string payload((char*)msg->payload, msg->payloadlen);

        if (topic.find("pipes/sensors1/all") == std::string::npos) {
            return;
        }

        try {
            auto j = json::parse(payload);

            if (!j.contains("sensors")) return;

            int chunk = j.value("chunk", -1);
            int total = j.value("total_chunks", -1);

            if (chunk < 0 || total <= 0) return;

            expected_chunks = total;
            chunk_buffer[chunk] = j["sensors"];

            // Wait until ALL chunks arrive
            if ((int)chunk_buffer.size() < expected_chunks) {
                return;
            }

            // ================= FULL FRAME RECEIVED =================
            std::cout << "\n================ FULL SENSOR FRAME ================\n";

            int printed = 0;

            for (int i = 0; i < expected_chunks; i++) {
                for (const auto &s : chunk_buffer[i]) {

                    int id = s["sensor_id"].get<int>();
                    std::cout << std::fixed << std::setprecision(3);


                    std::cout
                        << "SENSOR_ID=" << id
                        << " | PIPE_ID=" << s["pipe_id"]
                        << " | TYPE=" << s["type"]
                        << " | PRESSURE=" << s["pressure_bar"].get<double>() << " bar"
                        << " | LEVEL=" << s["level_pct"].get<double>() << "%"
                        << " | VALVE=" << (s["valve"].get<int>() ? "OPEN" : "CLOSED")
                        << "\n";

                    printed++;
                }
            }

            std::cout << "==================================================\n";
            std::cout << "TOTAL SENSORS RECEIVED: " << printed << "\n";
            std::cout << "==================================================\n";

            // Reset for next frame
            chunk_buffer.clear();
            expected_chunks = -1;

        } catch (const std::exception &e) {
            std::cerr << "JSON error: " << e.what() << "\n";
        }
    }

    int main()
    {
        mosquitto_lib_init();

        mosquitto *mosq = mosquitto_new("linux_rx", true, nullptr);
        if (!mosq) {
            std::cerr << "Failed to create mosquitto client\n";
            return 1;
        }

        mosquitto_message_callback_set(mosq, on_message);

        if (mosquitto_connect(mosq, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
            std::cerr << "MQTT connection failed\n";
            return 1;
        }

        mosquitto_subscribe(mosq, nullptr, "pipes/sensors1/all/#", 0);

        std::cout << "Listening on pipes/sensors1/all/#\n";

        mosquitto_loop_forever(mosq, -1, 1);

        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }


#endif /* ABBA0285_EB4A_4018_89A7_CCA104D23C51 */
