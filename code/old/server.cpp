#include "httplib.h"
#include "json.hpp"
#include <cmath>
#include <iostream>

//Switch to websocket eventually

using json = nlohmann::json;

int main() {
    httplib::Server svr;
    svr.set_mount_point("/", "./www");
    svr.Get("/status", [](const httplib::Request &, httplib::Response &res) {
        static double t = 0;
        t += 0.05;

        json data;
        for (int i = 0; i < 4; ++i) {
            data["thrusters"].push_back({
                {"angle", i * 90 + 45 * std::sin(t + i)},
                {"thrust", 0.5 + 0.5 * std::sin(t + i)}
            });
        }

        data["imu"] = {
            {"roll", 10 * std::sin(t)},
            {"pitch", 5 * std::sin(t / 2)},
            {"yaw", fmod(t * 20, 360.0)}
        };

        res.set_content(data.dump(), "application/json");
    });

    std::cout << "Listening on http://localhost:8080\n";
    svr.listen("0.0.0.0", 8080);
}
