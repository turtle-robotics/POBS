#include <uwebsockets/App.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <cmath>

using json = nlohmann::json;

int main() {
    uWS::App().ws<false>("/*", {
        .open = [](auto *ws) {
            std::cout << "Client connected!" << std::endl;
        },
        .message = [](auto *ws, std::string_view msg, uWS::OpCode) {
            // Could handle commands from the web page here
        }
    }).listen(9002, [](auto *listen_socket) {
        if (listen_socket)
            std::cout << "Listening on port 9002\n";
    }).run();
}

// Example function to broadcast thruster data
void sendThrusterData(auto *ws) {
    static double t = 0;
    t += 0.05;
    json data;
    for (int i = 0; i < 4; ++i) {
        data["thrusters"].push_back({
            {"angle", i * 90 + 45 * std::sin(t + i)},
            {"thrust", 0.5 + 0.5 * std::sin(t + i)}
        });
    }
    ws->send(data.dump(), uWS::OpCode::TEXT);
}
