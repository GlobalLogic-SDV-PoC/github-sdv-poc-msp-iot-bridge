#pragma once
#include <unordered_map>

#include "context.hpp"
#include "iotb/client_iot.hpp"
#include "nlohmann/json.hpp"

namespace iotb
{
class App
{
    using TopicsMap = std::unordered_map<std::string, std::vector<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>>>;
public:
    App(const std::shared_ptr<IClientIot>& iot_client);
    void start();
    void stop();

private:
    void on_iot_received(Span topic, Span payload);
    void on_ipc_received(const std_msgs::msg::String& msg);

    void init();
    void parse_config();

    std::shared_ptr<IClientIot> m_iot_client;

    std::shared_ptr<Context> m_ctx;
    TopicsMap m_topics;
    nlohmann::json m_config;
};
}  // namespace iotb
