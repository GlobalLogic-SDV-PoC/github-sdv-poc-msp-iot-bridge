#pragma once
#include <ipc/interface/client.hpp>
#include <ipc/interface/server.hpp>
#include <memory>
#include <unordered_map>

#include "context.hpp"
#include "iotb/client_iot.hpp"
#include "ipc/server.hpp"
#include "nlohmann/json.hpp"

namespace iotb
{
class App
{
    // topic -> { reponse_address , response_sender }
    using TopicsMap = std::unordered_map<std::string, std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>>;

public:
    App(const std::shared_ptr<IClientIot>& iot_client);
    void start();
    void stop();

private:
    void on_iot_received(std::string topic, std::string payload);
    void on_ipc_received(ipc::IServer::RequestSharedPtr req, ipc::IServer::ResponseSharedPtr resp);

    void init();
    void parse_config();

private:
    std::shared_ptr<IClientIot> m_iot_client;
    std::shared_ptr<ipc::IServer> m_ipc_client;
    std::shared_ptr<Context> m_ctx;
    TopicsMap m_subscriptions;
    nlohmann::json m_config;
};
}  // namespace iotb
