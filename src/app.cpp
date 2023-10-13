#include "iotb/app.hpp"

#include <fstream>
#include <rclcpp/logging.hpp>

#include "iotb/client_iot.hpp"

using namespace std::placeholders;
namespace iotb
{
App::App(const std::shared_ptr<IClientIot>& iot_client)
    : m_iot_client(iot_client)
{
    init();
    parse_config();
    m_ctx->iot = m_ctx->node->create_subscription<std_msgs::msg::String>(m_config["iot_message_topic"], 60, std::bind(&App::on_ipc_received, this, _1));
}
void App::start()
{
    m_iot_client->connect(m_ctx, m_config["iot"], std::bind(&App::on_iot_received, this, _1, _2));
    rclcpp::spin(m_ctx->node);
}
void App::on_iot_received(Span topic, Span payload)
{
    std::string str_topic(static_cast<char*>(topic.buf), topic.len);
    const auto it = m_topics.find(str_topic);
    if (it == m_topics.end())
    {
        RCLCPP_WARN(m_ctx->node->get_logger(), "No subscribed nodes to topic %s", str_topic.data());
        return;
    }
    for (const auto& sub : it->second)
    {
        auto message = std_msgs::msg::String();
        message.data = {static_cast<char*>(payload.buf), payload.len};
        sub->publish(message);
        RCLCPP_INFO(m_ctx->node->get_logger(), "Forwarding from %s to %s", str_topic.data(), message.data.data());
    }
}
void App::on_ipc_received(const std_msgs::msg::String& msg)
{
    auto command = nlohmann::json::parse(msg.data);
    auto topic = command["topic"].get_ptr<std::string*>();
    auto action = command["action"].get_ptr<std::string*>();

    if (!topic || !action)
    {
        RCLCPP_ERROR(m_ctx->node->get_logger(), "Invalid msg: %s", msg.data.data());
        return;
    }
    if (*action == "forward")
    {
        auto payload = command["payload"].get_ptr<std::string*>();
        m_iot_client->publish({topic->data(), topic->size()}, {payload->data(), payload->size()});

        RCLCPP_INFO(m_ctx->node->get_logger(), "Forwarding msg: %s", msg.data.data());
    }
    else if (*action == "subscribe")
    {
        const auto it = m_topics.find(*topic);
        auto sub = m_ctx->node->create_publisher<std_msgs::msg::String>(*topic, 5);
        if (it == m_topics.end())
        {
            m_topics.emplace_hint(it, *topic, std::vector<decltype(sub)>{std::move(sub)});
        }
        else
        {
            it->second.push_back(std::move(sub));
        }
        RCLCPP_INFO(m_ctx->node->get_logger(), "Subscribing to topic: %s", topic->data());
    }
    else if (*action == "unsubscribe")
    {
        const auto it = m_topics.find(*topic);
        if (it == m_topics.end())
        {
            RCLCPP_ERROR(m_ctx->node->get_logger(), "No subs to topic %s exists", topic->data());
            return;
        }
        auto& cont = it->second;
        using value_t = typename std::decay_t<decltype(cont)>::value_type;
        cont.erase(std::remove_if(cont.begin(), cont.end(), [topic](const value_t& val)
                                  { return *topic == val->get_topic_name(); }),
                   cont.end());
    }
    else
    {
        RCLCPP_ERROR(m_ctx->node->get_logger(), "Not a valid command: %s", topic->data());
    }
}
void App::stop()
{
    m_iot_client->disconnect();
    rclcpp::shutdown();
}
void App::init()
{
    m_ctx = std::make_shared<Context>();
    m_ctx->node = std::make_shared<rclcpp::Node>("iot");
    m_ctx->node->declare_parameter("config_path", "");
}
void App::parse_config()
{
    std::string config_path;
    if (!m_ctx->node->get_parameter("config_path", config_path))
    {
        RCLCPP_FATAL(m_ctx->node->get_logger(), "Parameter 'config_path' not found. Terminating");
        exit(-1);
    }
    if (config_path.empty())
    {
        RCLCPP_FATAL(m_ctx->node->get_logger(), "Parameter 'config_path' must not be empty. Terminating");
        exit(-1);
    }
    try
    {
        std::ifstream config(config_path);
        config.seekg(std::ios_base::beg);
        m_config = nlohmann::json::parse(config);
    }
    catch (std::exception& e)
    {
        RCLCPP_FATAL(m_ctx->node->get_logger(), "Failed to use config from parameter %s due to %s. Terminating", config_path.data(), e.what());
        exit(-1);
    }
}
}  // namespace iotb