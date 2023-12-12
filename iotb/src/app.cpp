#include "iotb/app.hpp"

#include "iotb/client_iot.hpp"
#include "ipc/server.hpp"

#include <fstream>
#include <ipc/interface/server.hpp>
#include <ipc/server.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

using namespace std::placeholders;

namespace iotb
{
App::App(const std::shared_ptr<IClientIot>& iot_client)
    : m_iot_client(iot_client)
{
    init();
    parse_config();
}

void App::start()
{
    m_iot_client->connect(m_ctx, m_config["iot"], std::bind(&App::on_iot_received, this, _1, _2));
    m_ipc_client->start(m_config["ipc"], std::bind(&App::on_ipc_received, this, _1, _2));
    rclcpp::spin(m_ctx->node);
}

void App::on_iot_received(std::string topic, std::string payload)
{
    const auto it = m_subscriptions.find(topic);
    if (it == m_subscriptions.end())
    {
        RCLCPP_WARN(m_ctx->node->get_logger(), "No subscribed nodes to topic %s", topic.data());
        return;
    }
    for (const auto& sub : it->second)
    {
        auto message = std_msgs::msg::String();
        message.data = std::move(payload);
        sub.second->publish(std::move(message));
        RCLCPP_INFO(m_ctx->node->get_logger(), "Forwarding from %s to %s", topic.data(), message.data.data());
    }
}

void App::on_ipc_received(ipc::IServer::RequestSharedPtr req, ipc::IServer::ResponseSharedPtr resp)
{
    if (req->type == ipc::IServer::RequestSharedPtr::element_type::HELLO_TYPE)
    {
        // TODO: maybe add keepalive functionality
        RCLCPP_INFO(m_ctx->node->get_logger(), "Received hello from %s", req->topic.c_str());
    }
    else if (req->type == ipc::IServer::RequestSharedPtr::element_type::GOODBYE_TYPE)
    {
        RCLCPP_INFO(m_ctx->node->get_logger(), "Received shutdown notification from %s, unsubscribing",
                    req->topic.c_str());
        for (auto it = std::begin(m_subscriptions); it != std::end(m_subscriptions);)
        {
            it->second.erase(req->topic);
            if (it->second.empty())
            {
                // last topic listener => unsubscribe and delete self
                m_iot_client->unsubscribe(it->first);
                it = m_subscriptions.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
    else if (req->type == ipc::IServer::RequestSharedPtr::element_type::FORWARD_TYPE)
    {
        m_iot_client->publish(req->topic, req->payload);
        RCLCPP_INFO(m_ctx->node->get_logger(), "Forwarding msg: %s %s", req->topic.c_str(), req->payload.c_str());
    }
    else if (req->type == ipc::IServer::RequestSharedPtr::element_type::SUBSCRIBE_TYPE)
    {
        const auto it = m_subscriptions.find(req->topic);
        auto sub = m_ctx->node->create_publisher<std_msgs::msg::String>(req->topic, 5);
        if (it == m_subscriptions.end())
        {
            // no sub to given topic => subscribe
            m_iot_client->subscribe(req->topic);
            m_subscriptions.emplace_hint(
                it, req->topic, decltype(m_subscriptions)::mapped_type{std::make_pair(req->payload, std::move(sub))});
        }
        else
        {
            // sub is present => just add to subscription list
            it->second.emplace(req->payload, std::move(sub));
        }
        RCLCPP_INFO(m_ctx->node->get_logger(), "Subscribing to topic: %s, response address %s", req->topic.c_str(),
                    req->payload.c_str());
    }
    else if (req->type == ipc::IServer::RequestSharedPtr::element_type::UNSUBSCRIBE_TYPE)
    {
        const auto it = m_subscriptions.find(req->topic);
        if (it == m_subscriptions.end())
        {
            RCLCPP_ERROR(m_ctx->node->get_logger(), "No subs to topic %s exists", req->topic.c_str());
            return;
        }
        auto& cont = it->second;
        // payload is the response_address name
        cont.erase(req->payload);
        // all unsubbed => not tracking
        if (cont.empty())
        {
            m_iot_client->unsubscribe(req->topic);
        }
    }
    else
    {
        RCLCPP_ERROR(m_ctx->node->get_logger(), "Not a valid packet destination: %d topic: %s payload: %s", req->type,
                     req->topic.c_str(), req->payload.c_str());
        resp->status_code = ipc::IServer::ResponseSharedPtr::element_type::FAILED;
        return;
    }
    resp->status_code = ipc::IServer::ResponseSharedPtr::element_type::ACK;
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
    m_ipc_client = std::make_shared<ipc::Server>("iot_server");

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
        RCLCPP_FATAL(m_ctx->node->get_logger(), "Failed to use config from parameter %s due to %s. Terminating",
                     config_path.data(), e.what());
        exit(-1);
    }
}
}  // namespace iotb