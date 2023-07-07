#pragma once

#include <string_view>
#include <functional>
#include <nlohmann/json.hpp>

namespace iotb
{
class IClientIot
{
public:
    using on_received_handler = std::function<void(std::string_view /*topic*/, std::string_view /*payload*/)>;

public:
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void subscribe(std::string_view topic) = 0;
    virtual void unsubscribe(std::string_view topic) = 0;
    virtual void publish(std::string_view topic, std::string_view payload) = 0;
    virtual void setOnReceivedHandler(const on_received_handler& handler) = 0;
    virtual void setConfig(const nlohmann::json& config)
        = 0;
        
    virtual ~IClientIot() = default;
};
}  // namespace iotb