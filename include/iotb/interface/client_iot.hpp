#pragma once

#include <string_view>
#include <functional>

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
    virtual void setCredentials(std::string_view endpoint_path,
                                std::string_view cert_path,
                                std::string_view key_path,
                                std::string_view ca_path,
                                std::string_view clientId_path)
        = 0;
        
    virtual ~IClientIot() = default;
};
}  // namespace iotb