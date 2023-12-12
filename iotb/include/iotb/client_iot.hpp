#pragma once
#include <cstdint>
#include <functional>
#include <string>

#include "nlohmann/json.hpp"
#include "context.hpp"

namespace iotb
{


class IClientIot
{
public:
    using onReceivedHandler = std::function<void(std::string/*topic*/, std::string /*payload*/)>;

public:
    virtual void connect(const std::shared_ptr<Context>& ctx, const nlohmann::json& config, const onReceivedHandler& rec) = 0;
    virtual void disconnect() = 0;
    virtual void subscribe(std::string topic) = 0;
    virtual void unsubscribe(std::string topic) = 0;
    virtual void publish(std::string topic, std::string payload) = 0;

    virtual ~IClientIot() = default;
};
}  // namespace iotb
