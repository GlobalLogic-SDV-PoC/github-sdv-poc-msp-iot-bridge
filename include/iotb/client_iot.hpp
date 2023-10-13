#pragma once
#include <cstdint>
#include <functional>

#include "nlohmann/json.hpp"
#include "context.hpp"

namespace iotb
{
struct Span
{
    void* buf = nullptr;
    std::size_t len = 0;
};

class IClientIot
{
public:
    using onReceivedHandler = std::function<void(Span /*topic*/, Span /*payload*/)>;

public:
    virtual void connect(const std::shared_ptr<Context>& ctx, const nlohmann::json& config, const onReceivedHandler& rec) = 0;
    virtual void disconnect() = 0;
    virtual void subscribe(Span topic) = 0;
    virtual void unsubscribe(Span topic) = 0;
    virtual void publish(Span topic, Span payload) = 0;

    virtual ~IClientIot() = default;
};

}  // namespace iotb