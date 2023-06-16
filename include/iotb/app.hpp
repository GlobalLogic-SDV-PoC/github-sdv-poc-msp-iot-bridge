#pragma once

#include "interface/client_iot.hpp"
#include "ipc/server.hpp"

namespace iotb
{
namespace net = ipc::net;
class App
{
    using work_guard = net::executor_work_guard<net::io_context::executor_type>;

public:
    App(const std::shared_ptr<IClientIot>& client_iot, nlohmann::json config);
    ~App();
    void init();
    void start();
    void stop();
    void initDefaultLogger(std::string_view filepath,
                           size_t max_size,
                           size_t max_files,
                           std::chrono::seconds flush_interval);
    const std::string& getLogDir() const;

private:
    void onIpcReceive(size_t uuid, const std::shared_ptr<ipc::Packet>& packet, std::weak_ptr<ipc::Server> server);
    void onIotReceive(std::string_view topic, std::string_view data);

    void initIpc();
    void initIotBridge();

private:
    net::io_context m_context;
    work_guard m_work_guard;
    nlohmann::json m_config;
    std::shared_ptr<IClientIot> m_client_iot;
    std::shared_ptr<ipc::Server> m_ipc_server;
    std::string m_log_dir;
    // TODO: wildcard support
    std::vector<std::pair<size_t, std::string>> m_subscriptions;
};
}  // namespace iotb