#include "iotb/app.hpp"

#include "iotb/fs.hpp"
#include "ipc/util.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#ifdef __ANDROID__
#include "spdlog/sinks/android_sink.h"
#else
#include "spdlog/sinks/stdout_color_sinks.h"
#endif

namespace iotb
{
App::App(const std::shared_ptr<IClientIot>& client_iot, nlohmann::json config)
    : m_work_guard(net::make_work_guard(m_context))
    , m_config(std::move(config))
    , m_client_iot(client_iot)

{
}

void App::init()
{
    SPDLOG_DEBUG("[iotb] initializing app");
    initIpc();
    initIotBridge();
    SPDLOG_DEBUG("[iotb] initializing app: DONE");
}

void App::initIpc()
{
    SPDLOG_DEBUG("[iotb] initializing ipc server");
    auto& ipc_json_config = m_config["ipc"];
    ipc::Server::Config ipc_server_config;
    ipc_server_config.endpoint = ipc::tcp::endpoint(ipc::tcp::v4(), ipc_json_config["port"]);
    ipc_server_config.max_listen_connections = ipc_json_config["max_listen_connections"];
    ipc_server_config.reuse_address = ipc_json_config["reuse_address"];
    ipc_server_config.enable_connection_aborted = ipc_json_config["enable_connection_aborted"];
    ipc_server_config.header_buffer_size = ipc_json_config["header_buffer_size"];
    ipc_server_config.body_buffer_size = ipc_json_config["body_buffer_size"];
    ipc_server_config.on_receive_handler = ipc::bind_front(&App::onIpcReceive, this);
    m_ipc_server = std::make_shared<ipc::Server>(m_context, std::move(ipc_server_config));
    SPDLOG_DEBUG("[iotb] initializing ipc server: DONE");
}

void App::initIotBridge()
{
    SPDLOG_DEBUG("[iotb] initializing iot bridge");
    auto& iot_config = m_config["iot"];
    m_client_iot->setCredentials(iot_config["endpoint"].get<std::string_view>(),
                                 iot_config["certificate"].get<std::string_view>(),
                                 iot_config["private"].get<std::string_view>(),
                                 iot_config["root"].get<std::string_view>(),
                                 iot_config["clientId"].get<std::string_view>());
    m_client_iot->setOnReceivedHandler(ipc::bind_front(&App::onIotReceive, this));
    SPDLOG_DEBUG("[iotb] initializing iot bridge: DONE");
}

App::~App()
{
    stop();
}

void App::start()
{
    SPDLOG_DEBUG("[iotb] starting app");
    m_ipc_server->start();
    m_client_iot->connect();
    m_context.run();
}

void App::stop()
{
    SPDLOG_DEBUG("[iotb] stopping app");
    m_ipc_server->stop();
    m_client_iot->disconnect();
    SPDLOG_DEBUG("[iotb] stopping app: DONE");
}
const std::string& App::getLogDir() const
{
    return m_log_dir;
}

void App::onIpcReceive(size_t uuid, const std::shared_ptr<ipc::Packet>& packet, std::weak_ptr<ipc::Server> server)
{
    const auto action = packet->header["action"].get<std::string_view>();
    const auto topic = packet->header["topic"].get<std::string_view>();
    // TODO: add proper logging to ipc, so we dont need to duplicate messages
    if (action == "forward")
    {
        SPDLOG_DEBUG("[iotb] forwarding packet");
        m_client_iot->publish(topic, std::move(packet->payload));
    }
    else if (action == "subscribe")
    {
        SPDLOG_DEBUG("[iotb] subscribing to topic {}", topic);
        m_client_iot->subscribe(topic);
        // TODO: rewrite this in future
        m_subscriptions.emplace_back(uuid, topic);
    }
    else if (action == "unsubscribe")
    {
        SPDLOG_ERROR("[iotb] trying to unsubscribe. Unsupported feature. Topic {}, Session UUID = {}", topic, uuid);
        // TODO: no need to unsubscribe for now
    }
}

void App::onIotReceive(std::string_view topic, std::string_view data)
{
    SPDLOG_DEBUG("[iotb] received message from iot {}", topic);
    SPDLOG_TRACE("[iotb] iot message {} {}", topic, data);

    // TODO: optimize out creation of packet if no subbed session is found
    // TODO: consider creating packet view
    auto packet = std::make_shared<ipc::Packet>();
    packet->header["action"] = "forward";
    packet->payload = data;
    // find subbed sessions
    for (const auto& sub : m_subscriptions)
    {
        if (sub.second == topic)
        {
            SPDLOG_DEBUG("[iotb] forwarding to session uuid={}", sub.first);
            m_ipc_server->post(sub.first, packet);
        }
    }
}

void App::initDefaultLogger(std::string_view filepath,
                            size_t max_size,
                            size_t max_files,
                            std::chrono::seconds flush_interval)
{
#ifdef __ANDROID__
    auto console_logger = std::make_shared<spdlog::sinks::android_sink_mt>("iot_bridge");
#else
    auto console_logger = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
#endif
    console_logger->set_level(spdlog::level::trace);

    auto file_logger
        = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filepath.data(),
                                                                 max_size,
                                                                 max_files);
    file_logger->set_level(spdlog::level::trace);

    m_log_dir = fs::absolute(
        fs::path(
            file_logger->filename())
            .parent_path());

    auto new_logger = std::make_shared<spdlog::logger>("file_console",
                                                       spdlog::sinks_init_list{std::move(file_logger),
                                                                               std::move(console_logger)});

    spdlog::set_default_logger(std::move(new_logger));
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog::set_level(static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
    spdlog::flush_every(flush_interval);
}
}  // namespace iotb