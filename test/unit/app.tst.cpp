/*
 * MENTION: this file is only one for testing
 * but the similar file is not tested in other repositories
 */

#include <thread>

#include "iotb/app.hpp"
#include "iotb/fs.hpp"
#include "iotb/interface/client_iot.hpp"
#include "ipc/util.hpp"
#include "nlohmann/json.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#ifdef __ANDROID__
#include "spdlog/sinks/android_sink.h"
#else
#include "spdlog/sinks/stdout_color_sinks.h"
#endif

#include <fstream>
#include <functional>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace std::chrono_literals;
using namespace ipc::net;

/* Create dummy IoT Client */
class AppIot : public ::iotb::IClientIot
{
protected:
    void connect() override
    {
    }
    void disconnect() override
    {
    }
    void subscribe(std::string_view topic) override
    {
    }
    void unsubscribe(std::string_view topic) override
    {
    }
    void publish(std::string_view topic, std::string_view payload) override
    {
    }
    void setOnReceivedHandler(const on_received_handler& handler) override
    {
    }
    void setConfig(const nlohmann::json& config) override
    {
    }
};

/* Test class with all necessary objects */
class AppTest : public ::testing::Test
{
protected:
    std::shared_ptr<AppIot> m_appIot;
    std::shared_ptr<ipc::Server> m_ipc_server;
    std::shared_ptr<ipc::Packet> m_packet;
};

TEST_F(AppTest, Test)
{
    m_appIot = std::make_shared<AppIot>();
    std::ifstream config_file("/var/configs/main_config.json");

    ::iotb::App app(m_appIot,
                    nlohmann::json::parse(config_file));

    app.initDefaultLogger("logs/logs.txt", 1'000'000, 5, 5s);
    std::string log_dir = app.getLogDir();

    app.init();

    /* 
     * The code below should be tested by integration 
     * start() and stop() functions use 
     * the inner io_context and tcp acceptor
     */

    /* 
    app.start();
  
    std::string topic = "topicName";
    std::string data =
        "{"
        "attribute"
        " : "
        "value"
        "}";
    app.onIotReceive(topic, data);

    size_t uuid = 1;
    m_packet = std::make_shared<ipc::Packet>();
    m_packet->header["action"] = "forward";
    m_packet->header["topic"] = "/data_collection/send_data/temp";
    m_packet->payload =
        "{"
        "temp"
        ":48}"; 
    app.onIpcReceive(uuid, m_packet, m_ipc_server);

    m_packet->header["action"] = "subscribe";
    app.onIpcReceive(uuid, m_packet, m_ipc_server);

    m_packet->header["action"] = "unsubscribe";
    app.onIpcReceive(uuid, m_packet, m_ipc_server);
    */

    /*
    * There is a problem with a stop() function that called in the App destructor
    * That is the issue, because there is no io_context created. 
    */
}