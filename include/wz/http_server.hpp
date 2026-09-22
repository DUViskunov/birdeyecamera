// HTTP-сервер: MJPEG-поток, снимки и управляющий API.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "wz/config.hpp"

namespace wz {

// Хранит последний готовый JPEG. Клиенты ждут появления нового кадра,
// а не опрашивают буфер в цикле.
class FrameBroker {
public:
    void publish(std::vector<uint8_t> jpeg, int64_t timestampUs);

    // Ждёт кадр с номером больше lastSeen. Обновляет lastSeen.
    // false - истёк таймаут или брокер остановлен.
    bool waitForNext(uint64_t& lastSeen, std::vector<uint8_t>& out, int timeoutMs);

    bool latest(std::vector<uint8_t>& out) const;
    uint64_t sequence() const;
    void stop();
    bool stopped() const { return stopped_.load(); }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<uint8_t> jpeg_;
    uint64_t sequence_ = 0;
    int64_t timestampUs_ = 0;
    std::atomic<bool> stopped_{false};
};

struct HttpRequest {
    std::string method;
    std::string path;
    std::map<std::string, std::string> query;
    std::string body;

    std::string param(const std::string& key, const std::string& fallback = std::string()) const;
};

// Обработчик возвращает тело ответа; пустая строка - 404.
using ApiHandler = std::function<std::string(const HttpRequest&, std::string& contentType)>;

class HttpServer {
public:
    ~HttpServer();

    bool start(const ServerConfig& cfg, FrameBroker* broker, std::string* error);
    void stop();
    bool running() const { return running_.load(); }
    int port() const { return cfg_.port; }
    uint64_t servedRequests() const { return served_.load(); }
    int activeStreams() const { return streams_.load(); }

    void setApiHandler(ApiHandler handler) { api_ = std::move(handler); }

private:
    void acceptLoop();
    void handleClient(uintptr_t socketHandle);
    bool sendStream(uintptr_t socketHandle);

    ServerConfig cfg_;
    FrameBroker* broker_ = nullptr;
    ApiHandler api_;

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> served_{0};
    std::atomic<int> streams_{0};
    // Обработчики клиентов отсоединяются, а их число отслеживается счётчиком:
    // завершившийся std::thread остаётся joinable, и список рос бы без предела.
    std::atomic<int> activeClients_{0};
    uintptr_t listenSocket_ = 0;
    std::thread acceptThread_;
};

}  // namespace wz
