#include "wz/http_server.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#define WZ_INVALID_SOCKET INVALID_SOCKET
#define wz_close closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
#define WZ_INVALID_SOCKET (-1)
#define wz_close ::close
#endif

namespace wz {

void FrameBroker::publish(std::vector<uint8_t> jpeg, int64_t timestampUs) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        jpeg_ = std::move(jpeg);
        timestampUs_ = timestampUs;
        ++sequence_;
    }
    cv_.notify_all();
}

bool FrameBroker::waitForNext(uint64_t& lastSeen, std::vector<uint8_t>& out, int timeoutMs) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ok = cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                 [&] { return stopped_.load() || sequence_ > lastSeen; });
    if (!ok || stopped_.load() || jpeg_.empty()) return false;

    lastSeen = sequence_;
    out = jpeg_;
    return true;
}

bool FrameBroker::latest(std::vector<uint8_t>& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (jpeg_.empty()) return false;
    out = jpeg_;
    return true;
}

uint64_t FrameBroker::sequence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sequence_;
}

void FrameBroker::stop() {
    stopped_.store(true);
    cv_.notify_all();
}

std::string HttpRequest::param(const std::string& key, const std::string& fallback) const {
    auto it = query.find(key);
    return it == query.end() ? fallback : it->second;
}

namespace {

#ifdef _WIN32
// Winsock требует явной инициализации ровно один раз на процесс.
struct WinsockGuard {
    WinsockGuard() {
        WSADATA data;
        ok = WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }
    ~WinsockGuard() {
        if (ok) WSACleanup();
    }
    bool ok = false;
};
#endif

bool sendAll(socket_t s, const char* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        const int n = ::send(s, data + sent, static_cast<int>(len - sent), 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool sendAll(socket_t s, const std::string& text) {
    return sendAll(s, text.data(), text.size());
}

std::string urlDecode(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '+') {
            out += ' ';
        } else if (in[i] == '%' && i + 2 < in.size()) {
            const std::string hex = in.substr(i + 1, 2);
            out += static_cast<char>(std::strtol(hex.c_str(), nullptr, 16));
            i += 2;
        } else {
            out += in[i];
        }
    }
    return out;
}

const char* mimeFor(const std::string& path) {
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return "application/octet-stream";
    const std::string ext = path.substr(dot + 1);
    if (ext == "html" || ext == "htm") return "text/html; charset=utf-8";
    if (ext == "css") return "text/css; charset=utf-8";
    if (ext == "js") return "application/javascript; charset=utf-8";
    if (ext == "json") return "application/json; charset=utf-8";
    if (ext == "png") return "image/png";
    if (ext == "jpg" || ext == "jpeg") return "image/jpeg";
    if (ext == "svg") return "image/svg+xml";
    return "application/octet-stream";
}

std::string httpResponse(const std::string& status, const std::string& contentType,
                         const std::string& body) {
    std::ostringstream os;
    os << "HTTP/1.1 " << status << "\r\n"
       << "Content-Type: " << contentType << "\r\n"
       << "Content-Length: " << body.size() << "\r\n"
       << "Cache-Control: no-store\r\n"
       << "Connection: close\r\n\r\n"
       << body;
    return os.str();
}

// Защита от выхода за пределы webRoot: запрос вида /../../secret.txt
// не должен читать ничего за каталогом веб-интерфейса.
bool isSafeRelativePath(const std::string& path) {
    if (path.find("..") != std::string::npos) return false;
    if (path.find('\\') != std::string::npos) return false;
    if (path.find(':') != std::string::npos) return false;
    if (!path.empty() && path[0] == '/') return false;
    return true;
}

}  // namespace

HttpServer::~HttpServer() { stop(); }

bool HttpServer::start(const ServerConfig& cfg, FrameBroker* broker, std::string* error) {
    if (running_.load()) {
        if (error) *error = "сервер уже запущен";
        return false;
    }

    cfg_ = cfg;
    broker_ = broker;
    if (cfg_.maxClients < 1) cfg_.maxClients = 1;

#ifdef _WIN32
    static WinsockGuard guard;
    if (!guard.ok) {
        if (error) *error = "не удалось инициализировать Winsock";
        return false;
    }
#endif

    socket_t listenSock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock == WZ_INVALID_SOCKET) {
        if (error) *error = "не удалось создать сокет";
        return false;
    }

    int yes = 1;
    ::setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes),
                 sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(cfg_.port));
    if (::inet_pton(AF_INET, cfg_.bindAddress.c_str(), &addr.sin_addr) != 1) {
        wz_close(listenSock);
        if (error) *error = "некорректный адрес привязки: " + cfg_.bindAddress;
        return false;
    }

    if (::bind(listenSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        wz_close(listenSock);
        if (error) {
            *error = "не удалось занять " + cfg_.bindAddress + ":" + std::to_string(cfg_.port) +
                     " (порт занят другим процессом?)";
        }
        return false;
    }

    if (::listen(listenSock, 16) != 0) {
        wz_close(listenSock);
        if (error) *error = "listen() завершился ошибкой";
        return false;
    }

    listenSocket_ = static_cast<uintptr_t>(listenSock);
    running_.store(true);
    acceptThread_ = std::thread(&HttpServer::acceptLoop, this);
    return true;
}

void HttpServer::stop() {
    if (!running_.exchange(false)) return;

    if (listenSocket_ != 0) {
        wz_close(static_cast<socket_t>(listenSocket_));
        listenSocket_ = 0;
    }
    if (acceptThread_.joinable()) acceptThread_.join();

    // Обработчики отсоединены, поэтому ждём, пока они сами заметят остановку.
    for (int i = 0; i < 200 && activeClients_.load() > 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void HttpServer::acceptLoop() {
    while (running_.load()) {
        const socket_t client = ::accept(static_cast<socket_t>(listenSocket_), nullptr, nullptr);
        if (client == WZ_INVALID_SOCKET) {
            if (!running_.load()) break;
            continue;
        }

        if (activeClients_.load() >= cfg_.maxClients) {
            sendAll(client, httpResponse("503 Service Unavailable",
                                         "application/json; charset=utf-8",
                                         "{\"error\":\"слишком много подключений\"}"));
            wz_close(client);
            continue;
        }

        ++activeClients_;
        std::thread(&HttpServer::handleClient, this, static_cast<uintptr_t>(client)).detach();
    }
}

void HttpServer::handleClient(uintptr_t socketHandle) {
    const socket_t s = static_cast<socket_t>(socketHandle);

    std::string raw;
    char buf[4096];
    // Читаем до конца заголовков: тела у наших запросов нет.
    while (raw.find("\r\n\r\n") == std::string::npos && raw.size() < 64 * 1024) {
        const int n = ::recv(s, buf, sizeof(buf), 0);
        if (n <= 0) {
            wz_close(s);
            --activeClients_;
            return;
        }
        raw.append(buf, static_cast<size_t>(n));
    }

    HttpRequest req;
    {
        std::istringstream head(raw);
        std::string target, version;
        head >> req.method >> target >> version;

        const size_t q = target.find('?');
        if (q == std::string::npos) {
            req.path = urlDecode(target);
        } else {
            req.path = urlDecode(target.substr(0, q));
            const std::string qs = target.substr(q + 1);
            size_t start = 0;
            while (start <= qs.size()) {
                const size_t amp = qs.find('&', start);
                const std::string pair =
                    qs.substr(start, amp == std::string::npos ? std::string::npos : amp - start);
                const size_t eq = pair.find('=');
                if (eq != std::string::npos) {
                    req.query[urlDecode(pair.substr(0, eq))] = urlDecode(pair.substr(eq + 1));
                }
                if (amp == std::string::npos) break;
                start = amp + 1;
            }
        }
    }

    ++served_;

    if (req.path == "/stream.mjpg") {
        sendStream(socketHandle);
        wz_close(s);
        --activeClients_;
        return;
    }

    if (req.path == "/snapshot.jpg") {
        std::vector<uint8_t> jpeg;
        if (broker_ && broker_->latest(jpeg)) {
            std::ostringstream os;
            os << "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: " << jpeg.size()
               << "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
            sendAll(s, os.str());
            sendAll(s, reinterpret_cast<const char*>(jpeg.data()), jpeg.size());
        } else {
            sendAll(s, httpResponse("503 Service Unavailable", "application/json; charset=utf-8",
                                    "{\"error\":\"кадр ещё не готов\"}"));
        }
        wz_close(s);
        --activeClients_;
        return;
    }

    if (req.path.rfind("/api/", 0) == 0 && api_) {
        std::string contentType = "application/json; charset=utf-8";
        const std::string body = api_(req, contentType);
        if (body.empty()) {
            sendAll(s, httpResponse("404 Not Found", "application/json; charset=utf-8",
                                    "{\"error\":\"неизвестный метод\"}"));
        } else {
            sendAll(s, httpResponse("200 OK", contentType, body));
        }
        wz_close(s);
        --activeClients_;
        return;
    }

    // Статика веб-интерфейса.
    const std::string rel = req.path == "/" ? "index.html" : req.path.substr(1);
    if (!isSafeRelativePath(rel)) {
        sendAll(s, httpResponse("403 Forbidden", "text/plain; charset=utf-8", "Доступ запрещён"));
        wz_close(s);
        --activeClients_;
        return;
    }

    std::ifstream file(cfg_.webRoot + "/" + rel, std::ios::binary);
    if (!file) {
        sendAll(s, httpResponse("404 Not Found", "text/plain; charset=utf-8",
                                "Файл не найден: " + rel));
    } else {
        std::ostringstream ss;
        ss << file.rdbuf();
        sendAll(s, httpResponse("200 OK", mimeFor(rel), ss.str()));
    }

    wz_close(s);
    --activeClients_;
}

bool HttpServer::sendStream(uintptr_t socketHandle) {
    const socket_t s = static_cast<socket_t>(socketHandle);
    if (!broker_) return false;

    ++streams_;
    const std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=wzframe\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n";
    if (!sendAll(s, header)) {
        --streams_;
        return false;
    }

    uint64_t lastSeen = 0;
    std::vector<uint8_t> jpeg;

    while (running_.load() && !broker_->stopped()) {
        if (!broker_->waitForNext(lastSeen, jpeg, 1000)) {
            if (broker_->stopped()) break;
            continue;  // таймаут - просто ждём следующий кадр
        }

        std::ostringstream part;
        part << "--wzframe\r\nContent-Type: image/jpeg\r\nContent-Length: " << jpeg.size()
             << "\r\n\r\n";
        if (!sendAll(s, part.str())) break;
        if (!sendAll(s, reinterpret_cast<const char*>(jpeg.data()), jpeg.size())) break;
        if (!sendAll(s, "\r\n")) break;
    }

    --streams_;
    return true;
}

}  // namespace wz
