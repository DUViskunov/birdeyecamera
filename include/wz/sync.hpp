// Временная синхронизация нескольких видеопотоков.
#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "wz/frame.hpp"

namespace wz {

struct SyncConfig {
    // Максимальный разброс меток времени внутри набора. 20 мс - примерно
    // половина межкадрового интервала при 30 кадр/с.
    int64_t toleranceUs = 20000;
    // Глубина буфера на источник. Больше - устойчивее к рывкам, но выше задержка.
    size_t bufferDepth = 8;
};

struct SyncStats {
    uint64_t pushed = 0;
    uint64_t matched = 0;
    uint64_t droppedStale = 0;   // кадры, для которых не нашлось пары
    uint64_t droppedOverflow = 0;  // вытеснены из переполненного буфера
    int64_t lastSpreadUs = 0;
    double meanSpreadUs = 0.0;
};

// Собирает из независимых потоков наборы кадров, снятых примерно одновременно.
// Потокобезопасен: источники пишут из своих потоков, конвейер читает из своего.
class FrameSynchronizer {
public:
    FrameSynchronizer(size_t sourceCount, const SyncConfig& cfg);

    void push(Frame frame);

    // Забирает согласованный набор (по кадру с каждого источника).
    // Возвращает false, если пока не хватает данных.
    bool tryPop(std::vector<Frame>& out, int64_t* spreadUs = nullptr);

    SyncStats stats() const;
    size_t sourceCount() const { return buffers_.size(); }

private:
    mutable std::mutex mutex_;
    std::vector<std::deque<Frame>> buffers_;
    SyncConfig cfg_;
    SyncStats stats_;
    double spreadAccum_ = 0.0;
};

}  // namespace wz
