#include "wz/sync.hpp"

#include <algorithm>
#include <limits>

namespace wz {

FrameSynchronizer::FrameSynchronizer(size_t sourceCount, const SyncConfig& cfg)
    : buffers_(sourceCount), cfg_(cfg) {
    if (cfg_.bufferDepth < 2) cfg_.bufferDepth = 2;
    if (cfg_.toleranceUs < 0) cfg_.toleranceUs = 0;
}

void FrameSynchronizer::push(Frame frame) {
    if (frame.sourceIndex < 0 ||
        static_cast<size_t>(frame.sourceIndex) >= buffers_.size()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto& buf = buffers_[static_cast<size_t>(frame.sourceIndex)];
    buf.push_back(std::move(frame));
    ++stats_.pushed;

    // Переполнение означает, что потребитель не успевает. Выбрасываем самые
    // старые кадры: свежая картинка важнее полноты истории.
    while (buf.size() > cfg_.bufferDepth) {
        buf.pop_front();
        ++stats_.droppedOverflow;
    }
}

bool FrameSynchronizer::tryPop(std::vector<Frame>& out, int64_t* spreadUs) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffers_.empty()) return false;
    for (const auto& b : buffers_) {
        if (b.empty()) return false;
    }

    // Якорь - самая поздняя из "самых старых" меток. Кадры остальных
    // источников, что старше якоря на величину допуска, уже не догонят пару.
    int64_t anchor = std::numeric_limits<int64_t>::min();
    for (const auto& b : buffers_) anchor = std::max(anchor, b.front().timestampUs);

    for (auto& b : buffers_) {
        while (b.size() > 1 && b.front().timestampUs < anchor - cfg_.toleranceUs) {
            b.pop_front();
            ++stats_.droppedStale;
        }
    }

    int64_t lo = std::numeric_limits<int64_t>::max();
    int64_t hi = std::numeric_limits<int64_t>::min();
    for (const auto& b : buffers_) {
        const int64_t t = b.front().timestampUs;
        lo = std::min(lo, t);
        hi = std::max(hi, t);
    }

    // Набор ещё не сложился - ждём, пока отставший источник догонит.
    if (hi - lo > cfg_.toleranceUs) return false;

    out.clear();
    out.reserve(buffers_.size());
    for (auto& b : buffers_) {
        out.push_back(std::move(b.front()));
        b.pop_front();
    }

    const int64_t spread = hi - lo;
    ++stats_.matched;
    stats_.lastSpreadUs = spread;
    spreadAccum_ += static_cast<double>(spread);
    stats_.meanSpreadUs = spreadAccum_ / static_cast<double>(stats_.matched);
    if (spreadUs) *spreadUs = spread;
    return true;
}

SyncStats FrameSynchronizer::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

}  // namespace wz
