#include "wz/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace wz {

Pipeline::~Pipeline() {
    requestStop();
    join();
}

bool Pipeline::start(const SystemConfig& cfg, std::string* error) {
    if (running_.load()) {
        if (error) *error = "конвейер уже запущен";
        return false;
    }
    if (!cfg.validate(error)) return false;

    cfg_ = cfg;
    processing_ = cfg.processing;
    processing_.clampToValidRange();

    sources_.clear();
    for (size_t i = 0; i < cfg_.sources.size(); ++i) {
        std::string err;
        auto src = createSource(cfg_.sources[i], static_cast<int>(i), &err);
        if (!src) {
            if (error) *error = "источник '" + cfg_.sources[i].id + "': " + err;
            sources_.clear();
            return false;
        }
        sources_.push_back(std::move(src));
    }

    if (!stitcher_.configure(cfg_.stitch, cfg_.cameras, error)) {
        sources_.clear();
        return false;
    }
    coverageVis_ = stitcher_.coverageMap();

    sync_ = std::make_unique<FrameSynchronizer>(sources_.size(), cfg_.sync);

    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_ = PipelineStats{};
        stats_.canvasWidth = cfg_.stitch.canvasWidth;
        stats_.canvasHeight = cfg_.stitch.canvasHeight;
        stats_.perSourceFrames.assign(sources_.size(), 0);
        stats_.stitch = stitcher_.stats();
    }

    stopRequested_.store(false);
    running_.store(true);

    for (size_t i = 0; i < sources_.size(); ++i) {
        sourceThreads_.emplace_back(&Pipeline::sourceLoop, this, i);
    }
    stitchThread_ = std::thread(&Pipeline::stitchLoop, this);
    return true;
}

void Pipeline::requestStop() {
    stopRequested_.store(true);
    broker_.stop();
    // Источник может стоять в блокирующем чтении с устройства - без этого
    // join() не дождался бы его никогда.
    for (auto& s : sources_) {
        if (s) s->cancel();
    }
}

void Pipeline::join() {
    for (std::thread& t : sourceThreads_) {
        if (t.joinable()) t.join();
    }
    sourceThreads_.clear();
    if (stitchThread_.joinable()) stitchThread_.join();

    for (auto& s : sources_) {
        if (s) s->close();
    }
    running_.store(false);
}

void Pipeline::sourceLoop(size_t index) {
    Frame frame;
    while (!stopRequested_.load()) {
        if (!sources_[index]->read(frame)) break;  // поток закончился

        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            if (index < stats_.perSourceFrames.size()) ++stats_.perSourceFrames[index];
        }
        sync_->push(std::move(frame));
        frame = Frame{};
    }
}

void Pipeline::stitchLoop() {
    std::vector<Frame> set;
    Image canvas;
    Image processed;
    std::vector<uint8_t> jpeg;

    int64_t windowStartUs = nowMicros();
    uint64_t windowFrames = 0;
    double stitchMsAccum = 0.0;
    double processMsAccum = 0.0;
    double encodeMsAccum = 0.0;
    uint64_t measured = 0;

    while (!stopRequested_.load()) {
        int64_t spread = 0;
        if (!sync_->tryPop(set, &spread)) {
            // Короткая пауза вместо активного ожидания: источники пишут
            // в своём темпе, крутить процессор смысла нет.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (keepRaw_.load()) {
            std::lock_guard<std::mutex> lock(rawMutex_);
            rawFrames_.clear();
            for (const Frame& f : set) rawFrames_.push_back(f.image);
        }

        const int64_t t0 = nowMicros();
        if (!stitcher_.blend(set, canvas)) continue;
        const int64_t t1 = nowMicros();

        const ProcessingConfig p = processing();
        applyProcessing(canvas, processed, p);

        const int64_t t2 = nowMicros();
        if (!encodeJpeg(processed, p.jpegQuality, jpeg)) continue;
        const int64_t t3 = nowMicros();

        broker_.publish(jpeg, set.front().timestampUs);

        stitchMsAccum += static_cast<double>(t1 - t0) / 1000.0;
        processMsAccum += static_cast<double>(t2 - t1) / 1000.0;
        encodeMsAccum += static_cast<double>(t3 - t2) / 1000.0;
        ++measured;
        ++windowFrames;

        const int64_t now = nowMicros();
        const int64_t elapsed = now - windowStartUs;

        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.framesStitched;
        stats_.syncSpreadUs = spread;
        stats_.sync = sync_->stats();
        stats_.stitch = stitcher_.stats();
        stats_.framesDropped = stats_.sync.droppedStale + stats_.sync.droppedOverflow;
        if (measured > 0) {
            stats_.stitchMs = stitchMsAccum / static_cast<double>(measured);
            stats_.processMs = processMsAccum / static_cast<double>(measured);
            stats_.encodeMs = encodeMsAccum / static_cast<double>(measured);
        }
        // Частоту считаем на скользящем окне в секунду: среднее за всё время
        // не показывает текущую просадку.
        if (elapsed > 1000000) {
            stats_.outputFps =
                static_cast<double>(windowFrames) * 1e6 / static_cast<double>(elapsed);
            windowStartUs = now;
            windowFrames = 0;
            stitchMsAccum = 0.0;
            processMsAccum = 0.0;
            encodeMsAccum = 0.0;
            measured = 0;
        }
    }
}

void Pipeline::applyProcessing(const Image& src, Image& dst, const ProcessingConfig& p) const {
    // Режим диагностики: вместо панорамы показываем, какие участки холста
    // видит одна камера, какие две, а какие не видит никто.
    const Image& base = (p.showCoverage && !coverageVis_.empty()) ? coverageVis_ : src;
    if (base.empty()) {
        dst = Image();
        return;
    }

    // Кадрирование в пикселях исходного холста.
    const int cropX = std::max(0, static_cast<int>(p.cropX * base.width()));
    const int cropY = std::max(0, static_cast<int>(p.cropY * base.height()));
    const int cropW =
        std::max(1, std::min(base.width() - cropX, static_cast<int>(p.cropW * base.width())));
    const int cropH =
        std::max(1, std::min(base.height() - cropY, static_cast<int>(p.cropH * base.height())));

    const int outW = p.outputWidth > 0 ? p.outputWidth : cropW;
    const int outH = p.outputHeight > 0 ? p.outputHeight : cropH;
    if (dst.width() != outW || dst.height() != outH) dst = Image(outW, outH);

    const float scaleX = static_cast<float>(cropW) / static_cast<float>(outW);
    const float scaleY = static_cast<float>(cropH) / static_cast<float>(outH);

    // Обычный режим - полный холст без масштабирования. Тогда билинейная
    // выборка сводится к копированию пикселя, и её можно обойти: на кадре
    // 1280x720 это экономит больше, чем вся цветокоррекция.
    const bool identityGeometry = cropX == 0 && cropY == 0 && outW == base.width() &&
                                  outH == base.height() && cropW == base.width() &&
                                  cropH == base.height();

    const float contrast = p.contrast;
    const float brightness = p.brightness;
    const float saturation = p.saturation;

    RGB c;
    for (int y = 0; y < outH; ++y) {
        const float sy = cropY + (static_cast<float>(y) + 0.5f) * scaleY - 0.5f;
        for (int x = 0; x < outW; ++x) {
            if (identityGeometry) {
                c = base.at(x, y);
            } else {
                const float sx = cropX + (static_cast<float>(x) + 0.5f) * scaleX - 0.5f;
                if (!base.sampleBilinear(sx, sy, c)) {
                    dst.set(x, y, RGB{0.f, 0.f, 0.f});
                    continue;
                }
            }

            // Контраст крутим вокруг середины диапазона, иначе картинка
            // одновременно темнеет и теряет детали в тенях.
            c.r = (c.r - 128.f) * contrast + 128.f + brightness;
            c.g = (c.g - 128.f) * contrast + 128.f + brightness;
            c.b = (c.b - 128.f) * contrast + 128.f + brightness;

            if (std::fabs(saturation - 1.f) > 1e-3f) {
                const float luma = 0.299f * c.r + 0.587f * c.g + 0.114f * c.b;
                c.r = luma + (c.r - luma) * saturation;
                c.g = luma + (c.g - luma) * saturation;
                c.b = luma + (c.b - luma) * saturation;
            }

            dst.set(x, y, c);
        }
    }

    // Подсветка зоны перекрытия: видно, где панорама держится на двух камерах.
    if (p.showSeams && !p.showCoverage && !coverageVis_.empty()) {
        for (int y = 0; y < outH; ++y) {
            const float sy = cropY + (static_cast<float>(y) + 0.5f) * scaleY - 0.5f;
            for (int x = 0; x < outW; ++x) {
                const float sx = cropX + (static_cast<float>(x) + 0.5f) * scaleX - 0.5f;
                RGB cov;
                if (!coverageVis_.sampleNearest(sx, sy, cov)) continue;
                // Зелёный в карте покрытия означает "видят две камеры".
                if (cov.g > 150.f && cov.r < 120.f) {
                    RGB cur = dst.at(x, y);
                    cur.g = std::min(255.f, cur.g + 45.f);
                    dst.set(x, y, cur);
                }
            }
        }
    }
}

void Pipeline::setKeepRawFrames(bool on) { keepRaw_.store(on); }

bool Pipeline::rawFrames(std::vector<Image>& out) const {
    std::lock_guard<std::mutex> lock(rawMutex_);
    if (rawFrames_.empty()) return false;
    out = rawFrames_;
    return true;
}

PipelineStats Pipeline::stats() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    PipelineStats copy = stats_;
    // Счётчики синхронизатора берутся напрямую: в stats_ они попадают только
    // при удачно собранном наборе, то есть как раз тогда, когда всё хорошо.
    // При разборе неисправности нужна обратная картина.
    if (sync_) copy.sync = sync_->stats();
    return copy;
}

ProcessingConfig Pipeline::processing() const {
    std::lock_guard<std::mutex> lock(processingMutex_);
    return processing_;
}

void Pipeline::setProcessing(const ProcessingConfig& p) {
    ProcessingConfig copy = p;
    copy.clampToValidRange();
    std::lock_guard<std::mutex> lock(processingMutex_);
    processing_ = copy;
}

}  // namespace wz
