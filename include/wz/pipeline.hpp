// Конвейер: источники -> синхронизация -> склейка -> кодирование -> сервер.
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "wz/config.hpp"
#include "wz/http_server.hpp"
#include "wz/source.hpp"
#include "wz/stitcher.hpp"
#include "wz/sync.hpp"

namespace wz {

struct PipelineStats {
    uint64_t framesStitched = 0;
    uint64_t framesDropped = 0;
    double outputFps = 0.0;
    double stitchMs = 0.0;   // среднее время склейки кадра
    double processMs = 0.0;  // среднее время постобработки (яркость, кадрирование)
    double encodeMs = 0.0;   // среднее время кодирования JPEG
    int64_t syncSpreadUs = 0;
    SyncStats sync;
    StitchStats stitch;
    int canvasWidth = 0;
    int canvasHeight = 0;
    std::vector<uint64_t> perSourceFrames;
};

class Pipeline {
public:
    ~Pipeline();

    bool start(const SystemConfig& cfg, std::string* error);
    void requestStop();
    void join();
    bool running() const { return running_.load(); }

    FrameBroker& broker() { return broker_; }
    PipelineStats stats() const;

    ProcessingConfig processing() const;
    void setProcessing(const ProcessingConfig& p);

    // Сохранение исходных кадров сведённого набора. Копирование включается
    // явно: в обычном режиме оно только жгло бы память и такты.
    void setKeepRawFrames(bool on);
    bool rawFrames(std::vector<Image>& out) const;

    const SystemConfig& config() const { return cfg_; }

private:
    void sourceLoop(size_t index);
    void stitchLoop();
    // Яркость/контраст/насыщенность/кадрирование поверх собранной панорамы.
    void applyProcessing(const Image& src, Image& dst, const ProcessingConfig& p) const;

    SystemConfig cfg_;
    std::vector<std::unique_ptr<ISource>> sources_;
    std::unique_ptr<FrameSynchronizer> sync_;
    Stitcher stitcher_;
    FrameBroker broker_;
    // Карта покрытия считается один раз при запуске: она зависит только
    // от геометрии, а рисовать её на каждом кадре слишком дорого.
    Image coverageVis_;

    std::vector<std::thread> sourceThreads_;
    std::thread stitchThread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopRequested_{false};

    mutable std::mutex statsMutex_;
    PipelineStats stats_;

    mutable std::mutex processingMutex_;
    ProcessingConfig processing_;

    std::atomic<bool> keepRaw_{false};
    mutable std::mutex rawMutex_;
    std::vector<Image> rawFrames_;
};

}  // namespace wz
