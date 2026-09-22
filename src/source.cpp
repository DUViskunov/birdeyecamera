#include "wz/source.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <thread>

#include "wz/synthetic.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#endif

namespace wz {

int64_t nowMicros() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

namespace {

// Общая часть всех источников: выдержка темпа и нумерация кадров.
// Метки времени синтетические (индекс кадра / fps), поэтому поведение
// воспроизводимо, а задержка запуска не смазывает синхронизацию.
class SourceBase : public ISource {
public:
    SourceBase(const SourceConfig& cfg, int index) : cfg_(cfg), index_(index) {
        if (cfg_.fps < 0.1) cfg_.fps = 0.1;
    }

    const SourceConfig& config() const override { return cfg_; }
    uint64_t framesRead() const override { return frames_; }
    void close() override {}

protected:
    int64_t periodUs() const { return static_cast<int64_t>(1e6 / cfg_.fps); }

    // Ждёт момента, когда кадр с номером frames_ должен быть снят.
    void pace() {
        if (startWallUs_ == 0) {
            startWallUs_ = nowMicros();
            return;
        }
        const int64_t due = startWallUs_ + static_cast<int64_t>(frames_) * periodUs();
        const int64_t delta = due - nowMicros();
        if (delta > 0) std::this_thread::sleep_for(std::chrono::microseconds(delta));
    }

    void stamp(Frame& f) {
        f.sourceIndex = index_;
        f.sequence = frames_;
        f.timestampUs = cfg_.startOffsetUs + static_cast<int64_t>(frames_) * periodUs();
        ++frames_;
    }

    SourceConfig cfg_;
    int index_ = -1;
    uint64_t frames_ = 0;
    int64_t startWallUs_ = 0;
};

// Виртуальная сцена: работает всегда и ни от чего не зависит.
class SyntheticSource : public SourceBase {
public:
    using SourceBase::SourceBase;

    bool open(std::string* error) override {
        if (cfg_.width <= 0 || cfg_.height <= 0) {
            if (error) *error = "недопустимый размер кадра для синтетического источника";
            return false;
        }
        view_ = syntheticView(cfg_.syntheticView, cfg_.width, cfg_.height);
        return true;
    }

    bool read(Frame& out) override {
        pace();
        const double t = static_cast<double>(frames_) / cfg_.fps;
        renderSyntheticFrame(view_, t, out.image);
        stamp(out);
        return true;
    }

private:
    SyntheticView view_;
};

std::vector<std::string> listDirectorySorted(const std::string& dir) {
    std::vector<std::string> files;
#ifdef _WIN32
    WIN32_FIND_DATAA fd;
    const std::string pattern = dir + "\\*";
    HANDLE h = FindFirstFileA(pattern.c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return files;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) files.push_back(fd.cFileName);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
#else
    DIR* d = opendir(dir.c_str());
    if (!d) return files;
    while (dirent* e = readdir(d)) {
        if (e->d_name[0] != '.') files.push_back(e->d_name);
    }
    closedir(d);
#endif
    std::sort(files.begin(), files.end());
    return files;
}

bool hasImageExtension(const std::string& name) {
    const size_t dot = name.find_last_of('.');
    if (dot == std::string::npos) return false;
    std::string ext = name.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp";
}

// Каталог с кадрами: основной способ скормить системе записанное видео.
// Файлы сортируются по имени, поэтому нумерация должна быть с ведущими нулями.
class JpegDirSource : public SourceBase {
public:
    using SourceBase::SourceBase;

    bool open(std::string* error) override {
        for (const std::string& name : listDirectorySorted(cfg_.path)) {
            if (hasImageExtension(name)) files_.push_back(cfg_.path + "/" + name);
        }
        if (files_.empty()) {
            if (error) *error = "в каталоге " + cfg_.path + " нет кадров (jpg/png/bmp)";
            return false;
        }
        return true;
    }

    bool read(Frame& out) override {
        // Битый кадр пропускаем, но не крутимся вечно: если целых кадров
        // не осталось, честно сообщаем о конце потока.
        for (size_t attempt = 0; attempt < files_.size(); ++attempt) {
            if (cursor_ >= files_.size()) {
                if (!cfg_.loop) return false;
                cursor_ = 0;
            }
            pace();

            const std::string path = files_[cursor_++];
            std::string err;
            if (loadImageFile(path, out.image, &err)) {
                stamp(out);
                return true;
            }
        }
        return false;
    }

private:
    std::vector<std::string> files_;
    size_t cursor_ = 0;
};

// Файл с непрерывным потоком JPEG - так пишет MJPEG-камера и так же
// отдаёт наш собственный сервер. Кадры ищутся по маркерам SOI/EOI.
class MjpegFileSource : public SourceBase {
public:
    using SourceBase::SourceBase;

    bool open(std::string* error) override {
        std::ifstream in(cfg_.path, std::ios::binary);
        if (!in) {
            if (error) *error = "не удалось открыть " + cfg_.path;
            return false;
        }
        data_.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (!indexFrames()) {
            if (error) *error = "в " + cfg_.path + " не найдено ни одного JPEG-кадра";
            return false;
        }
        return true;
    }

    bool read(Frame& out) override {
        for (size_t attempt = 0; attempt < offsets_.size(); ++attempt) {
            if (cursor_ >= offsets_.size()) {
                if (!cfg_.loop) return false;
                cursor_ = 0;
            }
            pace();

            const auto span = offsets_[cursor_++];
            std::string err;
            if (loadImageFromMemory(reinterpret_cast<const uint8_t*>(data_.data()) + span.first,
                                    span.second, out.image, &err)) {
                stamp(out);
                return true;
            }
        }
        return false;
    }

private:
    bool indexFrames() {
        offsets_.clear();
        const uint8_t* p = reinterpret_cast<const uint8_t*>(data_.data());
        const size_t n = data_.size();

        size_t i = 0;
        while (i + 1 < n) {
            if (p[i] == 0xFF && p[i + 1] == 0xD8) {  // SOI - начало изображения
                size_t j = i + 2;
                while (j + 1 < n && !(p[j] == 0xFF && p[j + 1] == 0xD9)) ++j;
                if (j + 1 >= n) break;  // хвост обрезан - дальше кадров нет
                offsets_.emplace_back(i, j + 2 - i);
                i = j + 2;
            } else {
                ++i;
            }
        }
        return !offsets_.empty();
    }

    std::string data_;
    std::vector<std::pair<size_t, size_t>> offsets_;
    size_t cursor_ = 0;
};

}  // namespace

std::vector<std::string> listImageFiles(const std::string& dir) {
    std::vector<std::string> out;
    for (const std::string& name : listDirectorySorted(dir)) {
        if (hasImageExtension(name)) out.push_back(dir + "/" + name);
    }
    return out;
}

std::vector<std::string> availableSourceTypes() {
    return {"synthetic", "jpegdir", "mjpeg", "camera"};
}

std::unique_ptr<ISource> createSource(const SourceConfig& cfg, int sourceIndex,
                                      std::string* error) {
    std::unique_ptr<ISource> source;

    if (cfg.type == "synthetic") {
        source = std::make_unique<SyntheticSource>(cfg, sourceIndex);
    } else if (cfg.type == "jpegdir") {
        source = std::make_unique<JpegDirSource>(cfg, sourceIndex);
    } else if (cfg.type == "mjpeg") {
        source = std::make_unique<MjpegFileSource>(cfg, sourceIndex);
    } else if (cfg.type == "camera") {
        // Захват с устройства живёт в отдельном файле: он платформенный.
        return createCameraSource(cfg, sourceIndex, error);
    } else {
        if (error) *error = "неизвестный тип источника: " + cfg.type;
        return nullptr;
    }

    if (!source->open(error)) return nullptr;
    return source;
}

}  // namespace wz
