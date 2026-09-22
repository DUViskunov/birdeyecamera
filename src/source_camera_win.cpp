// Захват с реальных камер. Платформенный код вынесен сюда, чтобы Media
// Foundation не протекала в остальной конвейер.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

#include "wz/source.hpp"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>

namespace wz {
namespace {

template <class T>
void safeRelease(T*& p) {
    if (p) {
        p->Release();
        p = nullptr;
    }
}

std::string toUtf8(const wchar_t* w) {
    if (!w) return std::string();
    const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return std::string();
    std::string s(static_cast<size_t>(n - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
    return s;
}

// Media Foundation инициализируется один раз на процесс и не выключается:
// источники живут до конца работы, а MFShutdown из чужого потока опасен.
bool ensureMediaFoundation(std::string* error) {
    static std::once_flag once;
    static bool ok = false;

    std::call_once(once, [] {
        // MTA: читатель создаётся в одном потоке, а опрашивается в другом.
        const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        if (co != S_OK && co != S_FALSE && co != RPC_E_CHANGED_MODE) return;
        ok = SUCCEEDED(MFStartup(MF_VERSION, MFSTARTUP_LITE));
    });

    if (!ok && error) *error = "не удалось инициализировать Media Foundation";
    return ok;
}

struct DeviceList {
    IMFActivate** items = nullptr;
    UINT32 count = 0;

    ~DeviceList() {
        for (UINT32 i = 0; i < count; ++i) safeRelease(items[i]);
        if (items) CoTaskMemFree(items);
    }
};

bool enumerateDevices(DeviceList& out, std::string* error) {
    IMFAttributes* attrs = nullptr;
    if (FAILED(MFCreateAttributes(&attrs, 1))) {
        if (error) *error = "MFCreateAttributes не выполнилась";
        return false;
    }
    attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                   MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);

    const HRESULT hr = MFEnumDeviceSources(attrs, &out.items, &out.count);
    safeRelease(attrs);

    if (FAILED(hr)) {
        if (error) *error = "не удалось перечислить устройства захвата";
        return false;
    }
    return true;
}

std::string attributeString(IMFActivate* dev, const GUID& key) {
    WCHAR* value = nullptr;
    UINT32 length = 0;
    if (FAILED(dev->GetAllocatedString(key, &value, &length))) return std::string();
    std::string result = toUtf8(value);
    CoTaskMemFree(value);
    return result;
}

// Краткая сводка режимов: только MJPG и только 15 кадр/с и выше -
// остальное в подсказке оператору лишнее.
std::string describeModes(IMFActivate* dev) {
    IMFMediaSource* source = nullptr;
    if (FAILED(dev->ActivateObject(IID_PPV_ARGS(&source)))) return std::string();

    IMFSourceReader* reader = nullptr;
    std::string modes;
    if (SUCCEEDED(MFCreateSourceReaderFromMediaSource(source, nullptr, &reader))) {
        std::vector<std::string> seen;
        for (DWORD i = 0; i < 256; ++i) {
            IMFMediaType* type = nullptr;
            if (FAILED(reader->GetNativeMediaType(
                    static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), i, &type))) {
                break;
            }
            GUID subtype = GUID_NULL;
            UINT32 w = 0, h = 0, num = 0, den = 1;
            type->GetGUID(MF_MT_SUBTYPE, &subtype);
            MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
            MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &num, &den);
            safeRelease(type);

            if (subtype != MFVideoFormat_MJPG || den == 0) continue;
            const double fps = static_cast<double>(num) / static_cast<double>(den);
            if (fps < 14.9) continue;

            char buf[64];
            std::snprintf(buf, sizeof(buf), "%ux%u@%.0f", w, h, fps);
            if (std::find(seen.begin(), seen.end(), std::string(buf)) == seen.end()) {
                seen.push_back(buf);
            }
        }
        for (size_t i = 0; i < seen.size(); ++i) modes += (i ? " " : "") + seen[i];
        safeRelease(reader);
    }

    source->Shutdown();
    safeRelease(source);
    return modes;
}

class MediaFoundationCamera : public ISource {
public:
    MediaFoundationCamera(const SourceConfig& cfg, int index) : cfg_(cfg), index_(index) {
        if (cfg_.fps < 0.1) cfg_.fps = 30.0;
    }

    ~MediaFoundationCamera() override { close(); }

    const SourceConfig& config() const override { return cfg_; }
    uint64_t framesRead() const override { return frames_; }

    bool open(std::string* error) override {
        if (!ensureMediaFoundation(error)) return false;

        DeviceList devices;
        if (!enumerateDevices(devices, error)) return false;
        if (devices.count == 0) {
            if (error) *error = "в системе не найдено ни одной камеры";
            return false;
        }

        const int picked = resolveDevice(devices, error);
        if (picked < 0) return false;

        IMFMediaSource* source = nullptr;
        if (FAILED(devices.items[picked]->ActivateObject(IID_PPV_ARGS(&source)))) {
            if (error) {
                *error = "не удалось открыть камеру " + std::to_string(picked) +
                         " (занята другим приложением?)";
            }
            return false;
        }

        // Расширенная обработка нужна для запасного пути RGB32: без неё
        // читатель откажется отдавать формат, которого нет у устройства.
        IMFAttributes* attrs = nullptr;
        MFCreateAttributes(&attrs, 1);
        attrs->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE);

        const HRESULT hr = MFCreateSourceReaderFromMediaSource(source, attrs, &reader_);
        safeRelease(attrs);
        // Только Release: читатель продолжает работать с этим источником, и
        // Shutdown() здесь остановил бы его - кадры перестали бы приходить.
        safeRelease(source);

        if (FAILED(hr)) {
            if (error) *error = "не удалось создать читатель потока";
            return false;
        }

        if (!selectFormat(error)) {
            safeRelease(reader_);
            return false;
        }
        return true;
    }

    bool read(Frame& out) override {
        if (!reader_) return false;

        // open() выполняется в главном потоке, а read() - в потоке источника.
        // Каждый поток обязан войти в апартамент COM сам, иначе вызовы
        // читателя не работают: устройство открывается, но кадры не идут.
        static thread_local bool comReady = false;
        if (!comReady) {
            CoInitializeEx(nullptr, COINIT_MULTITHREADED);
            comReady = true;
        }

        // Кадр может прийти пустым (смена формата, пропуск) - это не ошибка,
        // просто ждём следующий. Но и бесконечно ждать нельзя.
        for (int attempt = 0; attempt < 120; ++attempt) {
            DWORD streamFlags = 0;
            LONGLONG timestamp = 0;
            IMFSample* sample = nullptr;

            const HRESULT hr = reader_->ReadSample(
                static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), 0, nullptr,
                &streamFlags, &timestamp, &sample);
            if (FAILED(hr)) return false;
            if (streamFlags & MF_SOURCE_READERF_ENDOFSTREAM) {
                safeRelease(sample);
                return false;
            }
            if (!sample) continue;  // таймаут устройства, пробуем ещё раз

            const bool ok = decodeSample(sample, out);
            safeRelease(sample);
            if (!ok) continue;

            out.sourceIndex = index_;
            out.sequence = frames_++;
            // Метка ставится по приходу кадра: USB-камеры не синхронизированы
            // аппаратно, и собственные метки устройств несравнимы между собой.
            out.timestampUs = nowMicros() + cfg_.startOffsetUs;
            return true;
        }
        return false;
    }

    void cancel() override {
        // Flush заставляет висящий ReadSample вернуться: другого способа
        // прервать синхронное чтение у читателя нет.
        if (reader_) reader_->Flush(static_cast<DWORD>(MF_SOURCE_READER_ALL_STREAMS));
    }

    void close() override { safeRelease(reader_); }

private:
    int resolveDevice(const DeviceList& devices, std::string* error) {
        // Пустой путь - берём устройство по порядковому номеру источника.
        if (cfg_.path.empty()) {
            if (index_ >= 0 && static_cast<UINT32>(index_) < devices.count) return index_;
            if (error) *error = "камер меньше, чем источников в конфигурации";
            return -1;
        }

        const bool numeric = cfg_.path.find_first_not_of("0123456789") == std::string::npos;
        if (numeric) {
            const int n = std::atoi(cfg_.path.c_str());
            if (n >= 0 && static_cast<UINT32>(n) < devices.count) return n;
            if (error) {
                *error = "камера с индексом " + cfg_.path + " не найдена, всего камер: " +
                         std::to_string(devices.count);
            }
            return -1;
        }

        // Иначе путь - подстрока символьной ссылки. Она переживает перезапуск
        // и перетыкание кабеля, в отличие от индекса.
        for (UINT32 i = 0; i < devices.count; ++i) {
            const std::string link = attributeString(
                devices.items[i], MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK);
            if (link.find(cfg_.path) != std::string::npos) return static_cast<int>(i);
        }
        if (error) *error = "ни одна камера не соответствует '" + cfg_.path + "'";
        return -1;
    }

    // Ищем родной MJPG нужного размера: его буфер - готовый JPEG, который
    // декодер проекта принимает напрямую, без промежуточных преобразований.
    bool selectFormat(std::string* error) {
        std::string available;
        IMFMediaType* best = nullptr;
        double bestFps = 1e9;

        for (DWORD i = 0; i < 256; ++i) {
            IMFMediaType* type = nullptr;
            if (FAILED(reader_->GetNativeMediaType(
                    static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), i, &type))) {
                break;
            }

            GUID subtype = GUID_NULL;
            UINT32 w = 0, h = 0, num = 0, den = 1;
            type->GetGUID(MF_MT_SUBTYPE, &subtype);
            MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
            MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &num, &den);

            const double fps = den ? static_cast<double>(num) / static_cast<double>(den) : 0.0;
            const bool sizeOk =
                static_cast<int>(w) == cfg_.width && static_cast<int>(h) == cfg_.height;

            // Точное совпадение по частоте важнее: камера перечисляет 60
            // раньше 30, и "не ниже запрошенной" молча выбрало бы 60 -
            // лишняя нагрузка на шину, особенно когда камер две.
            if (subtype == MFVideoFormat_MJPG && sizeOk && fps >= cfg_.fps - 0.5) {
                const bool exact = fps <= cfg_.fps + 0.5;
                if (exact || fps < bestFps) {
                    safeRelease(best);
                    best = type;
                    best->AddRef();
                    bestFps = fps;
                }
                if (exact) {
                    safeRelease(type);
                    break;
                }
            }

            if (subtype == MFVideoFormat_MJPG && fps >= 14.9) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%ux%u@%.0f ", w, h, fps);
                if (available.find(buf) == std::string::npos) available += buf;
            }
            safeRelease(type);
        }

        if (best) {
            const HRESULT hr = reader_->SetCurrentMediaType(
                static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, best);
            safeRelease(best);
            if (SUCCEEDED(hr)) {
                mjpeg_ = true;
                return true;
            }
        }

        // Запасной путь: просим RGB32, преобразование сделает сам читатель.
        IMFMediaType* rgb = nullptr;
        if (SUCCEEDED(MFCreateMediaType(&rgb))) {
            rgb->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
            rgb->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
            MFSetAttributeSize(rgb, MF_MT_FRAME_SIZE, static_cast<UINT32>(cfg_.width),
                               static_cast<UINT32>(cfg_.height));
            const HRESULT hr = reader_->SetCurrentMediaType(
                static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, rgb);
            safeRelease(rgb);
            if (SUCCEEDED(hr)) {
                mjpeg_ = false;
                return true;
            }
        }

        if (error) {
            *error = "камера не отдаёт " + std::to_string(cfg_.width) + "x" +
                     std::to_string(cfg_.height) + " при " +
                     std::to_string(static_cast<int>(cfg_.fps)) + " кадр/с.";
            if (!available.empty()) *error += " Доступны MJPG: " + available;
            *error += " Полный список - workzone_server --list-cameras";
        }
        return false;
    }

    bool decodeSample(IMFSample* sample, Frame& out) {
        IMFMediaBuffer* buffer = nullptr;
        if (FAILED(sample->ConvertToContiguousBuffer(&buffer))) return false;

        BYTE* data = nullptr;
        DWORD length = 0;
        if (FAILED(buffer->Lock(&data, nullptr, &length)) || length == 0) {
            safeRelease(buffer);
            return false;
        }

        bool ok = false;
        if (mjpeg_) {
            std::string err;
            ok = loadImageFromMemory(data, length, out.image, &err);
        } else {
            ok = convertRgb32(data, length, out.image);
        }

        buffer->Unlock();
        safeRelease(buffer);
        return ok;
    }

    // RGB32 приходит как BGRA и, как правило, строками снизу вверх.
    bool convertRgb32(const BYTE* data, DWORD length, Image& out) {
        const int w = cfg_.width, h = cfg_.height;
        if (static_cast<DWORD>(w) * static_cast<DWORD>(h) * 4u > length) return false;
        if (out.width() != w || out.height() != h) out = Image(w, h);

        for (int y = 0; y < h; ++y) {
            const BYTE* row = data + static_cast<size_t>(h - 1 - y) * w * 4;
            for (int x = 0; x < w; ++x) {
                const BYTE* px = row + static_cast<size_t>(x) * 4;
                out.set(x, y,
                        RGB{static_cast<float>(px[2]), static_cast<float>(px[1]),
                            static_cast<float>(px[0])});
            }
        }
        return true;
    }

    SourceConfig cfg_;
    int index_ = -1;
    uint64_t frames_ = 0;
    bool mjpeg_ = true;
    IMFSourceReader* reader_ = nullptr;
};

}  // namespace

std::vector<CameraInfo> listCameras(std::string* error) {
    std::vector<CameraInfo> result;
    if (!ensureMediaFoundation(error)) return result;

    DeviceList devices;
    if (!enumerateDevices(devices, error)) return result;

    for (UINT32 i = 0; i < devices.count; ++i) {
        CameraInfo info;
        info.index = static_cast<int>(i);
        info.name = attributeString(devices.items[i], MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME);
        info.symbolicLink = attributeString(
            devices.items[i], MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK);
        info.modes = describeModes(devices.items[i]);
        result.push_back(std::move(info));
    }
    return result;
}

std::unique_ptr<ISource> createCameraSource(const SourceConfig& cfg, int sourceIndex,
                                            std::string* error) {
    auto camera = std::make_unique<MediaFoundationCamera>(cfg, sourceIndex);
    if (!camera->open(error)) return nullptr;
    return camera;
}

}  // namespace wz

#else  // не Windows

namespace wz {

std::vector<CameraInfo> listCameras(std::string* error) {
    if (error) {
        *error =
            "перечисление камер реализовано только для Windows (Media Foundation); "
            "для Linux нужен backend V4L2, см. docs/HARDWARE.md";
    }
    return {};
}

std::unique_ptr<ISource> createCameraSource(const SourceConfig&, int, std::string* error) {
    if (error) {
        *error =
            "захват с устройства реализован только для Windows (Media Foundation); "
            "для записей используйте type=jpegdir или type=mjpeg";
    }
    return nullptr;
}

}  // namespace wz

#endif
