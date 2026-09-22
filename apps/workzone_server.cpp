// Главное приложение: поднимает конвейер и отдаёт панораму в браузер.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "wz/json.hpp"
#include "wz/pipeline.hpp"

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop.store(true); }

void printUsage() {
    std::cout <<
        R"(Система камер для наблюдения рабочей зоны.

Использование: workzone_server [опции]

  --config <файл>        загрузить конфигурацию (по умолчанию config/system.json,
                         если файл существует; иначе синтетическая сцена)
  --host <адрес>         адрес привязки сервера (по умолчанию 127.0.0.1)
  --port <порт>          порт сервера (по умолчанию 8080)
  --canvas <ШхВ>         размер холста панорамы, например 1280x720
  --source <тип:путь>    заменить источники; опцию можно повторять.
                         Примеры: --source jpegdir:data/cam_left
                                  --source mjpeg:data/right.mjpeg
                                  --source camera:0
                                  --source synthetic:0
  --web <каталог>        каталог веб-интерфейса (по умолчанию web)
  --quality <1..100>     качество JPEG (по умолчанию 80)
  --snapshot <файл.png>  собрать одну панораму, сохранить и выйти
  --frames <N>           сколько кадров собрать до снимка (по умолчанию 30)
  --save-config <файл>   записать итоговую конфигурацию и выйти
  --layout side-by-side  показать камеры рядом, без склейки. Нужен, пока
                         камеры не откалиброваны: гомографий ещё нет,
                         а смотреть на зону уже надо
  --size <ШхВ>           разрешение запроса к камерам (по умолчанию 640x480)
  --grab <префикс>       сохранить кадр каждой камеры и панораму, затем выйти
                         (<префикс>_cam0.png, ..., <префикс>_panorama.png)
  --list-cameras         показать подключённые камеры и их режимы
  --help                 эта справка

Без аргументов запускается виртуальная сцена с двумя камерами: система
работает целиком, железо не требуется.
)";
}

bool parseSize(const std::string& text, int& w, int& h) {
    const size_t x = text.find_first_of("xX*");
    if (x == std::string::npos) return false;
    w = std::atoi(text.substr(0, x).c_str());
    h = std::atoi(text.substr(x + 1).c_str());
    return w > 0 && h > 0;
}

// Разбор --source: "тип:путь". Для synthetic путь - номер вида.
bool parseSource(const std::string& spec, wz::SourceConfig& out, std::string* error) {
    const size_t colon = spec.find(':');
    if (colon == std::string::npos) {
        if (error) *error = "ожидался формат тип:путь, получено '" + spec + "'";
        return false;
    }

    out.type = spec.substr(0, colon);
    const std::string rest = spec.substr(colon + 1);

    if (out.type == "synthetic") {
        out.syntheticView = std::atoi(rest.c_str());
        out.path.clear();
    } else {
        out.path = rest;
    }

    for (const std::string& known : wz::availableSourceTypes()) {
        if (out.type == known) return true;
    }
    if (error) *error = "неизвестный тип источника '" + out.type + "'";
    return false;
}

// Печатает найденные устройства: по этому списку заполняется path источника.
int printCameras() {
    std::string err;
    const std::vector<wz::CameraInfo> cameras = wz::listCameras(&err);

    if (!err.empty()) {
        std::cerr << "Не удалось получить список камер: " << err << "\n";
        return 1;
    }
    if (cameras.empty()) {
        std::cout << "Камер не найдено.\n";
        return 0;
    }

    std::cout << "Найдено камер: " << cameras.size() << "\n\n";
    for (const wz::CameraInfo& c : cameras) {
        std::cout << "[" << c.index << "] " << (c.name.empty() ? "(без имени)" : c.name) << "\n";
        if (!c.modes.empty()) std::cout << "    MJPG: " << c.modes << "\n";
        std::cout << "    ссылка: " << c.symbolicLink << "\n\n";
    }
    std::cout << "Запуск на камерах:\n"
              << "  workzone_server --source camera:0 --source camera:1\n\n"
              << "Индекс может поменяться при перетыкании кабеля. Чтобы привязка\n"
              << "не сбивалась, указывайте вместо него кусок символьной ссылки,\n"
              << "например --source camera:8&3b549e9e\n";
    return 0;
}

// Вписывает кадр камеры в прямоугольник холста с сохранением пропорций.
wz::Mat3 fitImageIntoRect(int imgW, int imgH, double rx, double ry, double rw, double rh) {
    const double s = std::min(rw / imgW, rh / imgH);
    const double ox = rx + (rw - imgW * s) * 0.5;
    const double oy = ry + (rh - imgH * s) * 0.5;
    return wz::Mat3::translation(ox, oy) * wz::Mat3::scaling(s, s);
}

bool fileExists(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fclose(f);
    return true;
}

std::string statusJson(const wz::Pipeline& pipeline, const wz::HttpServer& server) {
    const wz::PipelineStats s = pipeline.stats();

    wz::JsonValue root = wz::JsonValue::object();
    root.set("running", wz::JsonValue(pipeline.running()));
    root.set("framesStitched", wz::JsonValue(static_cast<double>(s.framesStitched)));
    root.set("framesDropped", wz::JsonValue(static_cast<double>(s.framesDropped)));
    root.set("outputFps", wz::JsonValue(s.outputFps));
    root.set("stitchMs", wz::JsonValue(s.stitchMs));
    root.set("processMs", wz::JsonValue(s.processMs));
    root.set("encodeMs", wz::JsonValue(s.encodeMs));
    root.set("canvasWidth", wz::JsonValue(s.canvasWidth));
    root.set("canvasHeight", wz::JsonValue(s.canvasHeight));
    root.set("coverage", wz::JsonValue(s.stitch.coverage));
    root.set("overlap", wz::JsonValue(s.stitch.overlap));
    root.set("httpRequests", wz::JsonValue(static_cast<double>(server.servedRequests())));
    root.set("activeStreams", wz::JsonValue(server.activeStreams()));

    wz::JsonValue sync = wz::JsonValue::object();
    sync.set("matched", wz::JsonValue(static_cast<double>(s.sync.matched)));
    sync.set("pushed", wz::JsonValue(static_cast<double>(s.sync.pushed)));
    sync.set("droppedStale", wz::JsonValue(static_cast<double>(s.sync.droppedStale)));
    sync.set("droppedOverflow", wz::JsonValue(static_cast<double>(s.sync.droppedOverflow)));
    sync.set("lastSpreadUs", wz::JsonValue(static_cast<double>(s.sync.lastSpreadUs)));
    sync.set("meanSpreadUs", wz::JsonValue(s.sync.meanSpreadUs));
    root.set("sync", std::move(sync));

    wz::JsonValue gains = wz::JsonValue::array();
    for (double g : s.stitch.gains) gains.push(wz::JsonValue(g));
    root.set("exposureGains", std::move(gains));

    wz::JsonValue sources = wz::JsonValue::array();
    const auto& cfgSources = pipeline.config().sources;
    for (size_t i = 0; i < cfgSources.size(); ++i) {
        wz::JsonValue o = wz::JsonValue::object();
        o.set("id", wz::JsonValue(cfgSources[i].id));
        o.set("type", wz::JsonValue(cfgSources[i].type));
        o.set("frames", wz::JsonValue(static_cast<double>(
                            i < s.perSourceFrames.size() ? s.perSourceFrames[i] : 0)));
        sources.push(std::move(o));
    }
    root.set("sources", std::move(sources));

    return root.dump(2);
}

std::string settingsJson(const wz::ProcessingConfig& p) {
    wz::JsonValue o = wz::JsonValue::object();
    o.set("brightness", wz::JsonValue(p.brightness));
    o.set("contrast", wz::JsonValue(p.contrast));
    o.set("saturation", wz::JsonValue(p.saturation));
    o.set("jpegQuality", wz::JsonValue(p.jpegQuality));
    o.set("cropX", wz::JsonValue(p.cropX));
    o.set("cropY", wz::JsonValue(p.cropY));
    o.set("cropW", wz::JsonValue(p.cropW));
    o.set("cropH", wz::JsonValue(p.cropH));
    o.set("showSeams", wz::JsonValue(p.showSeams));
    o.set("showCoverage", wz::JsonValue(p.showCoverage));
    return o.dump(2);
}

// Числовой параметр запроса: отсутствие или мусор оставляют текущее значение.
void applyFloatParam(const wz::HttpRequest& req, const char* key, float& target) {
    const std::string v = req.param(key);
    if (v.empty()) return;
    char* end = nullptr;
    const double parsed = std::strtod(v.c_str(), &end);
    if (end != v.c_str()) target = static_cast<float>(parsed);
}

void applyBoolParam(const wz::HttpRequest& req, const char* key, bool& target) {
    const std::string v = req.param(key);
    if (v.empty()) return;
    target = (v == "1" || v == "true" || v == "on" || v == "yes");
}

}  // namespace

int main(int argc, char** argv) {
    std::string configPath;
    std::string snapshotPath;
    std::string saveConfigPath;
    std::string grabPrefix;
    bool sideBySide = false;
    int srcW = 0, srcH = 0;
    int snapshotAfter = 30;
    std::vector<wz::SourceConfig> customSources;

    wz::SystemConfig cfg = wz::SystemConfig::defaults();

    // Первый проход: ищем --config, чтобы остальные опции могли его перекрыть.
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
        if (arg == "--list-cameras") return printCameras();
        if (arg == "--config" && i + 1 < argc) configPath = argv[++i];
    }

    if (configPath.empty() && fileExists("config/system.json")) {
        configPath = "config/system.json";
    }
    if (!configPath.empty()) {
        std::string err;
        if (!wz::SystemConfig::load(configPath, cfg, &err)) {
            std::cerr << "Ошибка конфигурации " << configPath << ": " << err << "\n";
            return 1;
        }
        std::cout << "Конфигурация загружена: " << configPath << "\n";
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasNext = i + 1 < argc;

        if (arg == "--list-cameras") {
            continue;  // обработан в первом проходе
        } else if (arg == "--config" && hasNext) {
            ++i;  // уже обработан в первом проходе
        } else if (arg == "--host" && hasNext) {
            cfg.server.bindAddress = argv[++i];
        } else if (arg == "--port" && hasNext) {
            cfg.server.port = std::atoi(argv[++i]);
        } else if (arg == "--web" && hasNext) {
            cfg.server.webRoot = argv[++i];
        } else if (arg == "--quality" && hasNext) {
            cfg.processing.jpegQuality = std::atoi(argv[++i]);
        } else if (arg == "--frames" && hasNext) {
            snapshotAfter = std::atoi(argv[++i]);
        } else if (arg == "--layout" && hasNext) {
            const std::string value = argv[++i];
            if (value == "side-by-side") {
                sideBySide = true;
            } else if (value != "config") {
                std::cerr << "Неизвестная раскладка: " << value
                          << " (допустимо: side-by-side, config)\n";
                return 1;
            }
        } else if (arg == "--size" && hasNext) {
            const std::string value = argv[++i];
            if (!parseSize(value, srcW, srcH)) {
                std::cerr << "Не удалось разобрать размер: " << value << "\n";
                return 1;
            }
        } else if (arg == "--grab" && hasNext) {
            grabPrefix = argv[++i];
        } else if (arg == "--snapshot" && hasNext) {
            snapshotPath = argv[++i];
        } else if (arg == "--save-config" && hasNext) {
            saveConfigPath = argv[++i];
        } else if (arg == "--canvas" && hasNext) {
            int w = 0, h = 0;
            const std::string value = argv[++i];
            if (!parseSize(value, w, h)) {
                std::cerr << "Не удалось разобрать размер холста: " << value << "\n";
                return 1;
            }
            cfg.stitch.canvasWidth = w;
            cfg.stitch.canvasHeight = h;
        } else if (arg == "--source" && hasNext) {
            wz::SourceConfig src;
            std::string err;
            if (!parseSource(argv[++i], src, &err)) {
                std::cerr << "Ошибка в --source: " << err << "\n";
                return 1;
            }
            customSources.push_back(src);
        } else {
            std::cerr << "Неизвестный аргумент: " << arg << "\n\n";
            printUsage();
            return 1;
        }
    }

    if (!customSources.empty() && !sideBySide) {
        if (customSources.size() != cfg.cameras.size()) {
            std::cerr << "Задано источников: " << customSources.size()
                      << ", а камер в конфигурации: " << cfg.cameras.size()
                      << ".\nКаждому источнику нужна своя гомография - подготовьте конфигурацию "
                         "утилитой calibrate, либо запустите с --layout side-by-side.\n";
            return 1;
        }
        // Имена, размер кадра и темп берём из описания камер: склейка уже
        // настроена под них, подменяется только способ получения кадров.
        for (size_t i = 0; i < customSources.size(); ++i) {
            customSources[i].id = cfg.sources[i].id;
            customSources[i].width = cfg.cameras[i].imageWidth;
            customSources[i].height = cfg.cameras[i].imageHeight;
            customSources[i].fps = cfg.sources[i].fps;
            customSources[i].startOffsetUs = cfg.sources[i].startOffsetUs;
        }
        cfg.sources = customSources;
    }

    // Раскладка "рядом": гомографии не калибруются, а просто раскладывают
    // кадры по холсту. Плитки не перекрываются, поэтому растушёвка и
    // отбраковка перекрытий здесь только испортили бы картинку.
    if (sideBySide) {
        if (!customSources.empty()) cfg.sources = customSources;
        if (cfg.sources.empty()) {
            std::cerr << "Для --layout side-by-side задайте источники через --source@";
            return 1;
        }

        const size_t n = cfg.sources.size();
        const double tileW = static_cast<double>(cfg.stitch.canvasWidth) / static_cast<double>(n);

        cfg.cameras.assign(n, wz::CameraPlacement{});
        for (size_t i = 0; i < n; ++i) {
            wz::SourceConfig& src = cfg.sources[i];
            src.id = "cam" + std::to_string(i);
            if (srcW > 0) src.width = srcW;
            if (srcH > 0) src.height = srcH;

            wz::CameraPlacement& cam = cfg.cameras[i];
            cam.sourceId = src.id;
            cam.imageWidth = src.width;
            cam.imageHeight = src.height;
            cam.intrinsics = wz::Intrinsics{};  // параметры камер неизвестны
            cam.homography = fitImageIntoRect(src.width, src.height,
                                              static_cast<double>(i) * tileW, 0.0, tileW,
                                              cfg.stitch.canvasHeight);
        }

        cfg.stitch.featherPx = 0.0f;
        cfg.stitch.exposureCompensation = false;
        cfg.stitch.occlusionRejection = false;
    }

    {
        std::string err;
        if (!cfg.validate(&err)) {
            std::cerr << "Конфигурация некорректна: " << err << "\n";
            return 1;
        }
    }

    if (!saveConfigPath.empty()) {
        std::string err;
        if (!cfg.save(saveConfigPath, &err)) {
            std::cerr << "Не удалось сохранить конфигурацию: " << err << "\n";
            return 1;
        }
        std::cout << "Конфигурация записана в " << saveConfigPath << "\n";
        return 0;
    }

    wz::Pipeline pipeline;
    {
        std::string err;
        if (!pipeline.start(cfg, &err)) {
            std::cerr << "Не удалось запустить конвейер: " << err << "\n";
            return 1;
        }
    }

    std::cout << "Источники: ";
    for (size_t i = 0; i < cfg.sources.size(); ++i) {
        std::cout << (i ? ", " : "") << cfg.sources[i].id << " (" << cfg.sources[i].type << ")";
    }
    std::cout << "\nХолст: " << cfg.stitch.canvasWidth << "x" << cfg.stitch.canvasHeight << "\n";

    // Режим снятия кадров: сохраняет и то, что видит каждая камера,
    // и текущую панораму. По кадрам камер потом указываются метки
    // для калибровки.
    if (!grabPrefix.empty()) {
        pipeline.setKeepRawFrames(true);

        for (int i = 0; i < 400; ++i) {
            if (pipeline.stats().framesStitched >= 12) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        const wz::PipelineStats diag = pipeline.stats();
        std::cout << "С источников получено кадров:";
        for (size_t i = 0; i < diag.perSourceFrames.size(); ++i) {
            std::cout << " " << cfg.sources[i].id << "=" << diag.perSourceFrames[i];
        }
        std::cout << "\nсведено наборов: " << diag.sync.matched
                  << ", отброшено устаревших: " << diag.sync.droppedStale
                  << ", расхождение меток: " << diag.sync.lastSpreadUs / 1000 << " мс\n";

        std::vector<wz::Image> raw;
        if (!pipeline.rawFrames(raw)) {
            if (diag.sync.pushed == 0) {
                std::cerr << "Ни один источник не отдал кадр - проверьте, "
                             "не занята ли камера другим приложением.\n";
            } else {
                std::cerr << "Кадры идут, но синхронизатор не свёл ни одного набора. "
                             "Камеры не синхронизированы аппаратно, и постоянный сдвиг фаз "
                             "может превышать допуск sync.toleranceUs ("
                          << cfg.sync.toleranceUs / 1000 << " мс). "
                             "Увеличьте его до периода кадра.\n";
            }
            pipeline.requestStop();
            pipeline.join();
            return 1;
        }

        std::string err;
        for (size_t i = 0; i < raw.size(); ++i) {
            const std::string path = grabPrefix + "_cam" + std::to_string(i) + ".png";
            if (!wz::saveImagePng(path, raw[i], &err)) {
                std::cerr << "Не удалось сохранить " << path << ": " << err << "\n";
                pipeline.requestStop();
                pipeline.join();
                return 1;
            }
            std::cout << path << "  " << raw[i].width() << "x" << raw[i].height() << "\n";
        }

        std::vector<uint8_t> jpeg;
        if (pipeline.broker().latest(jpeg)) {
            wz::Image pano;
            const std::string path = grabPrefix + "_panorama.png";
            if (wz::loadImageFromMemory(jpeg.data(), jpeg.size(), pano, &err) &&
                wz::saveImagePng(path, pano, &err)) {
                std::cout << path << "  " << pano.width() << "x" << pano.height() << "\n";
            }
        }

        const wz::PipelineStats s = pipeline.stats();
        std::cout << "Сведено наборов: " << s.sync.matched << ", расхождение меток "
                  << s.sync.lastSpreadUs / 1000 << " мс\n";

        pipeline.requestStop();
        pipeline.join();
        return 0;
    }

    // Режим снимка: ни сервера, ни браузера - удобно для проверки в CI.
    if (!snapshotPath.empty()) {
        for (int i = 0; i < 600; ++i) {
            if (pipeline.stats().framesStitched >= static_cast<uint64_t>(snapshotAfter)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::vector<uint8_t> jpeg;
        if (!pipeline.broker().latest(jpeg)) {
            std::cerr << "Кадр так и не был собран\n";
            pipeline.requestStop();
            pipeline.join();
            return 1;
        }

        wz::Image img;
        std::string err;
        if (!wz::loadImageFromMemory(jpeg.data(), jpeg.size(), img, &err) ||
            !wz::saveImagePng(snapshotPath, img, &err)) {
            std::cerr << "Не удалось сохранить снимок: " << err << "\n";
            pipeline.requestStop();
            pipeline.join();
            return 1;
        }

        const wz::PipelineStats s = pipeline.stats();
        std::cout << "Снимок: " << snapshotPath << " (" << img.width() << "x" << img.height()
                  << ")\n"
                  << "Собрано кадров: " << s.framesStitched << ", покрытие "
                  << static_cast<int>(s.stitch.coverage * 100) << "%, перекрытие "
                  << static_cast<int>(s.stitch.overlap * 100) << "%\n"
                  << "Расхождение меток времени: " << s.sync.lastSpreadUs << " мкс\n";

        pipeline.requestStop();
        pipeline.join();
        return 0;
    }

    wz::HttpServer server;
    server.setApiHandler([&](const wz::HttpRequest& req, std::string& contentType) -> std::string {
        contentType = "application/json; charset=utf-8";

        if (req.path == "/api/status") return statusJson(pipeline, server);
        if (req.path == "/api/config") return pipeline.config().toJson();

        if (req.path == "/api/settings") {
            wz::ProcessingConfig p = pipeline.processing();

            if (req.param("reset") == "1") {
                p = wz::ProcessingConfig{};
            } else {
                applyFloatParam(req, "brightness", p.brightness);
                applyFloatParam(req, "contrast", p.contrast);
                applyFloatParam(req, "saturation", p.saturation);
                applyFloatParam(req, "cropX", p.cropX);
                applyFloatParam(req, "cropY", p.cropY);
                applyFloatParam(req, "cropW", p.cropW);
                applyFloatParam(req, "cropH", p.cropH);
                applyBoolParam(req, "showSeams", p.showSeams);
                applyBoolParam(req, "showCoverage", p.showCoverage);

                const std::string q = req.param("jpegQuality");
                if (!q.empty()) p.jpegQuality = std::atoi(q.c_str());
            }

            pipeline.setProcessing(p);
            return settingsJson(pipeline.processing());
        }
        return std::string();
    });

    {
        std::string err;
        if (!server.start(cfg.server, &pipeline.broker(), &err)) {
            std::cerr << "Не удалось запустить сервер: " << err << "\n";
            pipeline.requestStop();
            pipeline.join();
            return 1;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "Интерфейс: http://" << cfg.server.bindAddress << ":" << cfg.server.port << "/\n"
              << "Поток:     http://" << cfg.server.bindAddress << ":" << cfg.server.port
              << "/stream.mjpg\n"
              << "Остановка: Ctrl+C\n";

    while (!g_stop.load() && pipeline.running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "\nОстановка...\n";
    server.stop();
    pipeline.requestStop();
    pipeline.join();

    const wz::PipelineStats s = pipeline.stats();
    std::cout << "Собрано кадров: " << s.framesStitched << ", отброшено " << s.framesDropped
              << ", среднее расхождение меток " << static_cast<int64_t>(s.sync.meanSpreadUs)
              << " мкс\n";
    return 0;
}
