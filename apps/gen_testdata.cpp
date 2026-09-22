// Генератор тестовых записей: раскладывает кадры виртуальной сцены по каталогам,
// чтобы конвейер можно было гонять на файлах, как на настоящих записях камер.
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "wz/config.hpp"
#include "wz/synthetic.hpp"

#ifdef _WIN32
#include <direct.h>
#define wz_mkdir(p) _mkdir(p)
#else
#include <sys/stat.h>
#define wz_mkdir(p) ::mkdir(p, 0755)
#endif

namespace {

void printUsage() {
    std::cout <<
        R"(Генератор тестовых данных для системы наблюдения рабочей зоны.

Использование: gen_testdata [опции]

  --out <каталог>     куда писать (по умолчанию data)
  --frames <N>        сколько кадров на камеру (по умолчанию 120)
  --size <ШхВ>        размер кадра (по умолчанию 640x480)
  --fps <N>           частота, влияет на шаг времени сцены (по умолчанию 30)
  --quality <1..100>  качество JPEG (по умолчанию 90)
  --mjpeg             дополнительно склеить кадры в один файл .mjpeg
  --help              эта справка

Результат:
  <out>/cam_left/000000.jpg ...     кадры левой камеры
  <out>/cam_right/000000.jpg ...    кадры правой камеры
  <out>/system.json                 конфигурация под эти записи

Запуск на сгенерированных данных:
  workzone_server --config data/system.json
)";
}

bool parseSize(const std::string& text, int& w, int& h) {
    const size_t x = text.find_first_of("xX*");
    if (x == std::string::npos) return false;
    w = std::atoi(text.substr(0, x).c_str());
    h = std::atoi(text.substr(x + 1).c_str());
    return w > 0 && h > 0;
}

std::string frameName(const std::string& dir, int index) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "/%06d.jpg", index);
    return dir + buf;
}

}  // namespace

int main(int argc, char** argv) {
    std::string outDir = "data";
    int frames = 120;
    int width = 640;
    int height = 480;
    double fps = 30.0;
    int quality = 90;
    bool alsoMjpeg = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasNext = i + 1 < argc;

        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else if (arg == "--out" && hasNext) {
            outDir = argv[++i];
        } else if (arg == "--frames" && hasNext) {
            frames = std::atoi(argv[++i]);
        } else if (arg == "--fps" && hasNext) {
            fps = std::atof(argv[++i]);
        } else if (arg == "--quality" && hasNext) {
            quality = std::atoi(argv[++i]);
        } else if (arg == "--mjpeg") {
            alsoMjpeg = true;
        } else if (arg == "--size" && hasNext) {
            const std::string value = argv[++i];
            if (!parseSize(value, width, height)) {
                std::cerr << "Не удалось разобрать размер: " << value << "\n";
                return 1;
            }
        } else {
            std::cerr << "Неизвестный аргумент: " << arg << "\n\n";
            printUsage();
            return 1;
        }
    }

    if (frames < 1 || fps <= 0.0) {
        std::cerr << "Число кадров и частота должны быть положительными\n";
        return 1;
    }

    wz_mkdir(outDir.c_str());

    // Конфигурация строится от синтетической сцены, а затем источники
    // переключаются на каталоги с кадрами: геометрия остаётся истинной.
    wz::SystemConfig cfg = wz::SystemConfig::defaults();
    if (cfg.sources.size() != 2) {
        std::cerr << "Ожидалось две камеры в конфигурации по умолчанию\n";
        return 1;
    }

    const wz::SceneGeometry scene = wz::sceneGeometry();
    const wz::Mat3 toCanvas =
        wz::planeToCanvas(cfg.stitch, scene.plateWidthMm, scene.plateHeightMm);

    for (size_t view = 0; view < cfg.sources.size(); ++view) {
        const std::string id = cfg.sources[view].id;
        const std::string camDir = outDir + "/" + id;
        wz_mkdir(camDir.c_str());

        const wz::SyntheticView sv = wz::syntheticView(static_cast<int>(view), width, height);

        wz::Image frame;
        std::vector<uint8_t> mjpeg;

        for (int i = 0; i < frames; ++i) {
            const double t = static_cast<double>(i) / fps;
            wz::renderSyntheticFrame(sv, t, frame);

            std::string err;
            if (!wz::saveImageJpeg(frameName(camDir, i), frame, quality, &err)) {
                std::cerr << "Ошибка записи кадра: " << err << "\n";
                return 1;
            }

            if (alsoMjpeg) {
                std::vector<uint8_t> encoded;
                if (wz::encodeJpeg(frame, quality, encoded)) {
                    mjpeg.insert(mjpeg.end(), encoded.begin(), encoded.end());
                }
            }

            if ((i + 1) % 20 == 0 || i + 1 == frames) {
                std::cout << "\r" << id << ": " << (i + 1) << "/" << frames << std::flush;
            }
        }
        std::cout << "\n";

        if (alsoMjpeg) {
            const std::string path = outDir + "/" + id + ".mjpeg";
            std::FILE* f = std::fopen(path.c_str(), "wb");
            if (!f) {
                std::cerr << "Не удалось открыть на запись " << path << "\n";
                return 1;
            }
            std::fwrite(mjpeg.data(), 1, mjpeg.size(), f);
            std::fclose(f);
            std::cout << "  " << path << " (" << mjpeg.size() / 1024 << " КБ)\n";
        }

        // Источник переключается с генератора на записанные кадры.
        cfg.sources[view].type = "jpegdir";
        cfg.sources[view].path = outDir + "/" + id;
        cfg.sources[view].width = width;
        cfg.sources[view].height = height;
        cfg.sources[view].fps = fps;
        cfg.sources[view].loop = true;

        cfg.cameras[view].imageWidth = width;
        cfg.cameras[view].imageHeight = height;
        cfg.cameras[view].intrinsics = sv.intrinsics;
        cfg.cameras[view].homography = toCanvas * sv.imageToPlane;
        cfg.cameras[view].homography.normalizeH();
    }

    const std::string cfgPath = outDir + "/system.json";
    std::string err;
    if (!cfg.save(cfgPath, &err)) {
        std::cerr << "Не удалось сохранить конфигурацию: " << err << "\n";
        return 1;
    }

    std::cout << "\nГотово. Конфигурация: " << cfgPath << "\nЗапуск: workzone_server --config "
              << cfgPath << "\n";
    return 0;
}
