// Калибровка внутренних параметров камеры по шахматной доске.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "wz/chessboard.hpp"
#include "wz/intrinsics_calib.hpp"
#include "wz/source.hpp"

namespace {

void printUsage() {
    std::cout <<
        R"(Калибровка внутренних параметров камеры по шахматной доске.

Использование: calibrate_intrinsics [опции]

Откуда брать снимки (нужно указать одно):
  --images <каталог>    готовые снимки доски
  --camera <индекс>     снимать с камеры прямо сейчас; индекс или кусок
                        символьной ссылки из workzone_server --list-cameras

Параметры доски:
  --cols <N>            внутренних углов по горизонтали (по умолчанию 9)
  --rows <N>            внутренних углов по вертикали (по умолчанию 6)
  --square <мм>         сторона клетки (по умолчанию 25)

Прочее:
  --shots <N>           сколько снимков собрать с камеры (по умолчанию 15)
  --size <ШхВ>          разрешение съёмки (по умолчанию 640x480)
  --save-shots <кат>    сохранить снятые кадры
  --out <файл.json>     куда записать параметры (по умолчанию config/intrinsics.json)
  --help                эта справка

Доска задаётся числом ВНУТРЕННИХ углов, а не клеток: у доски 10x7 клеток
углов 9x6. Стороны должны различаться - на квадратной доске не определить,
как она повёрнута.

Как снимать, чтобы получилось:
  - не менее 10 снимков под РАЗНЫМИ наклонами и поворотами; параллельные
    снимки с одного ракурса дают вырожденную систему;
  - доска должна побывать у всех четырёх краёв кадра, иначе дисторсия
    на больших радиусах не определяется - именно там она и заметна;
  - доска плоская и жёсткая, лист бумаги не годится;
  - равномерный свет, без бликов на глянце.
)";
}

bool parseSize(const std::string& text, int& w, int& h) {
    const size_t x = text.find_first_of("xX*");
    if (x == std::string::npos) return false;
    w = std::atoi(text.substr(0, x).c_str());
    h = std::atoi(text.substr(x + 1).c_str());
    return w > 0 && h > 0;
}

// Снимки из каталога: каждый проверяется на наличие доски целиком.
bool collectFromDirectory(const std::string& dir, const wz::ChessboardSpec& spec,
                          std::vector<wz::ChessboardCorners>& out, int& width, int& height) {
    const std::vector<std::string> files = wz::listImageFiles(dir);
    if (files.empty()) {
        std::cerr << "В каталоге " << dir << " нет изображений\n";
        return false;
    }

    for (const std::string& path : files) {
        wz::Image img;
        std::string err;
        if (!wz::loadImageFile(path, img, &err)) {
            std::cout << "  " << path << ": не читается\n";
            continue;
        }
        if (width == 0) {
            width = img.width();
            height = img.height();
        } else if (img.width() != width || img.height() != height) {
            std::cout << "  " << path << ": размер отличается от первого снимка, пропуск\n";
            continue;
        }

        wz::ChessboardCorners corners;
        if (!wz::findChessboard(img, spec, corners, &err)) {
            std::cout << "  " << path << ": " << err << "\n";
            continue;
        }
        corners.source = path;
        out.push_back(std::move(corners));
        std::cout << "  " << path << ": доска найдена\n";
    }
    return true;
}

// Съёмка с камеры. Кадры берутся не подряд: иначе получится десяток почти
// одинаковых ракурсов, а они системе ничего не добавляют.
bool collectFromCamera(const std::string& device, const wz::ChessboardSpec& spec, int shots,
                       int width, int height, const std::string& saveDir,
                       std::vector<wz::ChessboardCorners>& out) {
    wz::SourceConfig cfg;
    cfg.id = "calib";
    cfg.type = "camera";
    cfg.path = device;
    cfg.width = width;
    cfg.height = height;
    cfg.fps = 30.0;

    std::string err;
    auto source = wz::createSource(cfg, 0, &err);
    if (!source) {
        std::cerr << "Не удалось открыть камеру: " << err << "\n";
        return false;
    }

    std::cout << "Держите доску перед камерой и меняйте наклон после каждого снимка.\n"
              << "Нужно " << shots << " снимков, Ctrl+C для остановки.\n\n";

    wz::Frame frame;
    int accepted = 0;
    int sinceAccepted = 25;

    for (int attempt = 0; attempt < 4000 && accepted < shots; ++attempt) {
        if (!source->read(frame)) break;
        ++sinceAccepted;
        // Пауза между принятыми кадрами: за это время доску успевают повернуть.
        if (sinceAccepted < 25) continue;

        wz::ChessboardCorners corners;
        if (!wz::findChessboard(frame.image, spec, corners, &err)) continue;

        corners.source = "shot" + std::to_string(accepted);
        if (!saveDir.empty()) {
            char name[64];
            std::snprintf(name, sizeof(name), "/%02d.png", accepted);
            const std::string path = saveDir + name;
            std::string saveErr;
            if (wz::saveImagePng(path, frame.image, &saveErr)) corners.source = path;
        }

        out.push_back(std::move(corners));
        ++accepted;
        sinceAccepted = 0;
        std::cout << "  снимок " << accepted << " из " << shots << " принят\n";
    }

    source->close();
    return accepted > 0;
}

}  // namespace

int main(int argc, char** argv) {
    wz::ChessboardSpec spec;
    std::string imagesDir, cameraSpec, saveDir;
    std::string outPath = "config/intrinsics.json";
    int shots = 15;
    int width = 640, height = 480;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasNext = i + 1 < argc;

        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else if (arg == "--images" && hasNext) {
            imagesDir = argv[++i];
        } else if (arg == "--camera" && hasNext) {
            cameraSpec = argv[++i];
        } else if (arg == "--cols" && hasNext) {
            spec.cols = std::atoi(argv[++i]);
        } else if (arg == "--rows" && hasNext) {
            spec.rows = std::atoi(argv[++i]);
        } else if (arg == "--square" && hasNext) {
            spec.squareMm = std::atof(argv[++i]);
        } else if (arg == "--shots" && hasNext) {
            shots = std::atoi(argv[++i]);
        } else if (arg == "--save-shots" && hasNext) {
            saveDir = argv[++i];
        } else if (arg == "--out" && hasNext) {
            outPath = argv[++i];
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

    if (!spec.valid()) {
        std::cerr << "Некорректная доска " << spec.cols << "x" << spec.rows
                  << ": нужно не менее 3x3 внутренних углов, и стороны должны различаться "
                     "(на квадратной доске не определить поворот)\n";
        return 1;
    }
    if (imagesDir.empty() == cameraSpec.empty()) {
        std::cerr << "Укажите либо --images <каталог>, либо --camera <индекс>\n\n";
        printUsage();
        return 1;
    }

    std::cout << "Доска: " << spec.cols << "x" << spec.rows << " внутренних углов, клетка "
              << spec.squareMm << " мм\n\n";

    std::vector<wz::ChessboardCorners> views;
    if (!imagesDir.empty()) {
        int w = 0, h = 0;
        if (!collectFromDirectory(imagesDir, spec, views, w, h)) return 1;
        width = w;
        height = h;
    } else if (!collectFromCamera(cameraSpec, spec, shots, width, height, saveDir, views)) {
        return 1;
    }

    std::cout << "\nПригодных видов: " << views.size() << "\n";
    if (views.size() < 3) {
        std::cerr << "Этого мало: нужно минимум 3, а на практике 10 и больше.\n"
                     "Если доска не находится - проверьте освещение и что она попадает "
                     "в кадр целиком.\n";
        return 1;
    }

    wz::IntrinsicsCalibResult result;
    std::string err;
    if (!wz::calibrateIntrinsics(views, width, height, result, &err)) {
        std::cerr << "Калибровка не удалась: " << err << "\n";
        return 1;
    }

    const wz::Intrinsics& in = result.intrinsics;
    std::cout << std::fixed << std::setprecision(2)
              << "\nПринято видов: " << result.views << " из " << views.size() << "\n"
              << "Точек: " << result.points << "\n\n"
              << "fx = " << in.fx << "   fy = " << in.fy << "\n"
              << "cx = " << in.cx << "   cy = " << in.cy << "\n"
              << std::setprecision(5) << "k1 = " << in.k1 << "   k2 = " << in.k2 << "\n\n"
              << std::setprecision(3) << "Невязка перепроецирования: RMS " << result.rmsPx
              << " пикселя, максимум " << result.maxPx << "\n";

    if (result.rmsPx > 1.0) {
        std::cout << "\nНевязка великовата. Обычно это значит, что часть снимков смазана "
                     "или доска не жёсткая.\n";
    }
    if (result.views < 6) {
        std::cout << "\nВидов принято мало - параметры определены неуверенно. "
                     "Доснимите доску под другими наклонами.\n";
    }

    std::ofstream out(outPath, std::ios::binary);
    if (!out) {
        std::cerr << "Не удалось открыть на запись " << outPath << "\n";
        return 1;
    }
    out << wz::intrinsicsToJsonString(in) << "\n";

    std::cout << "\nЗаписано: " << outPath
              << "\nПеренесите эти значения в поле intrinsics нужной камеры "
                 "в config/system.json\n";
    return 0;
}
