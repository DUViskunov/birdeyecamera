// Калибровка: по соответствиям "точка на кадре - точка на столе" строит
// гомографию каждой камеры и собирает готовую конфигурацию системы.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "wz/config.hpp"
#include "wz/json.hpp"
#include "wz/synthetic.hpp"

namespace {

void printUsage() {
    std::cout <<
        R"(Калибровка камер системы наблюдения рабочей зоны.

Использование: calibrate [опции]

  --points <файл.json>  соответствия точек, снятые с реальных камер
  --from-scene          взять соответствия из виртуальной сцены и проверить
                        результат по известной истинной геометрии
  --canvas <ШхВ>        размер холста панорамы (по умолчанию 1280x720)
  --out <файл.json>     куда записать конфигурацию (по умолчанию config/system.json)
  --help                эта справка

Формат файла соответствий:
{
  "plate":  { "widthMm": 400, "heightMm": 300 },
  "cameras": [
    {
      "id": "cam_left",
      "imageWidth": 640, "imageHeight": 480,
      "source": { "type": "jpegdir", "path": "data/cam_left", "fps": 30 },
      "intrinsics": { "fx": 430, "fy": 430, "cx": 320, "cy": 240,
                      "k1": -0.145, "k2": 0.031 },
      "points": [
        { "image": [122, 355], "plane": [60, 50] },
        { "image": [498, 340], "plane": [340, 50] },
        { "image": [462, 168], "plane": [340, 250] },
        { "image": [151, 175], "plane": [60, 250] }
      ]
    }
  ]
}

Нужно не менее четырёх точек на камеру, и никакие три из них не должны
лежать на одной прямой. Координаты image указываются по сырому кадру:
дисторсия снимается автоматически по заданным intrinsics.
)";
}

bool parseSize(const std::string& text, int& w, int& h) {
    const size_t x = text.find_first_of("xX*");
    if (x == std::string::npos) return false;
    w = std::atoi(text.substr(0, x).c_str());
    h = std::atoi(text.substr(x + 1).c_str());
    return w > 0 && h > 0;
}

wz::Intrinsics intrinsicsFrom(const wz::JsonValue* v) {
    wz::Intrinsics in;
    if (!v || !v->isObject()) return in;
    in.fx = v->numberAt("fx", 0.0);
    in.fy = v->numberAt("fy", 0.0);
    in.cx = v->numberAt("cx", 0.0);
    in.cy = v->numberAt("cy", 0.0);
    in.k1 = v->numberAt("k1", 0.0);
    in.k2 = v->numberAt("k2", 0.0);
    in.k3 = v->numberAt("k3", 0.0);
    in.p1 = v->numberAt("p1", 0.0);
    in.p2 = v->numberAt("p2", 0.0);
    return in;
}

bool pairFrom(const wz::JsonValue* v, wz::Vec2& out) {
    if (!v || !v->isArray() || v->items().size() != 2) return false;
    out.x = v->items()[0].asNumber(0.0);
    out.y = v->items()[1].asNumber(0.0);
    return true;
}

void report(const std::string& id, const wz::ReprojectionError& e, int pointCount) {
    std::cout << "  " << std::left << std::setw(12) << id << " точек: " << pointCount
              << ", ошибка перепроецирования: средняя " << std::fixed << std::setprecision(3)
              << e.mean << " мм, максимум " << e.max << " мм\n";
    if (e.max > 2.0) {
        std::cout << "    предупреждение: максимум больше 2 мм - проверьте, точно ли "
                     "указаны точки и верны ли intrinsics\n";
    }
}

// Калибровка по виртуальной сцене: истинная геометрия известна, поэтому
// заодно проверяем, что DLT восстанавливает её с точностью до долей миллиметра.
int calibrateFromScene(wz::SystemConfig& cfg) {
    const wz::SceneGeometry scene = wz::sceneGeometry();
    const wz::Mat3 toCanvas =
        wz::planeToCanvas(cfg.stitch, scene.plateWidthMm, scene.plateHeightMm);
    const std::vector<wz::Vec2> markers = wz::sceneMarkersMm();

    std::cout << "Калибровка по виртуальной сцене (" << markers.size() << " меток на столе)\n";

    double worstTruthErrorMm = 0.0;

    for (size_t view = 0; view < cfg.cameras.size(); ++view) {
        const wz::SyntheticView sv = wz::syntheticView(
            static_cast<int>(view), cfg.cameras[view].imageWidth, cfg.cameras[view].imageHeight);

        // Метки проходят полный путь оператора: проекция на стол -> сырой кадр
        // (с дисторсией) -> снятие дисторсии. Так учитываются все погрешности.
        std::vector<wz::Vec2> imagePoints;
        std::vector<wz::Vec2> planePoints;
        for (const wz::Vec2& m : markers) {
            bool ok = false;
            const wz::Vec2 ideal = sv.planeToImage.project(m, &ok);
            if (!ok) continue;

            const wz::Vec2 raw = sv.intrinsics.undistortedPixelToRaw(ideal);
            if (raw.x < 0 || raw.y < 0 || raw.x >= sv.width || raw.y >= sv.height) continue;

            imagePoints.push_back(sv.intrinsics.rawPixelToUndistorted(raw));
            planePoints.push_back(m);
        }

        if (imagePoints.size() < 4) {
            std::cerr << "Камера " << view << ": в кадр попало меньше четырёх меток\n";
            return 1;
        }

        wz::Mat3 imageToPlane;
        if (!wz::estimateHomography(imagePoints, planePoints, imageToPlane)) {
            std::cerr << "Камера " << view << ": не удалось оценить гомографию\n";
            return 1;
        }

        report(cfg.sources[view].id, wz::reprojectionError(imageToPlane, imagePoints, planePoints),
               static_cast<int>(imagePoints.size()));

        // Сверка с истиной: гоняем те же точки кадра через истинную и через
        // оценённую гомографию и сравниваем результат в миллиметрах стола.
        for (const wz::Vec2& ip : imagePoints) {
            bool okA = false, okB = false;
            const wz::Vec2 a = imageToPlane.project(ip, &okA);
            const wz::Vec2 b = sv.imageToPlane.project(ip, &okB);
            if (!okA || !okB) continue;
            worstTruthErrorMm =
                std::max(worstTruthErrorMm, std::hypot(a.x - b.x, a.y - b.y));
        }

        cfg.cameras[view].intrinsics = sv.intrinsics;
        cfg.cameras[view].homography = toCanvas * imageToPlane;
        cfg.cameras[view].homography.normalizeH();
    }

    std::cout << "  Расхождение с истинной геометрией: " << std::fixed << std::setprecision(4)
              << worstTruthErrorMm << " мм\n";
    return 0;
}

int calibrateFromFile(const std::string& path, wz::SystemConfig& cfg) {
    wz::JsonValue root;
    std::string err;
    if (!wz::JsonValue::parseFile(path, root, &err)) {
        std::cerr << "Не удалось прочитать " << path << ": " << err << "\n";
        return 1;
    }

    double plateW = 400.0, plateH = 300.0;
    if (const wz::JsonValue* plate = root.find("plate")) {
        plateW = plate->numberAt("widthMm", plateW);
        plateH = plate->numberAt("heightMm", plateH);
    }
    if (plateW < 1.0 || plateH < 1.0) {
        std::cerr << "Размеры стола должны быть положительными\n";
        return 1;
    }

    const wz::JsonValue* cams = root.find("cameras");
    if (!cams || !cams->isArray() || cams->items().empty()) {
        std::cerr << "В файле нет массива cameras\n";
        return 1;
    }

    const wz::Mat3 toCanvas = wz::planeToCanvas(cfg.stitch, plateW, plateH);

    cfg.sources.clear();
    cfg.cameras.clear();

    std::cout << "Калибровка по файлу " << path << " (стол " << plateW << "x" << plateH << " мм)\n";

    for (const wz::JsonValue& cam : cams->items()) {
        const std::string id = cam.stringAt("id", "cam" + std::to_string(cfg.cameras.size()));
        const int imgW = static_cast<int>(cam.numberAt("imageWidth", 640));
        const int imgH = static_cast<int>(cam.numberAt("imageHeight", 480));
        const wz::Intrinsics intrinsics = intrinsicsFrom(cam.find("intrinsics"));

        const wz::JsonValue* pts = cam.find("points");
        if (!pts || !pts->isArray() || pts->items().size() < 4) {
            std::cerr << "Камера '" << id << "': нужно не менее четырёх точек\n";
            return 1;
        }

        std::vector<wz::Vec2> imagePoints, planePoints;
        for (const wz::JsonValue& p : pts->items()) {
            wz::Vec2 img, plane;
            if (!pairFrom(p.find("image"), img) || !pairFrom(p.find("plane"), plane)) {
                std::cerr << "Камера '" << id << "': точка должна содержать image[2] и plane[2]\n";
                return 1;
            }
            // Оператор указывает точки по сырому кадру - снимаем дисторсию.
            imagePoints.push_back(intrinsics.valid() ? intrinsics.rawPixelToUndistorted(img) : img);
            planePoints.push_back(plane);
        }

        wz::Mat3 imageToPlane;
        if (!wz::estimateHomography(imagePoints, planePoints, imageToPlane)) {
            std::cerr << "Камера '" << id
                      << "': не удалось оценить гомографию. Проверьте, что точки не лежат "
                         "на одной прямой.\n";
            return 1;
        }

        report(id, wz::reprojectionError(imageToPlane, imagePoints, planePoints),
               static_cast<int>(imagePoints.size()));

        wz::SourceConfig src;
        src.id = id;
        src.width = imgW;
        src.height = imgH;
        if (const wz::JsonValue* s = cam.find("source")) {
            src.type = s->stringAt("type", "jpegdir");
            src.path = s->stringAt("path", "");
            src.fps = s->numberAt("fps", 30.0);
            src.loop = s->boolAt("loop", true);
            src.startOffsetUs = static_cast<int64_t>(s->numberAt("startOffsetUs", 0));
        }
        cfg.sources.push_back(src);

        wz::CameraPlacement placement;
        placement.sourceId = id;
        placement.imageWidth = imgW;
        placement.imageHeight = imgH;
        placement.intrinsics = intrinsics;
        placement.homography = toCanvas * imageToPlane;
        placement.homography.normalizeH();
        cfg.cameras.push_back(placement);
    }

    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::string pointsPath;
    std::string outPath = "config/system.json";
    bool fromScene = false;

    wz::SystemConfig cfg = wz::SystemConfig::defaults();

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasNext = i + 1 < argc;

        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else if (arg == "--points" && hasNext) {
            pointsPath = argv[++i];
        } else if (arg == "--out" && hasNext) {
            outPath = argv[++i];
        } else if (arg == "--from-scene") {
            fromScene = true;
        } else if (arg == "--canvas" && hasNext) {
            int w = 0, h = 0;
            const std::string value = argv[++i];
            if (!parseSize(value, w, h)) {
                std::cerr << "Не удалось разобрать размер холста: " << value << "\n";
                return 1;
            }
            cfg.stitch.canvasWidth = w;
            cfg.stitch.canvasHeight = h;
        } else {
            std::cerr << "Неизвестный аргумент: " << arg << "\n\n";
            printUsage();
            return 1;
        }
    }

    if (pointsPath.empty() && !fromScene) {
        std::cerr << "Укажите --points <файл.json> или --from-scene\n\n";
        printUsage();
        return 1;
    }

    const int rc = fromScene ? calibrateFromScene(cfg) : calibrateFromFile(pointsPath, cfg);
    if (rc != 0) return rc;

    std::string err;
    if (!cfg.validate(&err)) {
        std::cerr << "Полученная конфигурация некорректна: " << err << "\n";
        return 1;
    }
    if (!cfg.save(outPath, &err)) {
        std::cerr << "Не удалось сохранить конфигурацию: " << err << "\n";
        return 1;
    }

    std::cout << "Конфигурация записана: " << outPath << "\nЗапуск: workzone_server --config "
              << outPath << "\n";
    return 0;
}
