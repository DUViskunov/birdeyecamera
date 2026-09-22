// Проецирование кадров на общую плоскость и склейка в единую панораму.
#pragma once

#include <string>
#include <vector>

#include "wz/calibration.hpp"
#include "wz/frame.hpp"
#include "wz/geometry.hpp"

namespace wz {

// Положение камеры относительно рабочей плоскости.
struct CameraPlacement {
    std::string sourceId;
    int imageWidth = 640;
    int imageHeight = 480;
    Intrinsics intrinsics;  // недействительные параметры = кадр берётся как есть
    // Гомография из исправленного кадра камеры в холст "вид сверху".
    Mat3 homography = Mat3::identity();
};

struct StitchConfig {
    int canvasWidth = 1280;
    int canvasHeight = 720;
    // Ширина зоны растушёвки у краёв кадра. 0 отключает смешивание,
    // и шов становится виден - удобно при отладке геометрии.
    float featherPx = 48.0f;
    // Выравнивание яркости камер по зоне перекрытия.
    bool exposureCompensation = true;

    // Отбраковка перекрытых участков. Голова лазера находится над плоскостью
    // стола и потому нарушает допущение гомографии: в зоне перекрытия камеры
    // резко расходятся. Там, где расхождение больше порога, берётся более
    // светлый отсчёт - тот, где стол виден, а не закрыт тёмной головой.
    // Ради этого в отчёте и ставится вторая камера.
    bool occlusionRejection = true;
    float occlusionThreshold = 30.0f;  // единицы яркости, 0..255

    // Число потоков склейки. 0 - по числу ядер. Холст режется на полосы,
    // которые не пересекаются по памяти, поэтому синхронизация не нужна.
    int threads = 0;
    RGB background{16.f, 18.f, 22.f};
};

struct StitchStats {
    double coverage = 0.0;   // доля холста, покрытая хотя бы одной камерой
    double overlap = 0.0;    // доля холста, видимая более чем одной камерой
    std::vector<double> gains;  // подобранные коэффициенты яркости
};

// Таблицы переноса строятся один раз в configure(), поэтому склейка каждого
// кадра сводится к выборке и взвешенному суммированию.
class Stitcher {
public:
    bool configure(const StitchConfig& cfg, const std::vector<CameraPlacement>& cameras,
                   std::string* error);

    // frames должны идти в том же порядке, что и cameras в configure().
    bool blend(const std::vector<Frame>& frames, Image& out);

    bool configured() const { return !tables_.empty(); }
    const StitchConfig& config() const { return cfg_; }
    const std::vector<CameraPlacement>& cameras() const { return cameras_; }
    StitchStats stats() const { return stats_; }

    // Карта покрытия: сколько камер видит каждый пиксель холста. Для диагностики.
    Image coverageMap() const;

private:
    // Один пиксель холста, видимый одной камерой.
    struct WarpEntry {
        int32_t dstIndex;
        float srcX;
        float srcY;
        float weight;
    };

    StitchConfig cfg_;
    std::vector<CameraPlacement> cameras_;
    std::vector<std::vector<WarpEntry>> tables_;
    std::vector<uint8_t> coverage_;  // число камер на пиксель холста
    StitchStats stats_;

    std::vector<float> accumR_, accumG_, accumB_, accumW_;
    // Отсчёты кешируются между проходами отбраковки: повторная выборка
    // каждого пикселя обошлась бы дороже памяти.
    std::vector<std::vector<float>> samples_;
    std::vector<float> bestLuma_;
};

}  // namespace wz
