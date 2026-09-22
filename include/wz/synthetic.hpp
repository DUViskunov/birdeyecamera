// Виртуальная сцена рабочей зоны: две камеры смотрят на стол наплавки.
// Даёт воспроизводимые данные с известной истинной геометрией, поэтому
// склейку и калибровку можно проверять без железа.
#pragma once

#include <vector>

#include "wz/calibration.hpp"
#include "wz/geometry.hpp"
#include "wz/image.hpp"

namespace wz {

struct SceneGeometry {
    double plateWidthMm = 400.0;
    double plateHeightMm = 300.0;
    int viewCount = 2;
};

// Всё, что нужно знать о виртуальной камере: параметры и истинная гомография.
struct SyntheticView {
    int view = 0;
    Intrinsics intrinsics;
    int width = 640;
    int height = 480;
    // Плоскость стола (мм) -> пиксель идеального кадра.
    Mat3 planeToImage = Mat3::identity();
    // Обратное преобразование - им пользуется склейка.
    Mat3 imageToPlane = Mat3::identity();
    // Коэффициент яркости камеры: камеры намеренно расходятся по экспозиции.
    double exposureGain = 1.0;
};

SceneGeometry sceneGeometry();

// Параметры вида. view: 0 - левая камера, 1 - правая.
SyntheticView syntheticView(int view, int width, int height);

// Отрисовка кадра камеры на момент времени tSeconds.
// Голова лазера перекрывает часть стола - в каждом виде свою.
void renderSyntheticFrame(const SyntheticView& v, double tSeconds, Image& out);

// Контрольные метки на столе (мм) и их проекции в кадр - вход для калибровки.
std::vector<Vec2> sceneMarkersMm();
std::vector<Vec2> projectMarkers(const SyntheticView& v);

}  // namespace wz
