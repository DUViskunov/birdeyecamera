// Внутренние параметры камеры и устранение дисторсии.
#pragma once

#include <string>
#include <vector>

#include "wz/geometry.hpp"
#include "wz/image.hpp"

namespace wz {

// Модель камеры-обскуры с радиально-тангенциальной дисторсией (Брауна-Конради).
// Нулевой fx означает "калибровка не задана" - коррекция тогда не применяется.
struct Intrinsics {
    double fx = 0.0, fy = 0.0;
    double cx = 0.0, cy = 0.0;
    double k1 = 0.0, k2 = 0.0, k3 = 0.0;  // радиальные
    double p1 = 0.0, p2 = 0.0;            // тангенциальные

    bool valid() const { return fx > 1e-6 && fy > 1e-6; }
    bool hasDistortion() const;
    Mat3 K() const;

    // Прямая модель: точка на нормализованной плоскости -> пиксель искажённого кадра.
    Vec2 distortNormalized(const Vec2& n) const;

    // Пиксель идеального (исправленного) кадра -> пиксель реального кадра.
    Vec2 undistortedPixelToRaw(const Vec2& p) const;

    // Обратное преобразование. Замкнутой формы у него нет, поэтому решается
    // итерационно. Нужно там, где оператор указывает точки на сыром кадре.
    Vec2 rawPixelToUndistorted(const Vec2& p, int iterations = 8) const;

    // Разумные параметры по умолчанию для кадра w x h при заданном угле обзора.
    static Intrinsics fromFov(int w, int h, double horizontalFovDeg);
};

// Предрассчитанная карта коррекции: для каждого пикселя исправленного кадра
// хранит координаты в исходном. Строится один раз, применяется к каждому кадру.
class UndistortMap {
public:
    void build(const Intrinsics& in, int width, int height);
    bool valid() const { return w_ > 0 && h_ > 0; }
    int width() const { return w_; }
    int height() const { return h_; }

    void apply(const Image& src, Image& dst) const;

private:
    int w_ = 0, h_ = 0;
    std::vector<float> mapX_, mapY_;
};

}  // namespace wz
