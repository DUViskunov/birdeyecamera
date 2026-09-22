#include "wz/calibration.hpp"

#include <cmath>

namespace wz {

bool Intrinsics::hasDistortion() const {
    return std::fabs(k1) > 1e-12 || std::fabs(k2) > 1e-12 || std::fabs(k3) > 1e-12 ||
           std::fabs(p1) > 1e-12 || std::fabs(p2) > 1e-12;
}

Mat3 Intrinsics::K() const {
    Mat3 k = Mat3::identity();
    k(0, 0) = fx;
    k(1, 1) = fy;
    k(0, 2) = cx;
    k(1, 2) = cy;
    return k;
}

Vec2 Intrinsics::distortNormalized(const Vec2& n) const {
    const double r2 = n.x * n.x + n.y * n.y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    // Тангенциальная часть модели Брауна-Конради: учитывает неперпендикулярность
    // матрицы и оптической оси.
    const double dx = 2.0 * p1 * n.x * n.y + p2 * (r2 + 2.0 * n.x * n.x);
    const double dy = p1 * (r2 + 2.0 * n.y * n.y) + 2.0 * p2 * n.x * n.y;

    return Vec2{n.x * radial + dx, n.y * radial + dy};
}

Vec2 Intrinsics::undistortedPixelToRaw(const Vec2& p) const {
    if (!valid()) return p;
    const Vec2 n{(p.x - cx) / fx, (p.y - cy) / fy};
    const Vec2 d = distortNormalized(n);
    return Vec2{fx * d.x + cx, fy * d.y + cy};
}

Vec2 Intrinsics::rawPixelToUndistorted(const Vec2& p, int iterations) const {
    if (!valid() || !hasDistortion()) return p;

    const Vec2 target{(p.x - cx) / fx, (p.y - cy) / fy};
    Vec2 guess = target;
    for (int i = 0; i < iterations; ++i) {
        const Vec2 d = distortNormalized(guess);
        guess.x += target.x - d.x;
        guess.y += target.y - d.y;
    }
    return Vec2{fx * guess.x + cx, fy * guess.y + cy};
}

Intrinsics Intrinsics::fromFov(int w, int h, double horizontalFovDeg) {
    Intrinsics in;
    if (w <= 0 || h <= 0) return in;

    const double fovRad = horizontalFovDeg * 3.14159265358979323846 / 180.0;
    const double halfTan = std::tan(fovRad * 0.5);
    if (halfTan < 1e-9) return in;

    in.fx = (w * 0.5) / halfTan;
    in.fy = in.fx;  // квадратный пиксель - верно для подавляющего большинства матриц
    in.cx = w * 0.5;
    in.cy = h * 0.5;
    return in;
}

void UndistortMap::build(const Intrinsics& in, int width, int height) {
    w_ = 0;
    h_ = 0;
    mapX_.clear();
    mapY_.clear();
    if (width <= 0 || height <= 0 || !in.valid()) return;

    w_ = width;
    h_ = height;
    mapX_.resize(static_cast<size_t>(width) * height);
    mapY_.resize(static_cast<size_t>(width) * height);

    // Обратное отображение: идём по пикселям исправленного кадра и ищем,
    // откуда их взять в исходном. Прямое отображение оставило бы дыры.
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const Vec2 raw = in.undistortedPixelToRaw(Vec2{static_cast<double>(u),
                                                           static_cast<double>(v)});
            const size_t idx = static_cast<size_t>(v) * width + u;
            mapX_[idx] = static_cast<float>(raw.x);
            mapY_[idx] = static_cast<float>(raw.y);
        }
    }
}

void UndistortMap::apply(const Image& src, Image& dst) const {
    if (!valid() || src.empty()) {
        dst = src;
        return;
    }
    if (dst.width() != w_ || dst.height() != h_) dst = Image(w_, h_);

    RGB c;
    for (int v = 0; v < h_; ++v) {
        for (int u = 0; u < w_; ++u) {
            const size_t idx = static_cast<size_t>(v) * w_ + u;
            if (src.sampleBilinear(mapX_[idx], mapY_[idx], c)) {
                dst.set(u, v, c);
            } else {
                // Точка пришла из-за края матрицы - чёрное поле,
                // склейка потом закроет его соседней камерой.
                dst.set(u, v, RGB{0.f, 0.f, 0.f});
            }
        }
    }
}

}  // namespace wz
