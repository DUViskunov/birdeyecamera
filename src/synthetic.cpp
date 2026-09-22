#include "wz/synthetic.hpp"

#include <algorithm>
#include <cmath>

namespace wz {
namespace {

struct Vec3d {
    double x, y, z;
};

Vec3d sub(const Vec3d& a, const Vec3d& b) { return Vec3d{a.x - b.x, a.y - b.y, a.z - b.z}; }

Vec3d cross(const Vec3d& a, const Vec3d& b) {
    return Vec3d{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3d normalize(const Vec3d& v) {
    const double n = std::sqrt(dot(v, v));
    if (n < 1e-12) return Vec3d{0, 0, 1};
    return Vec3d{v.x / n, v.y / n, v.z / n};
}

// Положение камер в мм относительно угла стола. Камеры разведены по краям,
// чтобы их поля зрения перекрывались в середине рабочей зоны.
Vec3d cameraCenter(int view) {
    const SceneGeometry g = sceneGeometry();
    const double cx = g.plateWidthMm * 0.5;
    const double cy = g.plateHeightMm * 0.5;
    if (view == 0) return Vec3d{cx - 230.0, cy - 190.0, 430.0};
    return Vec3d{cx + 230.0, cy - 170.0, 455.0};
}

Vec3d cameraTarget(int view) {
    const SceneGeometry g = sceneGeometry();
    const double cx = g.plateWidthMm * 0.5;
    const double cy = g.plateHeightMm * 0.5;
    // Камеры смотрят не строго в центр: так перекрытие получается несимметричным,
    // как и на реальной установке.
    if (view == 0) return Vec3d{cx - 40.0, cy + 10.0, 0.0};
    return Vec3d{cx + 45.0, cy - 5.0, 0.0};
}

// Матрица поворота "мир -> камера" по правилу смотрящей камеры
// (x вправо, y вниз, z вперёд). Строки - базис камеры в мировых осях.
void lookAtRotation(int view, Vec3d rows[3]) {
    const Vec3d c = cameraCenter(view);
    const Vec3d t = cameraTarget(view);
    const Vec3d forward = normalize(sub(t, c));
    const Vec3d worldUp{0.0, 0.0, 1.0};
    const Vec3d right = normalize(cross(forward, worldUp));
    const Vec3d up = cross(right, forward);

    rows[0] = right;
    rows[1] = Vec3d{-up.x, -up.y, -up.z};  // ось y экрана направлена вниз
    rows[2] = forward;
}

double smoothstep(double edge0, double edge1, double x) {
    if (edge1 - edge0 < 1e-12) return x < edge0 ? 0.0 : 1.0;
    double t = (x - edge0) / (edge1 - edge0);
    t = std::min(1.0, std::max(0.0, t));
    return t * t * (3.0 - 2.0 * t);
}

double hashNoise(double x, double y) {
    const double s = std::sin(x * 12.9898 + y * 78.233) * 43758.5453;
    return s - std::floor(s);
}

// Траектория головы: растровый проход по столу, как при послойной наплавке.
Vec2 headPosition(double tSeconds) {
    const SceneGeometry g = sceneGeometry();
    const double marginX = 70.0;
    const double marginY = 70.0;
    const double spanX = g.plateWidthMm - 2.0 * marginX;
    const double spanY = g.plateHeightMm - 2.0 * marginY;

    const double passDuration = 4.0;  // секунд на один проход вдоль X
    const double passes = 6.0;
    const double raw = tSeconds / passDuration;
    const int pass = static_cast<int>(std::floor(raw));

    double u = raw - std::floor(raw);
    if (pass % 2 == 1) u = 1.0 - u;

    const double v = static_cast<double>(pass % static_cast<int>(passes)) / (passes - 1.0);
    return Vec2{marginX + u * spanX, marginY + v * spanY};
}

// Пройденный участок дорожки: точка считается наплавленной, если голова
// уже проходила достаточно близко к ней.
double beadCoverage(const Vec2& p, double tSeconds) {
    double best = 0.0;
    const double step = 0.08;
    for (double t = 0.0; t <= tSeconds; t += step) {
        const Vec2 h = headPosition(t);
        const double dx = p.x - h.x;
        const double dy = p.y - h.y;
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d < 9.0) {
            best = std::max(best, 1.0 - smoothstep(4.0, 9.0, d));
            if (best > 0.99) break;
        }
    }
    return best;
}

// Цвет точки стола. Всё процедурно: сетка, рамка, метки, наплавленная дорожка
// и раскалённая ванна под головой.
RGB plateColor(const Vec2& mm, double tSeconds) {
    const SceneGeometry g = sceneGeometry();

    if (mm.x < 0.0 || mm.y < 0.0 || mm.x > g.plateWidthMm || mm.y > g.plateHeightMm) {
        return RGB{10.f, 11.f, 14.f};  // пол цеха за пределами стола
    }

    // Шлифованная сталь с лёгким зерном.
    const double grain = hashNoise(std::floor(mm.x * 2.0), std::floor(mm.y * 2.0));
    double base = 88.0 + grain * 12.0;

    // Разметочная сетка через 50 мм.
    const double gx = std::fabs(std::fmod(mm.x, 50.0) - 25.0);
    const double gy = std::fabs(std::fmod(mm.y, 50.0) - 25.0);
    if (gx > 24.2 || gy > 24.2) base += 34.0;

    // Рамка стола.
    const double edge = std::min(std::min(mm.x, mm.y),
                                 std::min(g.plateWidthMm - mm.x, g.plateHeightMm - mm.y));
    if (edge < 6.0) base = 52.0;

    RGB c{static_cast<float>(base), static_cast<float>(base * 1.02),
          static_cast<float>(base * 1.08)};

    // Контрольные метки - опорные точки для калибровки.
    for (const Vec2& m : sceneMarkersMm()) {
        const double dx = mm.x - m.x;
        const double dy = mm.y - m.y;
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d < 7.0) {
            const double k = 1.0 - smoothstep(5.0, 7.0, d);
            c.r = static_cast<float>(c.r * (1.0 - k) + 235.0 * k);
            c.g = static_cast<float>(c.g * (1.0 - k) + 230.0 * k);
            c.b = static_cast<float>(c.b * (1.0 - k) + 215.0 * k);
        }
    }

    // Наплавленная дорожка: тёмный окисленный валик.
    const double bead = beadCoverage(mm, tSeconds);
    if (bead > 0.01) {
        c.r = static_cast<float>(c.r * (1.0 - bead) + 120.0 * bead);
        c.g = static_cast<float>(c.g * (1.0 - bead) + 74.0 * bead);
        c.b = static_cast<float>(c.b * (1.0 - bead) + 48.0 * bead);
    }

    // Сварочная ванна и её свечение.
    const Vec2 head = headPosition(tSeconds);
    const double dhx = mm.x - head.x;
    const double dhy = mm.y - head.y;
    const double dh = std::sqrt(dhx * dhx + dhy * dhy);
    if (dh < 45.0) {
        const double glow = 1.0 - smoothstep(6.0, 45.0, dh);
        const double core = 1.0 - smoothstep(0.0, 7.0, dh);
        c.r = static_cast<float>(std::min(255.0, c.r + 190.0 * glow + 60.0 * core));
        c.g = static_cast<float>(std::min(255.0, c.g + 110.0 * glow + 55.0 * core));
        c.b = static_cast<float>(std::min(255.0, c.b + 30.0 * glow + 45.0 * core));
    }

    return c;
}

}  // namespace

SceneGeometry sceneGeometry() { return SceneGeometry{}; }

std::vector<Vec2> sceneMarkersMm() {
    const SceneGeometry g = sceneGeometry();
    // Шесть меток: четыре по углам рабочего поля и две в середине,
    // чтобы гомография определялась устойчиво.
    return {Vec2{60.0, 50.0},
            Vec2{g.plateWidthMm - 60.0, 50.0},
            Vec2{g.plateWidthMm - 60.0, g.plateHeightMm - 50.0},
            Vec2{60.0, g.plateHeightMm - 50.0},
            Vec2{g.plateWidthMm * 0.5, 40.0},
            Vec2{g.plateWidthMm * 0.5, g.plateHeightMm - 40.0}};
}

SyntheticView syntheticView(int view, int width, int height) {
    SyntheticView v;
    v.view = view;
    v.width = width;
    v.height = height;

    // Широкоугольная камера с заметной бочкой - та самая дисторсия,
    // которую отчёт называет недостатком широкоугольных решений.
    v.intrinsics = Intrinsics::fromFov(width, height, view == 0 ? 74.0 : 70.0);
    v.intrinsics.k1 = view == 0 ? -0.145 : -0.118;
    v.intrinsics.k2 = view == 0 ? 0.031 : 0.024;
    v.intrinsics.p1 = 0.0009;
    v.intrinsics.p2 = -0.0006;
    v.exposureGain = view == 0 ? 1.0 : 0.87;

    Vec3d rows[3];
    lookAtRotation(view, rows);
    const Vec3d c = cameraCenter(view);

    // t = -R * C
    const Vec3d t{-dot(rows[0], c), -dot(rows[1], c), -dot(rows[2], c)};

    // Для плоскости z = 0: H = K * [r1 r2 t], где r1, r2 - первые два столбца R.
    Mat3 rt;
    rt(0, 0) = rows[0].x;
    rt(1, 0) = rows[1].x;
    rt(2, 0) = rows[2].x;
    rt(0, 1) = rows[0].y;
    rt(1, 1) = rows[1].y;
    rt(2, 1) = rows[2].y;
    rt(0, 2) = t.x;
    rt(1, 2) = t.y;
    rt(2, 2) = t.z;

    v.planeToImage = v.intrinsics.K() * rt;
    v.planeToImage.normalizeH();
    if (!v.planeToImage.inverse(v.imageToPlane)) {
        v.imageToPlane = Mat3::identity();
    }
    return v;
}

std::vector<Vec2> projectMarkers(const SyntheticView& v) {
    std::vector<Vec2> out;
    for (const Vec2& m : sceneMarkersMm()) {
        bool ok = false;
        const Vec2 p = v.planeToImage.project(m, &ok);
        out.push_back(ok ? p : Vec2{-1.0, -1.0});
    }
    return out;
}

void renderSyntheticFrame(const SyntheticView& v, double tSeconds, Image& out) {
    if (out.width() != v.width || out.height() != v.height) out = Image(v.width, v.height);

    // Голова лазера - вертикальный столб над текущей точкой наплавки. В каждой
    // камере она закрывает свой участок стола: это и есть проблема, ради которой
    // в отчёте ставится вторая камера.
    struct HeadDisc {
        Vec2 centre;
        double radiusPx;
    };
    std::vector<HeadDisc> discs;

    const Vec2 head = headPosition(tSeconds);
    Vec3d rows[3];
    lookAtRotation(v.view, rows);
    const Vec3d centre = cameraCenter(v.view);

    for (double z = 8.0; z <= 150.0; z += 6.0) {
        const Vec3d rel = sub(Vec3d{head.x, head.y, z}, centre);
        const Vec3d cam{dot(rows[0], rel), dot(rows[1], rel), dot(rows[2], rel)};
        if (cam.z < 1.0) continue;

        const Vec2 ideal{v.intrinsics.fx * cam.x / cam.z + v.intrinsics.cx,
                         v.intrinsics.fy * cam.y / cam.z + v.intrinsics.cy};
        const double radiusMm = z < 40.0 ? 14.0 : 26.0;
        discs.push_back(HeadDisc{v.intrinsics.undistortedPixelToRaw(ideal),
                                 v.intrinsics.fx * radiusMm / cam.z});
    }

    for (int y = 0; y < v.height; ++y) {
        for (int x = 0; x < v.width; ++x) {
            const Vec2 raw{static_cast<double>(x), static_cast<double>(y)};

            // Сначала снимаем дисторсию, затем переходим на плоскость стола.
            const Vec2 ideal = v.intrinsics.rawPixelToUndistorted(raw);
            bool ok = false;
            const Vec2 mm = v.imageToPlane.project(ideal, &ok);

            RGB c = ok ? plateColor(mm, tSeconds) : RGB{10.f, 11.f, 14.f};

            // Виньетирование и разная экспозиция камер: компенсатору яркости
            // в склейке должно быть что выравнивать.
            const double dx = (x - v.intrinsics.cx) / v.intrinsics.cx;
            const double dy = (y - v.intrinsics.cy) / v.intrinsics.cy;
            const double gain = v.exposureGain * (1.0 - 0.18 * (dx * dx + dy * dy));
            c.r = static_cast<float>(c.r * gain);
            c.g = static_cast<float>(c.g * gain);
            c.b = static_cast<float>(c.b * gain);

            for (const HeadDisc& d : discs) {
                const double ddx = raw.x - d.centre.x;
                const double ddy = raw.y - d.centre.y;
                if (ddx * ddx + ddy * ddy < d.radiusPx * d.radiusPx) {
                    c = RGB{38.f, 40.f, 46.f};
                    break;
                }
            }

            out.set(x, y, c);
        }
    }
}

}  // namespace wz
