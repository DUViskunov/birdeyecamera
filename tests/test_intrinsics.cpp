#include <cmath>
#include <cstdio>
#include <vector>

#include "test_common.hpp"
#include "wz/chessboard.hpp"
#include "wz/intrinsics_calib.hpp"

using namespace wz;

namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 480;

// Параметры, которые алгоритм обязан восстановить, не зная их.
Intrinsics truthIntrinsics() {
    Intrinsics in;
    in.fx = 470.0;
    in.fy = 470.0;
    in.cx = 322.0;
    in.cy = 238.0;
    in.k1 = -0.130;
    in.k2 = 0.035;
    return in;
}

ChessboardSpec spec() {
    ChessboardSpec s;
    s.cols = 9;
    s.rows = 6;
    s.squareMm = 25.0;
    return s;
}

// Истинные положения углов в кадре: проекция плюс дисторсия.
std::vector<Vec2> trueCorners(const Mat3& boardToImage, const ChessboardSpec& s,
                              const Intrinsics& in) {
    std::vector<Vec2> out;
    for (int row = 0; row < s.rows; ++row) {
        for (int col = 0; col < s.cols; ++col) {
            bool ok = false;
            const Vec2 ideal =
                boardToImage.project(Vec2{col * s.squareMm, row * s.squareMm}, &ok);
            out.push_back(ok ? in.undistortedPixelToRaw(ideal) : Vec2{-1.0, -1.0});
        }
    }
    return out;
}

// Найденный набор может быть развёрнут на 180 градусов: какая вершина рамки
// отвечает углу (0,0), из одной картинки не определить. Для калибровки это
// безразлично, поэтому сверяемся с лучшим из двух вариантов.
double cornerError(const std::vector<Vec2>& found, const std::vector<Vec2>& truth) {
    const size_t n = found.size();
    if (n != truth.size() || n == 0) return 1e9;

    double direct = 0.0, flipped = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec2& a = truth[i];
        const Vec2& b = truth[n - 1 - i];
        direct = std::max(direct, std::hypot(found[i].x - a.x, found[i].y - a.y));
        flipped = std::max(flipped, std::hypot(found[i].x - b.x, found[i].y - b.y));
    }
    return std::min(direct, flipped);
}

void testDetection() {
    const Intrinsics truth = truthIntrinsics();
    const ChessboardSpec s = spec();

    Image img(kWidth, kHeight);
    const Mat3 boardToImage = renderChessboardView(5, s, truth, img);

    ChessboardCorners corners;
    std::string err;
    const bool ok = findChessboard(img, s, corners, &err);
    if (!ok) std::printf("    поиск не удался: %s\n", err.c_str());
    CHECK(ok);
    if (!ok) return;

    CHECK(corners.image.size() == static_cast<size_t>(s.cornerCount()));

    const double maxErr = cornerError(corners.image, trueCorners(boardToImage, s, truth));
    std::printf("    максимальная ошибка угла: %.3f пикселя\n", maxErr);

    // Субпиксельное уточнение должно давать заметно лучше пикселя.
    CHECK(maxErr < 1.0);
}

void testDetectionRejectsBlankImage() {
    Image blank(kWidth, kHeight);
    blank.fill(RGB{128.f, 128.f, 128.f});

    ChessboardCorners corners;
    std::string err;
    CHECK(!findChessboard(blank, spec(), corners, &err));
    CHECK(!err.empty());
}

void testCalibrationRecoversIntrinsics() {
    const Intrinsics truth = truthIntrinsics();
    const ChessboardSpec s = spec();

    std::vector<ChessboardCorners> views;
    const int rendered = 12;

    for (int view = 0; view < rendered; ++view) {
        Image img(kWidth, kHeight);
        renderChessboardView(view, s, truth, img);

        ChessboardCorners corners;
        std::string err;
        if (!findChessboard(img, s, corners, &err)) continue;
        corners.source = "view" + std::to_string(view);
        views.push_back(std::move(corners));
    }

    std::printf("    доска найдена на %zu видах из %d\n", views.size(), rendered);
    CHECK(views.size() >= 6);
    if (views.size() < 3) return;

    IntrinsicsCalibResult result;
    std::string err;
    const bool ok = calibrateIntrinsics(views, kWidth, kHeight, result, &err);
    if (!ok) std::printf("    калибровка не удалась: %s\n", err.c_str());
    CHECK(ok);
    if (!ok) return;

    const Intrinsics& got = result.intrinsics;
    std::printf("    fx %.1f (истина %.1f)   fy %.1f (%.1f)\n", got.fx, truth.fx, got.fy,
                truth.fy);
    std::printf("    cx %.1f (истина %.1f)   cy %.1f (%.1f)\n", got.cx, truth.cx, got.cy,
                truth.cy);
    std::printf("    k1 %.4f (истина %.4f)   k2 %.4f (%.4f)\n", got.k1, truth.k1, got.k2,
                truth.k2);
    std::printf("    невязка: RMS %.3f, максимум %.3f пикселя\n", result.rmsPx, result.maxPx);

    // Фокусное и дисторсия связаны: при одинаковой невязке решение может
    // отдать несколько процентов fx в пользу k1 и наоборот. Поэтому здесь
    // проверяется лишь отсутствие грубого промаха, а точность решения -
    // по отображению ниже, именно оно и используется в проекте.
    CHECK(std::fabs(got.fx - truth.fx) / truth.fx < 0.10);
    CHECK(std::fabs(got.fy - truth.fy) / truth.fy < 0.10);

    // Центр определяется хуже фокуса: он слабее влияет на невязку.
    CHECK(std::fabs(got.cx - truth.cx) < 12.0);
    CHECK(std::fabs(got.cy - truth.cy) < 12.0);

    // Главная проверка. Внутренние параметры нужны проекту ровно для одного:
    // пересчёта "идеальный пиксель <-> сырой". Поэтому мерится расхождение
    // самого отображения по всему кадру, а не отдельные коэффициенты,
    // которые вырождены и компенсируют друг друга.
    double worstShift = 0.0;
    for (int y = 0; y <= kHeight; y += 16) {
        for (int x = 0; x <= kWidth; x += 16) {
            const Vec2 p{static_cast<double>(x), static_cast<double>(y)};
            const Vec2 a = truth.undistortedPixelToRaw(p);
            const Vec2 b = got.undistortedPixelToRaw(p);
            worstShift = std::max(worstShift, std::hypot(a.x - b.x, a.y - b.y));
        }
    }
    std::printf("    расхождение модели дисторсии по кадру: %.2f пикселя\n", worstShift);
    CHECK(worstShift < 3.0);

    CHECK(result.rmsPx < 1.0);
    CHECK(result.views >= 6);
    CHECK(result.views <= static_cast<int>(views.size()));
    CHECK(result.points == result.views * s.cornerCount());
}

void testCalibrationRejectsTooFewViews() {
    std::vector<ChessboardCorners> views(2);
    for (ChessboardCorners& c : views) {
        c.image.assign(9, Vec2{});
        c.board.assign(9, Vec2{});
    }

    IntrinsicsCalibResult result;
    std::string err;
    CHECK(!calibrateIntrinsics(views, kWidth, kHeight, result, &err));
    CHECK(!err.empty());
}

}  // namespace

int main() {
    testDetection();
    testDetectionRejectsBlankImage();
    testCalibrationRecoversIntrinsics();
    testCalibrationRejectsTooFewViews();
    return wztest::report("intrinsics");
}
