#include "wz/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "test_common.hpp"

using namespace wz;

namespace {

void testInverse() {
    Mat3 m;
    m.m = {2.0, 0.5, -3.0, 1.0, 4.0, 2.0, 0.25, -1.0, 1.5};

    Mat3 inv;
    CHECK(m.inverse(inv));

    const Mat3 id = m * inv;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            CHECK_NEAR(id(r, c), r == c ? 1.0 : 0.0, 1e-12);
        }
    }

    // Вырожденная матрица: вторая строка кратна первой.
    Mat3 singular;
    singular.m = {1, 2, 3, 2, 4, 6, 7, 8, 9};
    Mat3 unused;
    CHECK(!singular.inverse(unused));
}

void testJacobi() {
    // Проверяем определяющие свойства разложения, а не заранее выписанные
    // корни: так тест ловит настоящие ошибки и не зависит от выбора матрицы.
    const std::vector<double> original = {2, -1, 0, -1, 3, -1, 0, -1, 3};

    std::vector<double> work = original;
    std::vector<double> values, vectors;
    CHECK(jacobiEigenSymmetric(work, 3, values, vectors));
    CHECK(values.size() == 3);

    // A * v = lambda * v для каждой пары.
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            double av = 0.0;
            for (int k = 0; k < 3; ++k) {
                av += original[static_cast<size_t>(row) * 3 + k] *
                      vectors[static_cast<size_t>(k) * 3 + col];
            }
            const double lv = values[static_cast<size_t>(col)] *
                              vectors[static_cast<size_t>(row) * 3 + col];
            CHECK_NEAR(av, lv, 1e-10);
        }
    }

    // Сумма собственных значений равна следу, произведение - определителю.
    const double trace = original[0] + original[4] + original[8];
    double sum = 0.0, product = 1.0;
    for (double v : values) {
        sum += v;
        product *= v;
    }
    CHECK_NEAR(sum, trace, 1e-10);
    CHECK_NEAR(product, 13.0, 1e-9);  // det([2,-1,0; -1,3,-1; 0,-1,3]) = 13

    // Собственные векторы симметричной матрицы образуют ортонормированный базис.
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            double dot = 0.0;
            for (int row = 0; row < 3; ++row) {
                dot += vectors[static_cast<size_t>(row) * 3 + a] *
                       vectors[static_cast<size_t>(row) * 3 + b];
            }
            CHECK_NEAR(dot, a == b ? 1.0 : 0.0, 1e-10);
        }
    }
}

void testHomography() {
    // Известная гомография: перспективное искажение квадрата.
    Mat3 truth;
    truth.m = {1.2, 0.15, 30.0, -0.08, 1.05, -12.0, 0.0004, -0.0002, 1.0};

    const std::vector<Vec2> src = {{0, 0},     {640, 0},   {640, 480},
                                   {0, 480},   {320, 240}, {120, 400}};
    std::vector<Vec2> dst;
    for (const Vec2& p : src) dst.push_back(truth.project(p));

    Mat3 estimated;
    CHECK(estimateHomography(src, dst, estimated));

    const ReprojectionError e = reprojectionError(estimated, src, dst);
    CHECK_NEAR(e.mean, 0.0, 1e-6);
    CHECK_NEAR(e.max, 0.0, 1e-6);

    // После нормировки матрицы должны совпасть поэлементно.
    Mat3 normalizedTruth = truth;
    normalizedTruth.normalizeH();
    for (size_t i = 0; i < 9; ++i) {
        CHECK_NEAR(estimated.m[i], normalizedTruth.m[i], 1e-8);
    }
}

void testHomographyRejectsBadInput() {
    Mat3 h;
    // Меньше четырёх точек.
    CHECK(!estimateHomography({{0, 0}, {1, 0}, {0, 1}}, {{0, 0}, {1, 0}, {0, 1}}, h));
    // Разное число точек.
    CHECK(!estimateHomography({{0, 0}, {1, 0}, {0, 1}, {1, 1}}, {{0, 0}, {1, 0}}, h));
    // Все точки в одной позиции - нормализация невозможна.
    const std::vector<Vec2> same(5, Vec2{7.0, 7.0});
    CHECK(!estimateHomography(same, same, h));
}

void testHomographyWithNoise() {
    // При небольшом шуме в координатах оценка должна оставаться разумной:
    // это моделирует ручное указание меток оператором.
    Mat3 truth;
    truth.m = {1.1, 0.05, 20.0, -0.04, 1.02, -8.0, 0.0003, -0.0001, 1.0};

    const std::vector<Vec2> src = {{20, 20},   {600, 30},  {610, 450},
                                   {30, 440},  {320, 240}, {150, 120}};
    std::vector<Vec2> dst;
    const double noise[6] = {0.4, -0.3, 0.25, -0.45, 0.2, -0.2};
    for (size_t i = 0; i < src.size(); ++i) {
        Vec2 p = truth.project(src[i]);
        p.x += noise[i];
        p.y -= noise[i];
        dst.push_back(p);
    }

    Mat3 estimated;
    CHECK(estimateHomography(src, dst, estimated));
    const ReprojectionError e = reprojectionError(estimated, src, dst);
    CHECK(e.max < 1.0);
}

}  // namespace

int main() {
    testInverse();
    testJacobi();
    testHomography();
    testHomographyRejectsBadInput();
    testHomographyWithNoise();
    return wztest::report("geometry");
}
