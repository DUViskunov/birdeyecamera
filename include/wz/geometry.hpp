// Векторы, матрицы 3x3 и оценка гомографии.
#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace wz {

struct Vec2 {
    double x = 0.0, y = 0.0;
};

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

// Матрица 3x3 в построчном порядке: m[row * 3 + col].
struct Mat3 {
    std::array<double, 9> m{};

    static Mat3 identity();
    static Mat3 zero() { return Mat3{}; }
    static Mat3 translation(double tx, double ty);
    static Mat3 scaling(double sx, double sy);

    double& operator()(int r, int c) { return m[static_cast<size_t>(r) * 3 + c]; }
    double operator()(int r, int c) const { return m[static_cast<size_t>(r) * 3 + c]; }

    Mat3 operator*(const Mat3& o) const;
    Vec3 operator*(const Vec3& v) const;

    // Проекция точки плоскости: применяет матрицу и делит на однородную координату.
    // ok == false, если точка уходит на бесконечность (w ~ 0).
    Vec2 project(const Vec2& p, bool* ok = nullptr) const;

    Mat3 transposed() const;
    double determinant() const;

    // Обратная матрица. При вырожденности возвращает false и не трогает out.
    bool inverse(Mat3& out) const;

    // Нормировка так, чтобы m[8] == 1 (канонический вид гомографии).
    void normalizeH();
};

// Собственные значения/векторы симметричной матрицы n x n методом Якоби.
// a — вход (построчно, n*n), портится в процессе.
// eigenvalues — n значений, eigenvectors — n*n, собственный вектор i лежит
// в столбце i. Возвращает false, если не сошлось за maxSweeps проходов.
bool jacobiEigenSymmetric(std::vector<double>& a, int n, std::vector<double>& eigenvalues,
                          std::vector<double>& eigenvectors, int maxSweeps = 100);

// Оценка гомографии src -> dst нормализованным DLT.
// Нужно не менее 4 соответствий, из которых никакие три не лежат на одной прямой.
bool estimateHomography(const std::vector<Vec2>& src, const std::vector<Vec2>& dst, Mat3& out);

// Средняя и максимальная ошибка перепроецирования — метрика качества калибровки.
struct ReprojectionError {
    double mean = 0.0;
    double max = 0.0;
};
ReprojectionError reprojectionError(const Mat3& h, const std::vector<Vec2>& src,
                                    const std::vector<Vec2>& dst);

}  // namespace wz
