#include "wz/geometry.hpp"

#include <cmath>

namespace wz {

Mat3 Mat3::identity() {
    Mat3 r;
    r.m = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    return r;
}

Mat3 Mat3::translation(double tx, double ty) {
    Mat3 r = identity();
    r(0, 2) = tx;
    r(1, 2) = ty;
    return r;
}

Mat3 Mat3::scaling(double sx, double sy) {
    Mat3 r = identity();
    r(0, 0) = sx;
    r(1, 1) = sy;
    return r;
}

Mat3 Mat3::operator*(const Mat3& o) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += (*this)(i, k) * o(k, j);
            r(i, j) = s;
        }
    }
    return r;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    Vec3 r;
    r.x = m[0] * v.x + m[1] * v.y + m[2] * v.z;
    r.y = m[3] * v.x + m[4] * v.y + m[5] * v.z;
    r.z = m[6] * v.x + m[7] * v.y + m[8] * v.z;
    return r;
}

Vec2 Mat3::project(const Vec2& p, bool* ok) const {
    const double w = m[6] * p.x + m[7] * p.y + m[8];
    if (std::fabs(w) < 1e-12) {
        if (ok) *ok = false;
        return Vec2{0.0, 0.0};
    }
    if (ok) *ok = true;
    return Vec2{(m[0] * p.x + m[1] * p.y + m[2]) / w, (m[3] * p.x + m[4] * p.y + m[5]) / w};
}

Mat3 Mat3::transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r(i, j) = (*this)(j, i);
    return r;
}

double Mat3::determinant() const {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

bool Mat3::inverse(Mat3& out) const {
    const double det = determinant();
    if (std::fabs(det) < 1e-14) return false;
    const double inv = 1.0 / det;
    Mat3 r;
    r.m[0] = (m[4] * m[8] - m[5] * m[7]) * inv;
    r.m[1] = (m[2] * m[7] - m[1] * m[8]) * inv;
    r.m[2] = (m[1] * m[5] - m[2] * m[4]) * inv;
    r.m[3] = (m[5] * m[6] - m[3] * m[8]) * inv;
    r.m[4] = (m[0] * m[8] - m[2] * m[6]) * inv;
    r.m[5] = (m[2] * m[3] - m[0] * m[5]) * inv;
    r.m[6] = (m[3] * m[7] - m[4] * m[6]) * inv;
    r.m[7] = (m[1] * m[6] - m[0] * m[7]) * inv;
    r.m[8] = (m[0] * m[4] - m[1] * m[3]) * inv;
    out = r;
    return true;
}

void Mat3::normalizeH() {
    if (std::fabs(m[8]) < 1e-15) return;
    const double s = 1.0 / m[8];
    for (double& v : m) v *= s;
}

bool jacobiEigenSymmetric(std::vector<double>& a, int n, std::vector<double>& eigenvalues,
                          std::vector<double>& eigenvectors, int maxSweeps) {
    if (n <= 0 || static_cast<int>(a.size()) < n * n) return false;

    eigenvectors.assign(static_cast<size_t>(n) * n, 0.0);
    for (int i = 0; i < n; ++i) eigenvectors[static_cast<size_t>(i) * n + i] = 1.0;

    auto A = [&](int r, int c) -> double& { return a[static_cast<size_t>(r) * n + c]; };
    auto V = [&](int r, int c) -> double& { return eigenvectors[static_cast<size_t>(r) * n + c]; };

    auto storeEigenvalues = [&]() {
        eigenvalues.resize(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) eigenvalues[static_cast<size_t>(i)] = A(i, i);
    };

    for (int sweep = 0; sweep < maxSweeps; ++sweep) {
        // Сумма квадратов внедиагональных элементов - мера сходимости.
        double off = 0.0;
        for (int p = 0; p < n; ++p)
            for (int q = p + 1; q < n; ++q) off += A(p, q) * A(p, q);
        if (off < 1e-30) {
            storeEigenvalues();
            return true;
        }

        for (int p = 0; p < n; ++p) {
            for (int q = p + 1; q < n; ++q) {
                const double apq = A(p, q);
                if (std::fabs(apq) < 1e-300) continue;

                const double theta = (A(q, q) - A(p, p)) / (2.0 * apq);
                const double sgn = theta >= 0.0 ? 1.0 : -1.0;
                const double t = sgn / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;

                // A <- J^T A J, в два прохода: сначала столбцы, затем строки.
                for (int k = 0; k < n; ++k) {
                    const double akp = A(k, p), akq = A(k, q);
                    A(k, p) = c * akp - s * akq;
                    A(k, q) = s * akp + c * akq;
                }
                for (int k = 0; k < n; ++k) {
                    const double apk = A(p, k), aqk = A(q, k);
                    A(p, k) = c * apk - s * aqk;
                    A(q, k) = s * apk + c * aqk;
                }
                for (int k = 0; k < n; ++k) {
                    const double vkp = V(k, p), vkq = V(k, q);
                    V(k, p) = c * vkp - s * vkq;
                    V(k, q) = s * vkp + c * vkq;
                }
            }
        }
    }

    storeEigenvalues();
    return false;
}

namespace {

// Нормализация Хартли: центр масс в начало координат, средний радиус sqrt(2).
// Без неё DLT плохо обусловлен на пиксельных координатах.
bool hartleyNormalize(const std::vector<Vec2>& pts, std::vector<Vec2>& out, Mat3& t) {
    const size_t n = pts.size();
    if (n == 0) return false;

    double cx = 0.0, cy = 0.0;
    for (const Vec2& p : pts) {
        cx += p.x;
        cy += p.y;
    }
    cx /= static_cast<double>(n);
    cy /= static_cast<double>(n);

    double meanDist = 0.0;
    for (const Vec2& p : pts) {
        const double dx = p.x - cx, dy = p.y - cy;
        meanDist += std::sqrt(dx * dx + dy * dy);
    }
    meanDist /= static_cast<double>(n);
    if (meanDist < 1e-12) return false;  // все точки совпали

    const double scale = std::sqrt(2.0) / meanDist;

    t = Mat3::identity();
    t(0, 0) = scale;
    t(1, 1) = scale;
    t(0, 2) = -scale * cx;
    t(1, 2) = -scale * cy;

    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
        out[i].x = (pts[i].x - cx) * scale;
        out[i].y = (pts[i].y - cy) * scale;
    }
    return true;
}

}  // namespace

bool estimateHomography(const std::vector<Vec2>& src, const std::vector<Vec2>& dst, Mat3& out) {
    if (src.size() < 4 || src.size() != dst.size()) return false;

    std::vector<Vec2> ns, nd;
    Mat3 ts, td;
    if (!hartleyNormalize(src, ns, ts)) return false;
    if (!hartleyNormalize(dst, nd, td)) return false;

    const int n = static_cast<int>(ns.size());

    // Вместо SVD от A (2n x 9) считаем собственные векторы A^T*A (9 x 9):
    // решение - вектор при наименьшем собственном значении.
    std::vector<double> ata(81, 0.0);

    auto accumulate = [&](const double* r) {
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 9; ++j) ata[static_cast<size_t>(i) * 9 + j] += r[i] * r[j];
    };

    for (int i = 0; i < n; ++i) {
        const double x = ns[static_cast<size_t>(i)].x, y = ns[static_cast<size_t>(i)].y;
        const double u = nd[static_cast<size_t>(i)].x, v = nd[static_cast<size_t>(i)].y;

        const double rowU[9] = {-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u};
        accumulate(rowU);
        const double rowV[9] = {0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v};
        accumulate(rowV);
    }

    std::vector<double> eigenvalues, eigenvectors;
    jacobiEigenSymmetric(ata, 9, eigenvalues, eigenvectors);
    if (eigenvalues.size() != 9) return false;

    int best = 0;
    for (int i = 1; i < 9; ++i)
        if (eigenvalues[static_cast<size_t>(i)] < eigenvalues[static_cast<size_t>(best)]) best = i;

    Mat3 hn;
    for (int i = 0; i < 9; ++i)
        hn.m[static_cast<size_t>(i)] = eigenvectors[static_cast<size_t>(i) * 9 + best];

    // Возврат в исходные координаты: H = Td^-1 * Hn * Ts
    Mat3 tdInv;
    if (!td.inverse(tdInv)) return false;

    Mat3 h = tdInv * hn * ts;
    if (std::fabs(h.m[8]) < 1e-15) return false;
    h.normalizeH();
    out = h;
    return true;
}

ReprojectionError reprojectionError(const Mat3& h, const std::vector<Vec2>& src,
                                    const std::vector<Vec2>& dst) {
    ReprojectionError e;
    if (src.empty() || src.size() != dst.size()) return e;

    double sum = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
        bool ok = false;
        const Vec2 p = h.project(src[i], &ok);
        if (!ok) {
            e.mean = e.max = 1e9;
            return e;
        }
        const double dx = p.x - dst[i].x, dy = p.y - dst[i].y;
        const double d = std::sqrt(dx * dx + dy * dy);
        sum += d;
        if (d > e.max) e.max = d;
    }
    e.mean = sum / static_cast<double>(src.size());
    return e;
}

}  // namespace wz
