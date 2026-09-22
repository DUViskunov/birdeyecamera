#include "wz/chessboard.hpp"

#include <algorithm>
#include <cmath>

namespace wz {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<float> toGray(const Image& img) {
    const int w = img.width(), h = img.height();
    std::vector<float> g(static_cast<size_t>(w) * h, 0.f);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const uint8_t* p = img.pixel(x, y);
            g[static_cast<size_t>(y) * w + x] = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        }
    }
    return g;
}

// Биномиальное сглаживание 1-2-1, раздельно по осям. Применяется дважды:
// вторая производная по сырым пикселям слишком шумная, чтобы искать по ней сёдла.
void smooth(std::vector<float>& g, int w, int h) {
    std::vector<float> tmp(g.size());
    for (int y = 0; y < h; ++y) {
        const size_t row = static_cast<size_t>(y) * w;
        for (int x = 0; x < w; ++x) {
            const int x0 = std::max(0, x - 1), x1 = std::min(w - 1, x + 1);
            tmp[row + x] = 0.25f * g[row + x0] + 0.5f * g[row + x] + 0.25f * g[row + x1];
        }
    }
    for (int y = 0; y < h; ++y) {
        const int y0 = std::max(0, y - 1), y1 = std::min(h - 1, y + 1);
        for (int x = 0; x < w; ++x) {
            g[static_cast<size_t>(y) * w + x] =
                0.25f * tmp[static_cast<size_t>(y0) * w + x] +
                0.5f * tmp[static_cast<size_t>(y) * w + x] +
                0.25f * tmp[static_cast<size_t>(y1) * w + x];
        }
    }
}

// Уточнение угла до субпикселя: по окну подгоняется квадратичная поверхность
// I = a + bx + cy + dx^2 + exy + fy^2, и берётся её седловая точка.
// Окно симметрично, поэтому суммы нечётных степеней обнуляются и нормальные
// уравнения распадаются - обходимся без общего решателя.
Vec2 refineSaddle(const std::vector<float>& g, int w, int h, int cx, int cy, int r) {
    const Vec2 fallback{static_cast<double>(cx), static_cast<double>(cy)};
    if (cx - r < 0 || cy - r < 0 || cx + r >= w || cy + r >= h) return fallback;

    double s0 = 0, s2 = 0, s4 = 0, s22 = 0;
    double sI = 0, sxI = 0, syI = 0, sxyI = 0, sx2I = 0, sy2I = 0;

    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            const double x = dx, y = dy;
            const double I = g[static_cast<size_t>(cy + dy) * w + (cx + dx)];
            s0 += 1.0;
            s2 += x * x;
            s4 += x * x * x * x;
            s22 += x * x * y * y;
            sI += I;
            sxI += x * I;
            syI += y * I;
            sxyI += x * y * I;
            sx2I += x * x * I;
            sy2I += y * y * I;
        }
    }

    const double b = s2 > 1e-12 ? sxI / s2 : 0.0;
    const double c = s2 > 1e-12 ? syI / s2 : 0.0;
    const double e = s22 > 1e-12 ? sxyI / s22 : 0.0;

    // d - f берётся из разности уравнений, d + f - из системы с константой a.
    const double dMinusF = (s4 - s22) > 1e-12 ? (sx2I - sy2I) / (s4 - s22) : 0.0;
    const double det = s0 * (s4 + s22) - 2.0 * s2 * s2;
    const double dPlusF =
        std::fabs(det) > 1e-12 ? (s0 * (sx2I + sy2I) - 2.0 * s2 * sI) / det : 0.0;
    const double d = 0.5 * (dPlusF + dMinusF);
    const double f = 0.5 * (dPlusF - dMinusF);

    // Градиент в ноль: [2d e; e 2f] * [ox oy]^T = [-b -c]^T
    const double m00 = 2.0 * d, m01 = e, m11 = 2.0 * f;
    const double md = m00 * m11 - m01 * m01;
    if (std::fabs(md) < 1e-12) return fallback;

    const double ox = (-b * m11 + c * m01) / md;
    const double oy = (-c * m00 + b * m01) / md;

    // Убегание за пределы окна означает, что подгонка не удалась.
    if (std::fabs(ox) > r || std::fabs(oy) > r) return fallback;
    return Vec2{cx + ox, cy + oy};
}

// Средняя яркость блока 3x3 вокруг точки.
float patchMean(const std::vector<float>& g, int w, int h, int x, int y) {
    float sum = 0.f;
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int px = x + dx, py = y + dy;
            if (px < 0 || py < 0 || px >= w || py >= h) continue;
            sum += g[static_cast<size_t>(py) * w + px];
            ++count;
        }
    }
    return count ? sum / static_cast<float>(count) : 0.f;
}

// Настоящий угол клетки - X-образный стык: противоположные квадранты одного
// тона, соседние - разного. Край доски даёт T-образный стык (две клетки плюс
// фон), и он этой проверки не проходит. Без неё лишние точки попадают в
// выпуклую оболочку, и рамка доски определяется неверно.
bool isCrossJunction(const std::vector<float>& g, int w, int h, int x, int y, int r) {
    const float a = patchMean(g, w, h, x + r, y + r);
    const float b = patchMean(g, w, h, x - r, y + r);
    const float c = patchMean(g, w, h, x - r, y - r);
    const float d = patchMean(g, w, h, x + r, y - r);

    const float diff = std::fabs((a + c) - (b + d));
    const float same = std::fabs(a - c) + std::fabs(b - d);

    return diff > 30.f && same < 0.6f * diff;
}

struct CornerCandidate {
    Vec2 p;
    float score;
    // Прошёл проверку на X-образность. Такие точки годятся для поиска рамки
    // доски; для снятия сетки используются все, иначе строгая проверка
    // выбрасывала бы углы на сильно скошенных ракурсах.
    bool strong;
};

double cross(const Vec2& o, const Vec2& a, const Vec2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Выпуклая оболочка методом Эндрю.
std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    if (pts.size() < 3) return pts;
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });

    std::vector<Vec2> hull(pts.size() * 2);
    size_t k = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) --k;
        hull[k++] = pts[i];
    }
    const size_t lower = k + 1;
    for (size_t i = pts.size() - 1; i > 0; --i) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0) --k;
        hull[k++] = pts[i - 1];
    }
    hull.resize(k > 0 ? k - 1 : 0);
    return hull;
}

double triangleArea(const Vec2& a, const Vec2& b, const Vec2& c) {
    return std::fabs(cross(a, b, c)) * 0.5;
}

// Четырёхугольник максимальной площади на оболочке - внешняя рамка доски.
// Перебирается диагональ, две оставшиеся вершины берутся как самые дальние
// по разные стороны от неё.
bool maxAreaQuad(const std::vector<Vec2>& hull, std::vector<Vec2>& quad) {
    const size_t n = hull.size();
    if (n < 4) return false;

    double best = -1.0;
    size_t bi = 0, bj = 0, bk = 0, bl = 0;

    for (size_t i = 0; i < n; ++i) {
        for (size_t k = i + 2; k < n; ++k) {
            double bestLeft = -1.0, bestRight = -1.0;
            size_t jl = i, jr = k;

            for (size_t j = i + 1; j < k; ++j) {
                const double a = triangleArea(hull[i], hull[j], hull[k]);
                if (a > bestLeft) {
                    bestLeft = a;
                    jl = j;
                }
            }
            for (size_t l = k + 1; l < n + i; ++l) {
                const double a = triangleArea(hull[i], hull[k], hull[l % n]);
                if (a > bestRight) {
                    bestRight = a;
                    jr = l % n;
                }
            }
            if (bestLeft < 0 || bestRight < 0) continue;

            if (bestLeft + bestRight > best) {
                best = bestLeft + bestRight;
                bi = i;
                bj = jl;
                bk = k;
                bl = jr;
            }
        }
    }
    if (best <= 0.0) return false;

    quad = {hull[bi], hull[bj], hull[bk], hull[bl]};

    // Ориентация обхода оболочки не задана заранее, а нумерация сетки задана.
    // При несовпадении гомография вышла бы зеркальной: углы всё равно нашлись
    // бы (отклик седла симметричен), но det(R) стал бы отрицательным, и
    // условия Чжана перестали бы выполняться. Приводим обход к сетке.
    double signedArea = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const Vec2& a = quad[i];
        const Vec2& b = quad[(i + 1) % 4];
        signedArea += a.x * b.y - b.x * a.y;
    }
    if (signedArea < 0.0) std::swap(quad[1], quad[3]);

    return true;
}

// Положение угла (col,row) в системе координат гомографии.
// inset = 0: рамка проходит по кольцу внутренних углов.
// inset = 1: рамка - внешняя граница доски, то есть на клетку шире с каждой
// стороны. Какая из двух найдена, заранее неизвестно: стыки крайних клеток
// с фоном тоже дают седловой отклик и попадают в выпуклую оболочку.
Vec2 gridSource(int col, int row, const ChessboardSpec& spec, int inset) {
    if (inset < 0) return Vec2{static_cast<double>(col), static_cast<double>(row)};
    if (inset == 0) {
        return Vec2{static_cast<double>(col) / (spec.cols - 1),
                    static_cast<double>(row) / (spec.rows - 1)};
    }
    return Vec2{(col + 1.0) / (spec.cols + 1.0), (row + 1.0) / (spec.rows + 1.0)};
}

// Сопоставление предсказанной сетки с кандидатами. Возвращает число найденных
// углов; matched[i] = -1, если угол не нашёлся.
int snapGrid(const Mat3& gridToImage, const ChessboardSpec& spec, int inset,
             const std::vector<Vec2>& candidates, double tolerance,
             std::vector<int>& matched) {
    matched.assign(static_cast<size_t>(spec.cornerCount()), -1);
    int found = 0;

    for (int row = 0; row < spec.rows; ++row) {
        for (int col = 0; col < spec.cols; ++col) {
            bool ok = false;
            const Vec2 predicted =
                gridToImage.project(gridSource(col, row, spec, inset), &ok);
            if (!ok) continue;

            double bestDist = tolerance;
            int bestIdx = -1;
            for (size_t c = 0; c < candidates.size(); ++c) {
                const double dx = candidates[c].x - predicted.x;
                const double dy = candidates[c].y - predicted.y;
                const double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = static_cast<int>(c);
                }
            }
            if (bestIdx >= 0) {
                // Один кандидат не может отвечать двум углам сразу.
                bool taken = false;
                for (int prev : matched) {
                    if (prev == bestIdx) { taken = true; break; }
                }
                if (taken) continue;
                matched[static_cast<size_t>(row) * spec.cols + col] = bestIdx;
                ++found;
            }
        }
    }
    return found;
}

std::vector<CornerCandidate> detectCandidates(const Image& img, int maxCount) {
    const int w = img.width(), h = img.height();
    if (w < 32 || h < 32) return {};

    std::vector<float> g = toGray(img);
    smooth(g, w, h);
    smooth(g, w, h);

    // Шаг шаблона второй производной. Единичный шаг настроен на пиксельный шум,
    // а угол клетки - структура масштаба в единицы пикселей.
    const int s = 2;
    std::vector<float> score(static_cast<size_t>(w) * h, 0.f);

    for (int y = s; y < h - s; ++y) {
        for (int x = s; x < w - s; ++x) {
            const size_t i = static_cast<size_t>(y) * w + x;
            const size_t rowStep = static_cast<size_t>(s) * w;
            const float c = g[i];
            const float ixx = g[i - s] - 2.f * c + g[i + s];
            const float iyy = g[i - rowStep] - 2.f * c + g[i + rowStep];
            const float ixy = 0.25f * (g[i + rowStep + s] - g[i + rowStep - s] -
                                       g[i - rowStep + s] + g[i - rowStep - s]);

            // Седло - отрицательный определитель гессиана. Его модуль и есть
            // сила отклика: у X-образного стыка клеток он максимален.
            const float det = ixx * iyy - ixy * ixy;
            score[i] = det < 0.f ? -det : 0.f;
        }
    }

    float peak = 0.f;
    for (float v : score) peak = std::max(peak, v);
    if (peak <= 0.f) return {};
    const float threshold = peak * 0.02f;

    std::vector<CornerCandidate> found;
    const int r = 4;  // радиус подавления немаксимумов

    for (int y = r; y < h - r; ++y) {
        for (int x = r; x < w - r; ++x) {
            const float v = score[static_cast<size_t>(y) * w + x];
            if (v < threshold) continue;

            bool isMax = true;
            for (int dy = -r; dy <= r && isMax; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (score[static_cast<size_t>(y + dy) * w + (x + dx)] > v) {
                        isMax = false;
                        break;
                    }
                }
            }
            if (!isMax) continue;

            // Проверка на нескольких радиусах: шаг клетки заранее неизвестен
            // и на скошенных ракурсах сильно меняется по кадру.
            const bool strong = isCrossJunction(g, w, h, x, y, 3) ||
                                isCrossJunction(g, w, h, x, y, 5) ||
                                isCrossJunction(g, w, h, x, y, 8);
            found.push_back({refineSaddle(g, w, h, x, y, 3), v, strong});
        }
    }

    std::sort(found.begin(), found.end(),
              [](const CornerCandidate& a, const CornerCandidate& b) {
                  return a.score > b.score;
              });
    if (static_cast<int>(found.size()) > maxCount) {
        found.resize(static_cast<size_t>(maxCount));
    }
    return found;
}

}  // namespace

std::vector<Vec2> detectSaddlePoints(const Image& img, int maxCount) {
    std::vector<Vec2> out;
    for (const CornerCandidate& c : detectCandidates(img, maxCount)) out.push_back(c.p);
    return out;
}

bool findChessboard(const Image& img, const ChessboardSpec& spec, ChessboardCorners& out,
                    std::string* error) {
    if (!spec.valid()) {
        if (error) {
            *error =
                "некорректные параметры доски: нужно не менее 3x3 внутренних углов, "
                "и стороны должны различаться";
        }
        return false;
    }
    if (img.empty()) {
        if (error) *error = "пустое изображение";
        return false;
    }

    const std::vector<CornerCandidate> raw = detectCandidates(img, 600);

    std::vector<Vec2> candidates, strong;
    for (const CornerCandidate& c : raw) {
        candidates.push_back(c.p);
        if (c.strong) strong.push_back(c.p);
    }

    if (static_cast<int>(candidates.size()) < spec.cornerCount()) {
        if (error) {
            *error = "найдено углов-кандидатов: " + std::to_string(candidates.size()) +
                     ", нужно минимум " + std::to_string(spec.cornerCount());
        }
        return false;
    }

    // Рамка ищется только по X-образным стыкам: края доски дают T-образные,
    // и они увели бы выпуклую оболочку за кольцо внутренних углов.
    const std::vector<Vec2> hull = convexHull(strong.size() >= 4 ? strong : candidates);
    std::vector<Vec2> quad;
    if (!maxAreaQuad(hull, quad)) {
        if (error) *error = "не удалось выделить рамку доски";
        return false;
    }

    // Гомография строится из единичного квадрата: так одна и та же рамка
    // проверяется под обе гипотезы о том, что она ограничивает.
    const std::vector<Vec2> unitSquare = {Vec2{0, 0}, Vec2{1, 0}, Vec2{1, 1}, Vec2{0, 1}};

    double span = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const Vec2& a = quad[i];
        const Vec2& b = quad[(i + 1) % 4];
        span = std::max(span, std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
    }
    // Допуск - доля шага сетки, чтобы не захватить соседний угол.
    const double tolerance = 0.35 * span / (std::max(spec.cols, spec.rows) + 1);

    std::vector<int> bestMatched;
    int bestFound = -1;

    // Какая вершина рамки отвечает углу сетки (0,0), заранее неизвестно:
    // перебираем четыре поворота. Для несимметричной доски неверные варианты
    // отпадают сами - сетка на них не ложится.
    for (int inset = 0; inset <= 1; ++inset) {
        for (int rot = 0; rot < 4; ++rot) {
            std::vector<Vec2> dst(4);
            for (int i = 0; i < 4; ++i) dst[static_cast<size_t>(i)] = quad[(i + rot) % 4];

            Mat3 h;
            if (!estimateHomography(unitSquare, dst, h)) continue;

            std::vector<int> matched;
            const int found = snapGrid(h, spec, inset, candidates, tolerance, matched);
            if (found > bestFound) {
                bestFound = found;
                bestMatched = matched;
            }
        }
    }

    if (bestFound < spec.cornerCount() * 3 / 4) {
        if (error) {
            *error = "сетка доски не сложилась: найдено " + std::to_string(bestFound) + " из " +
                     std::to_string(spec.cornerCount()) + " углов";
        }
        return false;
    }

    // Уточнение: по уже найденным углам гомография считается точнее, чем по
    // четырём вершинам рамки, и добирает недостающие.
    for (int pass = 0; pass < 3 && bestFound < spec.cornerCount(); ++pass) {
        std::vector<Vec2> src, dst;
        for (int row = 0; row < spec.rows; ++row) {
            for (int col = 0; col < spec.cols; ++col) {
                const int idx = bestMatched[static_cast<size_t>(row) * spec.cols + col];
                if (idx < 0) continue;
                src.push_back(gridSource(col, row, spec, -1));
                dst.push_back(candidates[static_cast<size_t>(idx)]);
            }
        }
        if (src.size() < 4) break;

        Mat3 refined;
        if (!estimateHomography(src, dst, refined)) break;

        std::vector<int> matched;
        const int found = snapGrid(refined, spec, -1, candidates, tolerance, matched);
        if (found <= bestFound) break;
        bestFound = found;
        bestMatched = matched;
    }

    if (bestFound != spec.cornerCount()) {
        if (error) {
            *error = "доска найдена не целиком: " + std::to_string(bestFound) + " из " +
                     std::to_string(spec.cornerCount()) +
                     " углов. Неполный набор испортил бы гомографию вида";
        }
        return false;
    }

    out.image.assign(static_cast<size_t>(spec.cornerCount()), Vec2{});
    out.board.assign(static_cast<size_t>(spec.cornerCount()), Vec2{});
    for (int row = 0; row < spec.rows; ++row) {
        for (int col = 0; col < spec.cols; ++col) {
            const size_t i = static_cast<size_t>(row) * spec.cols + col;
            out.image[i] = candidates[static_cast<size_t>(bestMatched[i])];
            out.board[i] = Vec2{col * spec.squareMm, row * spec.squareMm};
        }
    }
    return true;
}

Mat3 renderChessboardView(int view, const ChessboardSpec& spec, const Intrinsics& in,
                          Image& out) {
    const int w = out.width() > 0 ? out.width() : 640;
    const int h = out.height() > 0 ? out.height() : 480;
    if (out.width() != w || out.height() != h) out = Image(w, h);

    const double sq = spec.squareMm;
    const double centreX = (spec.cols - 1) * sq * 0.5;
    const double centreY = (spec.rows - 1) * sq * 0.5;

    // Ракурсы раскладываются по азимуту и наклону: параллельные снимки дают
    // вырожденную систему, и внутренние параметры по ним не определяются.
    const double azim = ((view % 5) - 2) * 17.0 * kPi / 180.0;
    const double elev = (34.0 + (view / 5) * 16.0) * kPi / 180.0;
    const double dist = 360.0 + (view % 3) * 45.0;

    const double camX = centreX + dist * std::cos(elev) * std::sin(azim);
    const double camY = centreY - dist * std::cos(elev) * std::cos(azim);
    const double camZ = dist * std::sin(elev);

    // Базис камеры: ось z смотрит в центр доски, ось y экрана направлена вниз.
    double fwdX = centreX - camX, fwdY = centreY - camY, fwdZ = -camZ;
    const double fn = std::sqrt(fwdX * fwdX + fwdY * fwdY + fwdZ * fwdZ);
    fwdX /= fn;
    fwdY /= fn;
    fwdZ /= fn;

    // right = normalize(forward x worldUp), worldUp = (0, 0, 1) -> (fy, -fx, 0).
    double rightX = fwdY, rightY = -fwdX, rightZ = 0.0;
    const double rn = std::sqrt(rightX * rightX + rightY * rightY);
    if (rn < 1e-9) {
        rightX = 1.0;
        rightY = 0.0;
    } else {
        rightX /= rn;
        rightY /= rn;
    }

    // down = right x forward
    const double downX = rightY * fwdZ - rightZ * fwdY;
    const double downY = rightZ * fwdX - rightX * fwdZ;
    const double downZ = rightX * fwdY - rightY * fwdX;

    // Доска смещается по кадру: если она всегда в центре, края остаются
    // неснятыми, и дисторсия на больших радиусах не определяется. То же
    // требование действует и при реальной съёмке.
    const double shiftRight = ((view % 3) - 1) * 85.0;
    const double shiftDown = (((view / 3) % 3) - 1) * 65.0;

    const double camX2 = camX + rightX * shiftRight + downX * shiftDown;
    const double camY2 = camY + rightY * shiftRight + downY * shiftDown;
    const double camZ2 = camZ + rightZ * shiftRight + downZ * shiftDown;

    const double t0 = -(rightX * camX2 + rightY * camY2 + rightZ * camZ2);
    const double t1 = -(downX * camX2 + downY * camY2 + downZ * camZ2);
    const double t2 = -(fwdX * camX2 + fwdY * camY2 + fwdZ * camZ2);

    // Для плоскости z = 0: H = K * [r1 r2 t], где r1, r2 - первые два столбца R.
    Mat3 rt;
    rt(0, 0) = rightX; rt(1, 0) = downX; rt(2, 0) = fwdX;
    rt(0, 1) = rightY; rt(1, 1) = downY; rt(2, 1) = fwdY;
    rt(0, 2) = t0;     rt(1, 2) = t1;    rt(2, 2) = t2;

    Mat3 boardToImage = in.K() * rt;
    boardToImage.normalizeH();

    Mat3 imageToBoard;
    if (!boardToImage.inverse(imageToBoard)) return boardToImage;

    // Клетки нарезаются вокруг сетки углов: внутренний угол лежит на стыке
    // четырёх клеток, поэтому клеток на одну больше по каждой стороне.
    const double minX = -sq, maxX = spec.cols * sq;
    const double minY = -sq, maxY = spec.rows * sq;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const Vec2 ideal =
                in.rawPixelToUndistorted(Vec2{static_cast<double>(x), static_cast<double>(y)});
            bool ok = false;
            const Vec2 b = imageToBoard.project(ideal, &ok);

            float value = 128.f;  // фон вокруг доски
            if (ok && b.x >= minX && b.x < maxX && b.y >= minY && b.y < maxY) {
                // Сдвиг на 1000 клеток держит аргумент floor положительным:
                // начало координат лежит на внутреннем углу, а не на краю доски.
                const int kx = static_cast<int>(std::floor(b.x / sq)) + 1000;
                const int ky = static_cast<int>(std::floor(b.y / sq)) + 1000;
                value = ((kx + ky) % 2 == 0) ? 232.f : 26.f;
            }
            out.set(x, y, RGB{value, value, value});
        }
    }
    return boardToImage;
}

}  // namespace wz
