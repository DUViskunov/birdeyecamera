#include "wz/intrinsics_calib.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace wz {
namespace {

void column(const Mat3& h, int i, double out[3]) {
    out[0] = h(0, i);
    out[1] = h(1, i);
    out[2] = h(2, i);
}

// Вектор v_ij из работы Чжана: свёртка условия h_i^T B h_j в строку
// относительно шести независимых элементов симметричной матрицы B.
void buildV(const Mat3& h, int i, int j, double v[6]) {
    double a[3], b[3];
    column(h, i, a);
    column(h, j, b);

    v[0] = a[0] * b[0];
    v[1] = a[0] * b[1] + a[1] * b[0];
    v[2] = a[1] * b[1];
    v[3] = a[2] * b[0] + a[0] * b[2];
    v[4] = a[2] * b[1] + a[1] * b[2];
    v[5] = a[2] * b[2];
}

// B = K^-T K^-1 -> внутренние параметры. Формулы прямые из работы Чжана.
bool intrinsicsFromB(const std::vector<double>& b, Intrinsics& out, std::string* why) {
    auto fail = [&](const char* reason, double value) {
        if (why) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%s = %.6g", reason, value);
            *why = buf;
        }
        return false;
    };
    const double B11 = b[0], B12 = b[1], B22 = b[2], B13 = b[3], B23 = b[4], B33 = b[5];

    const double denom = B11 * B22 - B12 * B12;
    // B = K^-T K^-1 положительно определена, поэтому её ведущий минор строго
    // положителен. Отрицательный - признак того, что нулевой вектор не является
    // допустимой коникой, а не того, что не повезло со знаком.
    if (denom <= 0.0) return fail("B11*B22 - B12^2", denom);
    if (B11 <= 0.0) return fail("B11", B11);

    const double v0 = (B12 * B13 - B11 * B23) / denom;
    const double lambda = B33 - (B13 * B13 + v0 * (B12 * B13 - B11 * B23)) / B11;
    if (lambda / B11 <= 0.0) return fail("lambda/B11", lambda / B11);
    if (lambda * B11 / denom <= 0.0) return fail("lambda*B11/denom", lambda * B11 / denom);

    const double alpha = std::sqrt(lambda / B11);
    const double beta = std::sqrt(lambda * B11 / denom);
    // Величины безразмерные (доля размера кадра), поэтому проверяется только
    // положительность: осмысленность диапазона оценивается после возврата
    // к пикселям.
    if (!(alpha > 1e-6)) return fail("fx", alpha);
    if (!(beta > 1e-6)) return fail("fy", beta);

    // Перекос у матричных сенсоров пренебрежимо мал, и в модели проекта его
    // нет. В формуле для u0 он всё же учитывается: так центр ближе к данным.
    const double gamma = -B12 * alpha * alpha * beta / lambda;
    const double u0 = gamma * v0 / beta - B13 * alpha * alpha / lambda;

    out.fx = alpha;
    out.fy = beta;
    out.cx = u0;
    out.cy = v0;
    return true;
}

struct Extrinsics {
    double r1[3], r2[3], t[3];
    bool valid = false;
};

Extrinsics extrinsicsFrom(const Mat3& kInv, const Mat3& h) {
    Extrinsics e;

    double h1[3], h2[3], h3[3];
    column(h, 0, h1);
    column(h, 1, h2);
    column(h, 2, h3);

    auto apply = [&](const double v[3], double result[3]) {
        for (int i = 0; i < 3; ++i) {
            result[i] = kInv(i, 0) * v[0] + kInv(i, 1) * v[1] + kInv(i, 2) * v[2];
        }
    };

    double a1[3], a2[3], a3[3];
    apply(h1, a1);
    apply(h2, a2);
    apply(h3, a3);

    const double norm = std::sqrt(a1[0] * a1[0] + a1[1] * a1[1] + a1[2] * a1[2]);
    if (norm < 1e-12) return e;

    double lambda = 1.0 / norm;
    // Доска обязана быть перед камерой: знак нулевого вектора произволен.
    if (a3[2] < 0.0) lambda = -lambda;

    for (int i = 0; i < 3; ++i) {
        e.r1[i] = lambda * a1[i];
        e.r2[i] = lambda * a2[i];
        e.t[i] = lambda * a3[i];
    }
    e.valid = true;
    return e;
}

// Нормализованная проекция точки доски при известном положении камеры.
bool projectNormalized(const Extrinsics& e, const Vec2& board, Vec2& out) {
    const double x = e.r1[0] * board.x + e.r2[0] * board.y + e.t[0];
    const double y = e.r1[1] * board.x + e.r2[1] * board.y + e.t[1];
    const double z = e.r1[2] * board.x + e.r2[2] * board.y + e.t[2];
    if (z < 1e-9) return false;
    out = Vec2{x / z, y / z};
    return true;
}

// Среднеквадратичная невязка модели дисторсии при заданных k1, k2.
double distortionResidual(const std::vector<ChessboardCorners>& views,
                          const std::vector<Extrinsics>& poses, const Intrinsics& in,
                          double k1, double k2) {
    double sumSq = 0.0;
    size_t count = 0;

    for (size_t v = 0; v < views.size(); ++v) {
        if (!poses[v].valid) continue;
        const ChessboardCorners& c = views[v];

        for (size_t i = 0; i < c.board.size(); ++i) {
            Vec2 n;
            if (!projectNormalized(poses[v], c.board[i], n)) continue;

            const double r2 = n.x * n.x + n.y * n.y;
            const double scale = 1.0 + k1 * r2 + k2 * r2 * r2;
            const double du = in.fx * n.x, dv = in.fy * n.y;

            const double ex = (du * scale + in.cx) - c.image[i].x;
            const double ey = (dv * scale + in.cy) - c.image[i].y;
            sumSq += ex * ex + ey * ey;
            ++count;
        }
    }
    return count ? std::sqrt(sumSq / static_cast<double>(count)) : 1e9;
}

// Оценка радиальной дисторсии по невязке между идеальной и наблюдаемой
// проекцией. Модель линейна по коэффициентам, поэтому хватает нормальных
// уравнений.
//
// Считаются два варианта. На доступном диапазоне радиусов r^2 и r^4 почти
// коллинеарны, и система 2x2 вырождается: подобранные k1 и k2 компенсируют
// друг друга внутри снятой области, а за её пределами дают дикий разброс.
// Поэтому второй коэффициент принимается, только если он реально улучшает
// невязку и остаётся в разумных границах.
bool estimateDistortion(const std::vector<ChessboardCorners>& views,
                        const std::vector<Extrinsics>& poses, Intrinsics& in) {
    double a00 = 0, a01 = 0, a11 = 0, b0 = 0, b1 = 0;

    for (size_t v = 0; v < views.size(); ++v) {
        if (!poses[v].valid) continue;
        const ChessboardCorners& c = views[v];

        for (size_t i = 0; i < c.board.size(); ++i) {
            Vec2 n;
            if (!projectNormalized(poses[v], c.board[i], n)) continue;

            const double r2 = n.x * n.x + n.y * n.y;
            const double r4 = r2 * r2;
            const double du = in.fx * n.x;  // = u_ideal - cx
            const double dv = in.fy * n.y;

            const double ru = du * r2, su = du * r4;
            const double rv = dv * r2, sv = dv * r4;
            const double eu = c.image[i].x - (du + in.cx);
            const double ev = c.image[i].y - (dv + in.cy);

            a00 += ru * ru + rv * rv;
            a01 += ru * su + rv * sv;
            a11 += su * su + sv * sv;
            b0 += ru * eu + rv * ev;
            b1 += su * eu + sv * ev;
        }
    }

    if (a00 < 1e-18) return false;

    const double onlyK1 = b0 / a00;
    double k1 = onlyK1, k2 = 0.0;

    const double det = a00 * a11 - a01 * a01;
    if (std::fabs(det) > 1e-18) {
        const double pairK1 = (b0 * a11 - b1 * a01) / det;
        const double pairK2 = (b1 * a00 - b0 * a01) / det;

        const double rmsOne = distortionResidual(views, poses, in, onlyK1, 0.0);
        const double rmsTwo = distortionResidual(views, poses, in, pairK1, pairK2);

        if (std::fabs(pairK2) < 0.5 && rmsTwo < rmsOne * 0.9) {
            k1 = pairK1;
            k2 = pairK2;
        }
    }

    in.k1 = k1;
    in.k2 = k2;
    in.k3 = 0.0;
    in.p1 = 0.0;
    in.p2 = 0.0;
    return true;
}

// Невязка в пикселях между наблюдаемыми углами и их предсказанием.
void reprojection(const std::vector<ChessboardCorners>& views,
                  const std::vector<Extrinsics>& poses, const Intrinsics& in, double& rms,
                  double& maxErr, std::vector<double>& perView) {
    double sumSq = 0.0;
    size_t total = 0;
    maxErr = 0.0;
    perView.assign(views.size(), 0.0);

    for (size_t v = 0; v < views.size(); ++v) {
        if (!poses[v].valid) continue;
        const ChessboardCorners& c = views[v];

        double viewSq = 0.0;
        size_t viewCount = 0;

        for (size_t i = 0; i < c.board.size(); ++i) {
            Vec2 n;
            if (!projectNormalized(poses[v], c.board[i], n)) continue;

            const Vec2 d = in.distortNormalized(n);
            const double dx = (in.fx * d.x + in.cx) - c.image[i].x;
            const double dy = (in.fy * d.y + in.cy) - c.image[i].y;
            const double e2 = dx * dx + dy * dy;

            viewSq += e2;
            ++viewCount;
            maxErr = std::max(maxErr, std::sqrt(e2));
        }

        if (viewCount) {
            perView[v] = std::sqrt(viewSq / static_cast<double>(viewCount));
            sumSq += viewSq;
            total += viewCount;
        }
    }
    rms = total ? std::sqrt(sumSq / static_cast<double>(total)) : 0.0;
}

// Решение V*b = 0 по набору нормированных гомографий.
bool solveZhang(const std::vector<Mat3>& hs, std::vector<double>& b) {
    if (hs.size() < 3) return false;

    std::vector<double> vtv(36, 0.0);
    auto accumulate = [&](const double row[6]) {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) vtv[static_cast<size_t>(i) * 6 + j] += row[i] * row[j];
        }
    };

    for (const Mat3& h : hs) {
        double v01[6], v00[6], v11[6], diff[6];
        buildV(h, 0, 1, v01);
        buildV(h, 0, 0, v00);
        buildV(h, 1, 1, v11);
        for (int i = 0; i < 6; ++i) diff[i] = v00[i] - v11[i];
        accumulate(v01);
        accumulate(diff);
    }

    std::vector<double> eigenvalues, eigenvectors;
    jacobiEigenSymmetric(vtv, 6, eigenvalues, eigenvectors);
    if (eigenvalues.size() != 6) return false;

    int best = 0;
    for (int i = 1; i < 6; ++i) {
        if (eigenvalues[static_cast<size_t>(i)] < eigenvalues[static_cast<size_t>(best)]) {
            best = i;
        }
    }

    b.assign(6, 0.0);
    for (int i = 0; i < 6; ++i) {
        b[static_cast<size_t>(i)] = eigenvectors[static_cast<size_t>(i) * 6 + best];
    }
    // Знак нулевого вектора произволен, а B обязана быть положительно
    // определённой: иначе подкоренные выражения уйдут в минус.
    if (b[0] < 0.0) {
        for (double& value : b) value = -value;
    }
    return true;
}

// Насколько вид согласуется с решением. Нормировка на длину строки делает
// величину сравнимой между видами: у верно размеченной доски она около 1e-3,
// у сбитой разметки на порядок-два больше.
double zhangResidual(const Mat3& h, const std::vector<double>& b) {
    double v01[6], v00[6], v11[6];
    buildV(h, 0, 1, v01);
    buildV(h, 0, 0, v00);
    buildV(h, 1, 1, v11);

    double r1 = 0, r2 = 0, n1 = 0, n2 = 0;
    for (int k = 0; k < 6; ++k) {
        const double d = v00[k] - v11[k];
        r1 += v01[k] * b[static_cast<size_t>(k)];
        r2 += d * b[static_cast<size_t>(k)];
        n1 += v01[k] * v01[k];
        n2 += d * d;
    }

    const double a = n1 > 1e-18 ? std::fabs(r1) / std::sqrt(n1) : 0.0;
    const double c = n2 > 1e-18 ? std::fabs(r2) / std::sqrt(n2) : 0.0;
    return std::max(a, c);
}

}  // namespace

bool calibrateIntrinsics(const std::vector<ChessboardCorners>& views, int imageWidth,
                         int imageHeight, IntrinsicsCalibResult& out, std::string* error) {
    if (views.size() < 3) {
        if (error) {
            *error = "нужно не менее трёх видов доски, передано " + std::to_string(views.size()) +
                     ". Два вида не определяют пять параметров";
        }
        return false;
    }
    if (imageWidth < 16 || imageHeight < 16) {
        if (error) *error = "недопустимый размер кадра";
        return false;
    }
    for (const ChessboardCorners& c : views) {
        if (c.image.size() != c.board.size() || c.image.size() < 4) {
            if (error) *error = "вид '" + c.source + "': некорректный набор углов";
            return false;
        }
    }

    // Гомография переводит миллиметры доски в пиксели кадра, поэтому её
    // элементы различаются на порядки, и V^T*V вырождается. Обе системы
    // координат приводятся к безразмерным величинам, а масштаб возвращается
    // уже в извлечённые параметры.
    const double pixelScale = std::max(imageWidth, imageHeight);
    const double centreX = imageWidth * 0.5;
    const double centreY = imageHeight * 0.5;

    Mat3 toNormalisedPixels = Mat3::identity();
    toNormalisedPixels(0, 0) = 1.0 / pixelScale;
    toNormalisedPixels(1, 1) = 1.0 / pixelScale;
    toNormalisedPixels(0, 2) = -centreX / pixelScale;
    toNormalisedPixels(1, 2) = -centreY / pixelScale;

    double boardCx = 0.0, boardCy = 0.0;
    size_t boardCount = 0;
    for (const ChessboardCorners& c : views) {
        for (const Vec2& p : c.board) {
            boardCx += p.x;
            boardCy += p.y;
            ++boardCount;
        }
    }
    if (boardCount) {
        boardCx /= static_cast<double>(boardCount);
        boardCy /= static_cast<double>(boardCount);
    }

    double boardScale = 1.0;
    for (const ChessboardCorners& c : views) {
        for (const Vec2& p : c.board) {
            boardScale = std::max(boardScale, std::hypot(p.x - boardCx, p.y - boardCy));
        }
    }
    // Сдвиг и равномерный масштаб доски условий Чжана не нарушают: первые два
    // столбца растягиваются одинаково, а третий ими не связан.
    Mat3 fromNormalisedBoard = Mat3::identity();
    fromNormalisedBoard(0, 0) = boardScale;
    fromNormalisedBoard(1, 1) = boardScale;
    fromNormalisedBoard(0, 2) = boardCx;
    fromNormalisedBoard(1, 2) = boardCy;

    // Порог согласия между видами. Измерено: верная разметка даёт около 1e-3,
    // сбитая на клетку - от 1e-2.
    const double kInlierTolerance = 0.01;
    std::vector<size_t> accepted;

    // Извлечение параметров с возвратом к пикселям и проверкой осмысленности.
    // Нужна и при переборе троек: вырожденная тройка даёт формально решаемую
    // систему, но центр проекции улетает за кадр.
    auto extract = [&](const std::vector<double>& b, Intrinsics& k, std::string* why) {
        if (!intrinsicsFromB(b, k, why)) return false;

        k.fx *= pixelScale;
        k.fy *= pixelScale;
        k.cx = k.cx * pixelScale + centreX;
        k.cy = k.cy * pixelScale + centreY;

        if (k.cx < 0.0 || k.cx > imageWidth || k.cy < 0.0 || k.cy > imageHeight) {
            if (why) {
                *why = "центр проекции вне кадра (" + std::to_string(static_cast<int>(k.cx)) +
                       ", " + std::to_string(static_cast<int>(k.cy)) + ")";
            }
            return false;
        }
        // Фокусное вне этих границ означает ошибку разметки, а не объектив.
        if (k.fx < 0.2 * pixelScale || k.fx > 20.0 * pixelScale) {
            if (why) *why = "неправдоподобное fx = " + std::to_string(static_cast<int>(k.fx));
            return false;
        }
        return true;
    };

    // Стартовое приближение. Без него первая итерация строит гомографии по
    // искажённым точкам: они не лежат на проективном образе плоскости, условия
    // Чжана нарушаются на величину дисторсии, и решения не существует вовсе.
    // У веб-камеры фокусное близко к ширине кадра, центр - к середине.
    Intrinsics in;
    in.fx = in.fy = pixelScale;
    in.cx = imageWidth * 0.5;
    in.cy = imageHeight * 0.5;

    bool solved = false;
    std::string lastFailure;

    // Схема чередует линейные шаги (K по Чжану, затем дисторсия) и строгой
    // монотонности не гарантирует: измерено, что невязка после нескольких
    // проходов начинает расти. Поэтому запоминается лучшая итерация, а не
    // последняя.
    Intrinsics bestIntrinsics;
    std::vector<Extrinsics> bestPoses;
    double bestRms = 1e30;

    // Построение нормированных гомографий при заданной модели дисторсии.
    auto buildHomographies = [&](const Intrinsics& model, std::vector<Mat3>& raw,
                                 std::vector<Mat3>& norm) {
        raw.resize(views.size());
        norm.resize(views.size());
        for (size_t v = 0; v < views.size(); ++v) {
            std::vector<Vec2> pts = views[v].image;
            if (model.hasDistortion()) {
                for (Vec2& p : pts) p = model.rawPixelToUndistorted(p);
            }
            if (!estimateHomography(views[v].board, pts, raw[v])) return false;
            norm[v] = toNormalisedPixels * raw[v] * fromNormalisedBoard;
        }
        return true;
    };

    // Грубый подбор k1. Пока дисторсия не снята, точки не лежат на проективном
    // образе плоскости: условия Чжана нарушаются при любом K, положительно
    // определённой B не существует, и итерации из этой ямы не выходят.
    // Одномерный перебор выводит в область, где система разрешима.
    {
        Intrinsics probe = in;
        double bestScore = 1e30;

        for (int step = -30; step <= 15; ++step) {
            probe.k1 = step * 0.02;

            std::vector<Mat3> trialRaw, trialNorm;
            if (!buildHomographies(probe, trialRaw, trialNorm)) continue;

            std::vector<double> trialB;
            if (!solveZhang(trialNorm, trialB)) continue;

            Intrinsics k;
            if (!extract(trialB, k, nullptr)) continue;

            double score = 0.0;
            for (const Mat3& h : trialNorm) score += zhangResidual(h, trialB);
            score /= static_cast<double>(trialNorm.size());

            if (score < bestScore) {
                bestScore = score;
                in.k1 = probe.k1;
            }
        }
    }
    std::vector<Extrinsics> poses(views.size());
    std::vector<Mat3> homographies(views.size());
    std::vector<Mat3> normalised(views.size());

    // Первый проход идёт по сырым углам: дисторсия ещё неизвестна. Дальше углы
    // выпрямляются текущей оценкой, и всё пересчитывается заново - так линейное
    // решение подтягивается к нелинейной модели без полноценного Левенберга.
    const int kIterations = 6;
    for (int iter = 0; iter < kIterations; ++iter) {
        if (!buildHomographies(in, homographies, normalised)) {
            if (error) *error = "не удалось оценить гомографию одного из видов";
            return false;
        }

        // На первом проходе отбираются виды, согласные между собой: одна
        // сбитая разметка делает B не положительно определённой, и решения
        // не существует вовсе. Троек мало, поэтому перебираются все.
        if (iter < 2) {
            const size_t n = normalised.size();
            size_t bestCount = 0;
            std::vector<size_t> best;

            for (size_t i = 0; i < n && bestCount < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    for (size_t k = j + 1; k < n; ++k) {
                        std::vector<Mat3> trial = {normalised[i], normalised[j], normalised[k]};
                        std::vector<double> trialB;
                        if (!solveZhang(trial, trialB)) continue;

                        Intrinsics probe;
                        if (!extract(trialB, probe, nullptr)) continue;

                        std::vector<size_t> inliers;
                        for (size_t v = 0; v < n; ++v) {
                            if (zhangResidual(normalised[v], trialB) < kInlierTolerance) {
                                inliers.push_back(v);
                            }
                        }
                        if (inliers.size() > bestCount) {
                            bestCount = inliers.size();
                            best = inliers;
                        }
                    }
                }
            }

            if (best.size() >= 3) accepted = best;
            if (accepted.size() < 3) {
                if (error) {
                    *error = "среди " + std::to_string(n) +
                             " видов не набралось трёх согласованных. Проверьте, что доска "
                             "снята под разными наклонами и что на кадрах нет бликов";
                }
                return false;
            }
        }

        std::vector<Mat3> selected;
        for (size_t v : accepted) selected.push_back(normalised[v]);

        std::vector<double> b;
        if (!solveZhang(selected, b)) {
            if (error) *error = "разложение не сошлось";
            return false;
        }

        Intrinsics candidate;
        std::string why;
        if (!extract(b, candidate, &why)) {
            // Ранние итерации ещё не выпрямили точки, и решение может не
            // извлечься. Это не повод сдаваться: продолжаем с текущим
            // приближением, уточнив по нему дисторсию.
            lastFailure = why;
            candidate = in;
        } else {
            solved = true;
        }

        candidate.k1 = in.k1;
        candidate.k2 = in.k2;

        Mat3 kInv;
        if (!candidate.K().inverse(kInv)) {
            if (error) *error = "вырожденная матрица камеры";
            return false;
        }
        for (Extrinsics& e : poses) e.valid = false;
        for (size_t v : accepted) poses[v] = extrinsicsFrom(kInv, homographies[v]);

        estimateDistortion(views, poses, candidate);
        in = candidate;

        double rms = 0.0, maxErr = 0.0;
        std::vector<double> perView;
        reprojection(views, poses, in, rms, maxErr, perView);
        out.iterationRmsPx.push_back(rms);

        if (solved && rms < bestRms) {
            bestRms = rms;
            bestIntrinsics = in;
            bestPoses = poses;
        }
    }

    if (solved && bestRms < 1e29) {
        in = bestIntrinsics;
        poses = bestPoses;
    }

    // Линейное решение Чжана чувствительно к шуму разметки: сильнее всего
    // страдает центр проекции, он слабо влияет на сами условия. Измерено, что
    // шум в треть пикселя уводит cy на десятки пикселей. Лечится прямой
    // минимизацией невязки перепроецирования - здесь покоординатным спуском
    // с дроблением шага: он не требует производных и не расходится.
    if (solved) {
        auto evaluate = [&](const Intrinsics& candidate, std::vector<Extrinsics>& out) {
            Mat3 kInv;
            if (!candidate.K().inverse(kInv)) return 1e30;

            std::vector<Mat3> raw, norm;
            if (!buildHomographies(candidate, raw, norm)) return 1e30;

            out.assign(views.size(), Extrinsics{});
            for (size_t v : accepted) out[v] = extrinsicsFrom(kInv, raw[v]);

            double rms = 0.0, maxErr = 0.0;
            std::vector<double> perView;
            reprojection(views, out, candidate, rms, maxErr, perView);
            return rms;
        };

        std::vector<Extrinsics> trialPoses;
        double current = evaluate(in, trialPoses);

        // k2 участвует наравне с остальными: здесь он ограничен реальной
        // невязкой, а не вырожденной линейной системой, и не разлетается.
        double steps[6] = {8.0, 8.0, 6.0, 6.0, 0.02, 0.01};
        double* fields[6] = {&in.fx, &in.fy, &in.cx, &in.cy, &in.k1, &in.k2};

        for (int sweep = 0; sweep < 16; ++sweep) {
            bool improved = false;

            for (int f = 0; f < 6; ++f) {
                for (int dir = -1; dir <= 1; dir += 2) {
                    Intrinsics candidate = in;
                    double* target = &candidate.fx + (fields[f] - &in.fx);
                    *target += dir * steps[f];

                    std::vector<Extrinsics> candidatePoses;
                    const double value = evaluate(candidate, candidatePoses);
                    if (value < current) {
                        current = value;
                        in = candidate;
                        poses = candidatePoses;
                        improved = true;
                        break;
                    }
                }
            }

            if (!improved) {
                for (int f = 0; f < 6; ++f) steps[f] *= 0.5;
            }
        }
    }

    if (!solved) {
        if (error) {
            *error = "внутренние параметры не определились (" + lastFailure +
                     "). Снимите доску под разными наклонами и поворотами, "
                     "а не только сдвигами, и так, чтобы она побывала у краёв кадра";
        }
        return false;
    }

    double rms = 0.0, maxErr = 0.0;
    std::vector<double> perView;
    reprojection(views, poses, in, rms, maxErr, perView);

    out.intrinsics = in;
    out.rmsPx = rms;
    out.maxPx = maxErr;
    out.views = static_cast<int>(accepted.size());
    out.points = out.views * static_cast<int>(views.front().image.size());
    out.perViewRmsPx = perView;
    return true;
}

std::string intrinsicsToJsonString(const Intrinsics& in) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\n"
                  "  \"fx\": %.6f,\n"
                  "  \"fy\": %.6f,\n"
                  "  \"cx\": %.6f,\n"
                  "  \"cy\": %.6f,\n"
                  "  \"k1\": %.9f,\n"
                  "  \"k2\": %.9f,\n"
                  "  \"k3\": %.9f,\n"
                  "  \"p1\": %.9f,\n"
                  "  \"p2\": %.9f\n"
                  "}",
                  in.fx, in.fy, in.cx, in.cy, in.k1, in.k2, in.k3, in.p1, in.p2);
    return std::string(buf);
}

}  // namespace wz
