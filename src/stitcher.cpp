#include "wz/stitcher.hpp"

#include <algorithm>
#include <cmath>
#include <thread>
#include <utility>

namespace wz {

bool Stitcher::configure(const StitchConfig& cfg, const std::vector<CameraPlacement>& cameras,
                         std::string* error) {
    tables_.clear();
    coverage_.clear();
    stats_ = StitchStats{};

    if (cameras.empty()) {
        if (error) *error = "не задана ни одна камера";
        return false;
    }
    if (cfg.canvasWidth <= 0 || cfg.canvasHeight <= 0) {
        if (error) *error = "недопустимый размер холста";
        return false;
    }

    cfg_ = cfg;
    cameras_ = cameras;

    const size_t canvasPixels = static_cast<size_t>(cfg_.canvasWidth) * cfg_.canvasHeight;
    tables_.resize(cameras_.size());
    coverage_.assign(canvasPixels, 0);

    accumR_.assign(canvasPixels, 0.f);
    accumG_.assign(canvasPixels, 0.f);
    accumB_.assign(canvasPixels, 0.f);
    accumW_.assign(canvasPixels, 0.f);

    const float feather = std::max(0.f, cfg_.featherPx);

    for (size_t ci = 0; ci < cameras_.size(); ++ci) {
        const CameraPlacement& cam = cameras_[ci];
        if (cam.imageWidth <= 0 || cam.imageHeight <= 0) {
            if (error) *error = "камера " + cam.sourceId + ": не задан размер кадра";
            return false;
        }

        Mat3 canvasToImage;
        if (!cam.homography.inverse(canvasToImage)) {
            if (error) *error = "камера " + cam.sourceId + ": вырожденная гомография";
            return false;
        }

        auto& table = tables_[ci];

        for (int y = 0; y < cfg_.canvasHeight; ++y) {
            for (int x = 0; x < cfg_.canvasWidth; ++x) {
                bool ok = false;
                // Холст -> идеальный (исправленный) кадр камеры.
                const Vec2 ideal = canvasToImage.project(
                    Vec2{static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5}, &ok);
                if (!ok) continue;

                // Отсекаем по границам идеального кадра: за ними данных нет.
                if (ideal.x < 0.0 || ideal.y < 0.0 || ideal.x > cam.imageWidth - 1.0 ||
                    ideal.y > cam.imageHeight - 1.0) {
                    continue;
                }

                // Вес: растушёвка к краю кадра, чтобы шов не был виден.
                float weight = 1.f;
                if (feather > 0.f) {
                    const double edge =
                        std::min(std::min(ideal.x, ideal.y),
                                 std::min(cam.imageWidth - 1.0 - ideal.x,
                                          cam.imageHeight - 1.0 - ideal.y));
                    weight = static_cast<float>(std::min(1.0, edge / feather));
                    if (weight <= 0.001f) continue;
                }

                // Дисторсию учитываем здесь же: одна выборка вместо двух
                // (сначала коррекция кадра, потом перенос) - меньше замыливания.
                const Vec2 raw = cam.intrinsics.valid()
                                     ? cam.intrinsics.undistortedPixelToRaw(ideal)
                                     : ideal;
                if (raw.x < 0.0 || raw.y < 0.0 || raw.x > cam.imageWidth - 1.0 ||
                    raw.y > cam.imageHeight - 1.0) {
                    continue;
                }

                const int32_t dst = y * cfg_.canvasWidth + x;
                table.push_back(WarpEntry{dst, static_cast<float>(raw.x),
                                          static_cast<float>(raw.y), weight});
                if (coverage_[static_cast<size_t>(dst)] < 255) {
                    ++coverage_[static_cast<size_t>(dst)];
                }
            }
        }

        if (table.empty()) {
            if (error) {
                *error = "камера " + cam.sourceId +
                         ": ни один пиксель не попал на холст, проверьте гомографию";
            }
            return false;
        }
    }

    samples_.resize(tables_.size());
    for (size_t ci = 0; ci < tables_.size(); ++ci) {
        samples_[ci].assign(tables_[ci].size() * 3, 0.f);
    }
    bestLuma_.assign(canvasPixels, 0.f);

    size_t covered = 0, overlapped = 0;
    for (uint8_t c : coverage_) {
        if (c > 0) ++covered;
        if (c > 1) ++overlapped;
    }
    stats_.coverage = static_cast<double>(covered) / static_cast<double>(canvasPixels);
    stats_.overlap = static_cast<double>(overlapped) / static_cast<double>(canvasPixels);
    stats_.gains.assign(cameras_.size(), 1.0);

    return true;
}

bool Stitcher::blend(const std::vector<Frame>& frames, Image& out) {
    if (!configured() || frames.size() != cameras_.size()) return false;

    const int w = cfg_.canvasWidth;
    const int h = cfg_.canvasHeight;
    if (out.width() != w || out.height() != h) out = Image(w, h);

    // Выравнивание яркости по зоне перекрытия. Опорной считается первая камера:
    // остальные подтягиваются к ней, иначе на шве видна ступенька.
    std::vector<double> gains(cameras_.size(), 1.0);
    if (cfg_.exposureCompensation && cameras_.size() > 1) {
        std::vector<double> sum(cameras_.size(), 0.0);
        std::vector<double> count(cameras_.size(), 0.0);

        for (size_t ci = 0; ci < tables_.size(); ++ci) {
            const Image& img = frames[ci].image;
            if (img.empty()) continue;
            // Хватает разреженной выборки: нужна средняя яркость, а не точность.
            const size_t step = std::max<size_t>(1, tables_[ci].size() / 4000);
            for (size_t i = 0; i < tables_[ci].size(); i += step) {
                const WarpEntry& e = tables_[ci][i];
                if (coverage_[static_cast<size_t>(e.dstIndex)] < 2) continue;
                RGB c;
                if (!img.sampleBilinear(e.srcX, e.srcY, c)) continue;
                sum[ci] += 0.299 * c.r + 0.587 * c.g + 0.114 * c.b;
                count[ci] += 1.0;
            }
        }

        const double reference = count[0] > 0.0 ? sum[0] / count[0] : 0.0;
        for (size_t ci = 1; ci < cameras_.size(); ++ci) {
            if (count[ci] > 0.0 && reference > 1.0) {
                const double mean = sum[ci] / count[ci];
                if (mean > 1.0) {
                    // Ограничиваем поправку: иначе блик в перекрытии
                    // перекрасит весь кадр.
                    gains[ci] = std::min(1.6, std::max(0.6, reference / mean));
                }
            }
        }
    }

    const float threshold =
        cfg_.occlusionRejection ? std::max(1.f, cfg_.occlusionThreshold) : 1e9f;

    // Холст режется на горизонтальные полосы. Таблицы переноса построены
    // в порядке обхода холста, поэтому записи каждой полосы лежат подряд и
    // находятся двоичным поиском: потоки не пересекаются ни по одному пикселю.
    int bandCount = cfg_.threads > 0 ? cfg_.threads
                                     : static_cast<int>(std::thread::hardware_concurrency());
    bandCount = std::max(1, std::min(bandCount, 32));
    bandCount = std::min(bandCount, h);

    auto processBand = [&](int band) {
        const int y0 = static_cast<int>(static_cast<int64_t>(band) * h / bandCount);
        const int y1 = static_cast<int>(static_cast<int64_t>(band + 1) * h / bandCount);
        if (y0 >= y1) return;

        const size_t first = static_cast<size_t>(y0) * w;
        const size_t last = static_cast<size_t>(y1) * w;

        for (size_t i = first; i < last; ++i) {
            accumR_[i] = 0.f;
            accumG_[i] = 0.f;
            accumB_[i] = 0.f;
            accumW_[i] = 0.f;
            bestLuma_[i] = -1.f;
        }

        auto rangeFor = [&](size_t ci) {
            const std::vector<WarpEntry>& t = tables_[ci];
            const auto byIndex = [](const WarpEntry& e, int32_t v) { return e.dstIndex < v; };
            const auto lo =
                std::lower_bound(t.begin(), t.end(), static_cast<int32_t>(first), byIndex);
            const auto hi =
                std::lower_bound(t.begin(), t.end(), static_cast<int32_t>(last), byIndex);
            return std::make_pair(static_cast<size_t>(lo - t.begin()),
                                  static_cast<size_t>(hi - t.begin()));
        };

        // Первый проход: выборка, приведение к общей экспозиции и поиск самой
        // светлой камеры для каждого пикселя полосы.
        for (size_t ci = 0; ci < tables_.size(); ++ci) {
            const Image& img = frames[ci].image;
            const float gain = static_cast<float>(gains[ci]);
            std::vector<float>& cache = samples_[ci];
            const auto range = rangeFor(ci);

            for (size_t i = range.first; i < range.second; ++i) {
                const WarpEntry& e = tables_[ci][i];
                RGB c;
                if (!img.empty() && img.sampleBilinear(e.srcX, e.srcY, c)) {
                    c.r *= gain;
                    c.g *= gain;
                    c.b *= gain;
                } else {
                    // Отрицательная яркость помечает отсутствующий отсчёт.
                    c = RGB{-1.f, -1.f, -1.f};
                }

                cache[i * 3 + 0] = c.r;
                cache[i * 3 + 1] = c.g;
                cache[i * 3 + 2] = c.b;

                if (c.r < 0.f) continue;
                const float l = 0.299f * c.r + 0.587f * c.g + 0.114f * c.b;
                float& best = bestLuma_[static_cast<size_t>(e.dstIndex)];
                if (l > best) best = l;
            }
        }

        // Второй проход: в сумму идут только отсчёты, близкие к самому светлому.
        // Отсечённое - участки, закрытые головой лазера в одной из камер.
        for (size_t ci = 0; ci < tables_.size(); ++ci) {
            const std::vector<float>& cache = samples_[ci];
            const auto range = rangeFor(ci);

            for (size_t i = range.first; i < range.second; ++i) {
                const WarpEntry& e = tables_[ci][i];
                const float r = cache[i * 3 + 0];
                if (r < 0.f) continue;
                const float g = cache[i * 3 + 1];
                const float b = cache[i * 3 + 2];

                const float l = 0.299f * r + 0.587f * g + 0.114f * b;
                const size_t idx = static_cast<size_t>(e.dstIndex);

                // Отставание от самой светлой камеры. Отсечение делается плавным:
                // резкая граница оставляла бы на панораме видимый контур тени.
                const float deficit = bestLuma_[idx] - l;
                float factor = 1.f;
                if (deficit > threshold) {
                    continue;
                } else if (deficit > threshold * 0.5f) {
                    factor = 1.f - (deficit - threshold * 0.5f) / (threshold * 0.5f);
                    if (factor <= 0.001f) continue;
                }

                const float wgt = e.weight * factor;
                accumR_[idx] += r * wgt;
                accumG_[idx] += g * wgt;
                accumB_[idx] += b * wgt;
                accumW_[idx] += wgt;
            }
        }

        for (int y = y0; y < y1; ++y) {
            for (int x = 0; x < w; ++x) {
                const size_t idx = static_cast<size_t>(y) * w + x;
                const float wsum = accumW_[idx];
                if (wsum > 1e-4f) {
                    out.set(x, y,
                            RGB{accumR_[idx] / wsum, accumG_[idx] / wsum, accumB_[idx] / wsum});
                } else {
                    out.set(x, y, cfg_.background);
                }
            }
        }
    };

    if (bandCount == 1) {
        processBand(0);
    } else {
        std::vector<std::thread> workers;
        workers.reserve(static_cast<size_t>(bandCount));
        for (int b = 0; b < bandCount; ++b) workers.emplace_back(processBand, b);
        for (std::thread& t : workers) t.join();
    }

    stats_.gains = gains;
    return true;
}

Image Stitcher::coverageMap() const {
    Image img(cfg_.canvasWidth, cfg_.canvasHeight);
    if (!configured()) return img;

    for (int y = 0; y < cfg_.canvasHeight; ++y) {
        for (int x = 0; x < cfg_.canvasWidth; ++x) {
            const uint8_t n = coverage_[static_cast<size_t>(y) * cfg_.canvasWidth + x];
            // Чёрный - слепая зона, синий - одна камера, зелёный - перекрытие.
            if (n == 0) {
                img.set(x, y, RGB{20.f, 20.f, 24.f});
            } else if (n == 1) {
                img.set(x, y, RGB{40.f, 90.f, 190.f});
            } else {
                img.set(x, y, RGB{60.f, 190.f, 110.f});
            }
        }
    }
    return img;
}

}  // namespace wz
