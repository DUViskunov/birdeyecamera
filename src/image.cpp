#include "wz/image.hpp"

#include <cmath>
#include <cstdio>

#include "stb_image.h"
#include "stb_image_write.h"

namespace wz {

bool Image::sampleBilinear(float x, float y, RGB& out) const {
    if (empty()) return false;
    // Полупиксельный допуск по краям не нужен: соседние камеры всё равно
    // перекрываются, а строгая граница избавляет от каймы на швах.
    if (x < 0.f || y < 0.f || x > static_cast<float>(w_ - 1) ||
        y > static_cast<float>(h_ - 1)) {
        return false;
    }

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, w_ - 1);
    const int y1 = std::min(y0 + 1, h_ - 1);

    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);

    const uint8_t* p00 = pixel(x0, y0);
    const uint8_t* p10 = pixel(x1, y0);
    const uint8_t* p01 = pixel(x0, y1);
    const uint8_t* p11 = pixel(x1, y1);

    const float w00 = (1.f - fx) * (1.f - fy);
    const float w10 = fx * (1.f - fy);
    const float w01 = (1.f - fx) * fy;
    const float w11 = fx * fy;

    out.r = p00[0] * w00 + p10[0] * w10 + p01[0] * w01 + p11[0] * w11;
    out.g = p00[1] * w00 + p10[1] * w10 + p01[1] * w01 + p11[1] * w11;
    out.b = p00[2] * w00 + p10[2] * w10 + p01[2] * w01 + p11[2] * w11;
    return true;
}

bool Image::sampleNearest(float x, float y, RGB& out) const {
    if (empty()) return false;
    const int xi = static_cast<int>(std::lround(x));
    const int yi = static_cast<int>(std::lround(y));
    if (!inside(xi, yi)) return false;
    out = at(xi, yi);
    return true;
}

namespace {

// Общий путь для stbi_load* : принудительно 3 канала, чтобы дальше по конвейеру
// не приходилось различать градации серого, RGB и RGBA.
bool adoptStbBuffer(unsigned char* raw, int w, int h, Image& out, std::string* error) {
    if (!raw) {
        if (error && error->empty()) {
            const char* reason = stbi_failure_reason();
            *error = reason ? reason : "неизвестная ошибка декодирования";
        }
        return false;
    }
    Image img(w, h);
    std::copy(raw, raw + static_cast<size_t>(w) * h * 3, img.data());
    stbi_image_free(raw);
    out = std::move(img);
    return true;
}

}  // namespace

bool loadImageFile(const std::string& path, Image& out, std::string* error) {
    int w = 0, h = 0, comp = 0;
    unsigned char* raw = stbi_load(path.c_str(), &w, &h, &comp, 3);
    if (!raw && error) *error = "не удалось открыть или декодировать " + path;
    return adoptStbBuffer(raw, w, h, out, error);
}

bool loadImageFromMemory(const uint8_t* bytes, size_t len, Image& out, std::string* error) {
    if (!bytes || len == 0) {
        if (error) *error = "пустой буфер";
        return false;
    }
    int w = 0, h = 0, comp = 0;
    unsigned char* raw = stbi_load_from_memory(bytes, static_cast<int>(len), &w, &h, &comp, 3);
    return adoptStbBuffer(raw, w, h, out, error);
}

bool saveImagePng(const std::string& path, const Image& img, std::string* error) {
    if (img.empty()) {
        if (error) *error = "пустое изображение";
        return false;
    }
    const int ok = stbi_write_png(path.c_str(), img.width(), img.height(), 3, img.data(),
                                  img.width() * 3);
    if (!ok && error) *error = "не удалось записать " + path;
    return ok != 0;
}

bool saveImageJpeg(const std::string& path, const Image& img, int quality, std::string* error) {
    if (img.empty()) {
        if (error) *error = "пустое изображение";
        return false;
    }
    const int ok = stbi_write_jpg(path.c_str(), img.width(), img.height(), 3, img.data(), quality);
    if (!ok && error) *error = "не удалось записать " + path;
    return ok != 0;
}

bool encodeJpeg(const Image& img, int quality, std::vector<uint8_t>& out) {
    out.clear();
    if (img.empty()) return false;

    // stb отдаёт сжатые данные порциями через колбэк - копим их в вектор.
    auto sink = [](void* ctx, void* data, int size) {
        auto* buf = static_cast<std::vector<uint8_t>*>(ctx);
        const auto* bytes = static_cast<const uint8_t*>(data);
        buf->insert(buf->end(), bytes, bytes + size);
    };

    const int ok = stbi_write_jpg_to_func(sink, &out, img.width(), img.height(), 3, img.data(),
                                          quality);
    return ok != 0 && !out.empty();
}

}  // namespace wz
