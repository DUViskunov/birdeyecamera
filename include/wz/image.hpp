// Растровое изображение RGB8 и операции выборки.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace wz {

struct RGB {
    float r = 0.f, g = 0.f, b = 0.f;
};

inline uint8_t clamp8(float v) {
    return static_cast<uint8_t>(std::min(255.f, std::max(0.f, v + 0.5f)));
}

// Изображение с чередующимися каналами RGB, 8 бит на канал.
class Image {
public:
    Image() = default;
    Image(int w, int h) : w_(w), h_(h), data_(static_cast<size_t>(w) * h * 3, 0) {}

    int width() const { return w_; }
    int height() const { return h_; }
    bool empty() const { return w_ <= 0 || h_ <= 0 || data_.empty(); }
    size_t byteSize() const { return data_.size(); }

    uint8_t* data() { return data_.data(); }
    const uint8_t* data() const { return data_.data(); }

    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }

    uint8_t* pixel(int x, int y) {
        return data_.data() + (static_cast<size_t>(y) * w_ + x) * 3;
    }
    const uint8_t* pixel(int x, int y) const {
        return data_.data() + (static_cast<size_t>(y) * w_ + x) * 3;
    }

    RGB at(int x, int y) const {
        const uint8_t* p = pixel(x, y);
        return RGB{static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
    }

    void set(int x, int y, const RGB& c) {
        uint8_t* p = pixel(x, y);
        p[0] = clamp8(c.r);
        p[1] = clamp8(c.g);
        p[2] = clamp8(c.b);
    }

    void fill(const RGB& c) {
        const uint8_t r = clamp8(c.r), g = clamp8(c.g), b = clamp8(c.b);
        for (size_t i = 0; i + 2 < data_.size(); i += 3) {
            data_[i] = r;
            data_[i + 1] = g;
            data_[i + 2] = b;
        }
    }

    // Билинейная выборка. Возвращает false, если точка вне изображения:
    // вызывающий код сам решает, что делать с непокрытой областью.
    bool sampleBilinear(float x, float y, RGB& out) const;

    // Ближайший сосед — используется при отладке карт переноса.
    bool sampleNearest(float x, float y, RGB& out) const;

private:
    int w_ = 0;
    int h_ = 0;
    std::vector<uint8_t> data_;
};

// Загрузка/сохранение через stb (JPEG, PNG, BMP, TGA).
bool loadImageFile(const std::string& path, Image& out, std::string* error = nullptr);
bool loadImageFromMemory(const uint8_t* bytes, size_t len, Image& out, std::string* error = nullptr);
bool saveImagePng(const std::string& path, const Image& img, std::string* error = nullptr);
bool saveImageJpeg(const std::string& path, const Image& img, int quality, std::string* error = nullptr);

// Кодирование в JPEG в память — основа MJPEG-потока.
bool encodeJpeg(const Image& img, int quality, std::vector<uint8_t>& out);

}  // namespace wz
