// Конфигурация системы: источники, калибровка, склейка, сервер.
#pragma once

#include <string>
#include <vector>

#include "wz/source.hpp"
#include "wz/stitcher.hpp"
#include "wz/sync.hpp"

namespace wz {

// Параметры, которые оператор меняет прямо с веб-страницы.
struct ProcessingConfig {
    float brightness = 0.0f;   // сдвиг яркости, -100..100
    float contrast = 1.0f;     // множитель, 0.2..3.0
    float saturation = 1.0f;   // 0 - оттенки серого
    int jpegQuality = 80;      // 1..100

    // Кадрирование в долях холста: позволяет приблизить зону наплавки,
    // не трогая геометрию склейки.
    float cropX = 0.0f, cropY = 0.0f, cropW = 1.0f, cropH = 1.0f;

    int outputWidth = 0;   // 0 - размер холста после кадрирования
    int outputHeight = 0;

    bool showSeams = false;     // подсветить границы вклада камер
    bool showCoverage = false;  // карта покрытия вместо картинки

    void clampToValidRange();
};

struct ServerConfig {
    std::string bindAddress = "127.0.0.1";
    int port = 8080;
    std::string webRoot = "web";
    int maxClients = 8;
};

struct SystemConfig {
    std::vector<SourceConfig> sources;
    std::vector<CameraPlacement> cameras;  // по одному на источник, в том же порядке
    StitchConfig stitch;
    SyncConfig sync;
    ServerConfig server;
    ProcessingConfig processing;

    // Конфигурация по умолчанию: две синтетические камеры над рабочим столом.
    // Позволяет запустить систему без единого файла на диске.
    static SystemConfig defaults();

    static bool load(const std::string& path, SystemConfig& out, std::string* error);
    bool save(const std::string& path, std::string* error) const;
    std::string toJson() const;

    // Проверка согласованности: совпадение числа камер и источников,
    // вменяемые размеры холста и т.п.
    bool validate(std::string* error) const;
};

// Отображение плоскости стола (мм) в пиксели холста "вид сверху".
// Стол вписывается в холст целиком, с небольшим полем по краям.
Mat3 planeToCanvas(const StitchConfig& stitch, double plateWidthMm, double plateHeightMm);

}  // namespace wz
