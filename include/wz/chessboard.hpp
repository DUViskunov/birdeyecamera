// Поиск углов шахматной доски - вход для калибровки внутренних параметров.
#pragma once

#include <string>
#include <vector>

#include "wz/calibration.hpp"
#include "wz/geometry.hpp"
#include "wz/image.hpp"

namespace wz {

struct ChessboardSpec {
    // Число ВНУТРЕННИХ углов, а не клеток: доска 10x7 клеток даёт 9x6 углов.
    int cols = 9;
    int rows = 6;
    double squareMm = 25.0;

    int cornerCount() const { return cols * rows; }
    // Несимметричная доска (cols != rows) снимает неоднозначность поворота.
    bool valid() const { return cols >= 3 && rows >= 3 && cols != rows && squareMm > 0.1; }
};

// Углы в порядке строк: индекс = row * cols + col.
struct ChessboardCorners {
    std::vector<Vec2> image;  // пиксели сырого кадра
    std::vector<Vec2> board;  // миллиметры на плоскости доски
    std::string source;       // имя файла, для сообщений
};

// Автоматический поиск доски целиком. Частично найденная доска отбраковывается:
// неполный набор углов испортил бы гомографию этого вида.
bool findChessboard(const Image& img, const ChessboardSpec& spec, ChessboardCorners& out,
                    std::string* error = nullptr);

// Кандидаты в углы: седловые точки яркости. Вынесено в интерфейс, чтобы
// разбирать неудачный поиск, не влезая внутрь.
std::vector<Vec2> detectSaddlePoints(const Image& img, int maxCount = 600);

// Тестовый рендер: доска под ракурсом view, снятая камерой in.
// Возвращает истинную гомографию "доска (мм) -> идеальный кадр".
Mat3 renderChessboardView(int view, const ChessboardSpec& spec, const Intrinsics& in,
                          Image& out);

}  // namespace wz
