// Калибровка внутренних параметров камеры методом Чжана.
#pragma once

#include <string>
#include <vector>

#include "wz/calibration.hpp"
#include "wz/chessboard.hpp"

namespace wz {

struct IntrinsicsCalibResult {
    Intrinsics intrinsics;
    double rmsPx = 0.0;  // среднеквадратичная невязка перепроецирования
    double maxPx = 0.0;
    int views = 0;
    int points = 0;
    std::vector<double> perViewRmsPx;
    std::vector<double> iterationRmsPx;  // как сходились уточнения
};

// Нужно не менее трёх видов доски под РАЗНЫМИ наклонами: параллельные
// снимки дают вырожденную систему, и параметры не определятся.
bool calibrateIntrinsics(const std::vector<ChessboardCorners>& views, int imageWidth,
                         int imageHeight, IntrinsicsCalibResult& out, std::string* error);

std::string intrinsicsToJsonString(const Intrinsics& in);

}  // namespace wz
