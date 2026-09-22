// Кадр с меткой времени - единица обмена между источником и конвейером.
#pragma once

#include <cstdint>
#include <string>

#include "wz/image.hpp"

namespace wz {

struct Frame {
    Image image;
    int64_t timestampUs = 0;  // монотонное время захвата, микросекунды
    uint64_t sequence = 0;    // номер кадра внутри источника
    int sourceIndex = -1;     // позиция источника в конфигурации

    bool valid() const { return !image.empty(); }
};

// Монотонные часы процесса в микросекундах. Едины для всех источников,
// иначе временная синхронизация теряет смысл.
int64_t nowMicros();

}  // namespace wz
