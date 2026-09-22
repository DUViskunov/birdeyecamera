#include "wz/sync.hpp"

#include <vector>

#include "test_common.hpp"

using namespace wz;

namespace {

Frame makeFrame(int sourceIndex, int64_t timestampUs, uint64_t seq) {
    Frame f;
    f.image = Image(4, 4);  // содержимое неважно, важны метки времени
    f.sourceIndex = sourceIndex;
    f.timestampUs = timestampUs;
    f.sequence = seq;
    return f;
}

void testMatchesWithinTolerance() {
    SyncConfig cfg;
    cfg.toleranceUs = 20000;
    FrameSynchronizer sync(2, cfg);

    std::vector<Frame> out;
    // Один источник заполнен - набора ещё нет.
    sync.push(makeFrame(0, 0, 0));
    CHECK(!sync.tryPop(out));

    // Второй приходит с задержкой 12 мс - это в пределах допуска.
    sync.push(makeFrame(1, 12000, 0));
    int64_t spread = -1;
    CHECK(sync.tryPop(out, &spread));
    CHECK(out.size() == 2);
    CHECK(spread == 12000);
    CHECK(out[0].sourceIndex == 0);
    CHECK(out[1].sourceIndex == 1);

    const SyncStats s = sync.stats();
    CHECK(s.matched == 1);
    CHECK_NEAR(s.meanSpreadUs, 12000.0, 1.0);
}

void testRejectsBeyondTolerance() {
    SyncConfig cfg;
    cfg.toleranceUs = 5000;
    FrameSynchronizer sync(2, cfg);

    sync.push(makeFrame(0, 0, 0));
    sync.push(makeFrame(1, 40000, 0));

    std::vector<Frame> out;
    // Расхождение 40 мс больше допуска: набор не выдаётся, кадр источника 0
    // остаётся ждать более свежей пары.
    CHECK(!sync.tryPop(out));

    // Приходит кадр источника 0, близкий по времени ко второму источнику.
    sync.push(makeFrame(0, 38000, 1));
    CHECK(sync.tryPop(out));
    CHECK(out.size() == 2);
    CHECK(out[0].timestampUs == 38000);
    CHECK(out[1].timestampUs == 40000);

    // Устаревший кадр должен быть отброшен, а не выдан в паре.
    CHECK(sync.stats().droppedStale >= 1);
}

void testOverflowDropsOldest() {
    SyncConfig cfg;
    cfg.toleranceUs = 1000;
    cfg.bufferDepth = 3;
    FrameSynchronizer sync(2, cfg);

    for (int i = 0; i < 10; ++i) {
        sync.push(makeFrame(0, i * 33000, static_cast<uint64_t>(i)));
    }

    const SyncStats s = sync.stats();
    CHECK(s.pushed == 10);
    CHECK(s.droppedOverflow == 7);  // в буфере осталось три последних кадра
}

void testIgnoresUnknownSource() {
    SyncConfig cfg;
    FrameSynchronizer sync(2, cfg);

    sync.push(makeFrame(5, 0, 0));   // индекса 5 не существует
    sync.push(makeFrame(-1, 0, 0));  // кадр без источника
    CHECK(sync.stats().pushed == 0);
}

void testSteadyStreamStaysMatched() {
    // Установившийся режим: оба источника идут по 30 кадр/с, правый смещён
    // на 12 мс. Каждый кадр должен находить пару.
    SyncConfig cfg;
    cfg.toleranceUs = 20000;
    cfg.bufferDepth = 8;
    FrameSynchronizer sync(2, cfg);

    const int64_t period = 33333;
    std::vector<Frame> out;
    int matched = 0;

    for (int i = 0; i < 50; ++i) {
        sync.push(makeFrame(0, i * period, static_cast<uint64_t>(i)));
        sync.push(makeFrame(1, i * period + 12000, static_cast<uint64_t>(i)));
        while (sync.tryPop(out)) ++matched;
    }

    CHECK(matched >= 49);
    CHECK(sync.stats().droppedOverflow == 0);
}

}  // namespace

int main() {
    testMatchesWithinTolerance();
    testRejectsBeyondTolerance();
    testOverflowDropsOldest();
    testIgnoresUnknownSource();
    testSteadyStreamStaysMatched();
    return wztest::report("sync");
}
