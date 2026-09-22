#include "wz/stitcher.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include "test_common.hpp"
#include "wz/config.hpp"
#include "wz/synthetic.hpp"

using namespace wz;

namespace {

float luma(const RGB& c) { return 0.299f * c.r + 0.587f * c.g + 0.114f * c.b; }

// Самый яркий пиксель в окрестности точки: метка занимает несколько пикселей,
// а точное попадание в центр зависит от округления.
float brightestNear(const Image& img, double cx, double cy, int radius) {
    float best = -1.f;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            const int x = static_cast<int>(std::lround(cx)) + dx;
            const int y = static_cast<int>(std::lround(cy)) + dy;
            if (!img.inside(x, y)) continue;
            best = std::max(best, luma(img.at(x, y)));
        }
    }
    return best;
}

std::vector<Frame> renderFrames(const SystemConfig& cfg, double tSeconds) {
    std::vector<Frame> frames;
    for (size_t i = 0; i < cfg.cameras.size(); ++i) {
        const SyntheticView sv = syntheticView(static_cast<int>(i), cfg.cameras[i].imageWidth,
                                               cfg.cameras[i].imageHeight);
        Frame f;
        renderSyntheticFrame(sv, tSeconds, f.image);
        f.sourceIndex = static_cast<int>(i);
        f.timestampUs = static_cast<int64_t>(tSeconds * 1e6);
        frames.push_back(std::move(f));
    }
    return frames;
}

void testCoverageAndOverlap() {
    const SystemConfig cfg = SystemConfig::defaults();

    Stitcher stitcher;
    std::string err;
    CHECK(stitcher.configure(cfg.stitch, cfg.cameras, &err));

    const StitchStats s = stitcher.stats();
    // Две камеры должны покрывать заметную часть холста и перекрываться:
    // именно перекрытие закрывает зону, которую прячет голова лазера.
    CHECK(s.coverage > 0.5);
    CHECK(s.overlap > 0.1);
    CHECK(s.overlap < s.coverage);
}

// Сквозная проверка геометрии: метки, чьи координаты на столе известны,
// обязаны оказаться в панораме там, куда их переводит planeToCanvas.
void testMarkersLandWhereExpected() {
    const SystemConfig cfg = SystemConfig::defaults();
    const SceneGeometry scene = sceneGeometry();
    const Mat3 toCanvas = planeToCanvas(cfg.stitch, scene.plateWidthMm, scene.plateHeightMm);

    Stitcher stitcher;
    std::string err;
    CHECK(stitcher.configure(cfg.stitch, cfg.cameras, &err));

    // t = 0.3 с: голова ещё в начале прохода и углы стола не перекрывает.
    const std::vector<Frame> frames = renderFrames(cfg, 0.3);

    Image panorama;
    CHECK(stitcher.blend(frames, panorama));
    CHECK(panorama.width() == cfg.stitch.canvasWidth);
    CHECK(panorama.height() == cfg.stitch.canvasHeight);

    int found = 0;
    for (const Vec2& marker : sceneMarkersMm()) {
        bool ok = false;
        const Vec2 expected = toCanvas.project(marker, &ok);
        if (!ok) continue;
        if (!panorama.inside(static_cast<int>(expected.x), static_cast<int>(expected.y))) continue;

        const float atMarker = brightestNear(panorama, expected.x, expected.y, 6);
        // Фон берём в стороне: сравнение с ним не зависит от экспозиции камеры.
        const float background = brightestNear(panorama, expected.x + 46.0, expected.y, 3);

        if (atMarker > background + 25.f) ++found;
    }

    // Метки светлые и крупные: если геометрия верна, находится хотя бы
    // большинство из них (часть может выпадать за край кадра камеры).
    CHECK(found >= 4);
}

void testBlendRejectsWrongFrameCount() {
    const SystemConfig cfg = SystemConfig::defaults();

    Stitcher stitcher;
    std::string err;
    CHECK(stitcher.configure(cfg.stitch, cfg.cameras, &err));

    std::vector<Frame> tooFew(1);
    Image out;
    CHECK(!stitcher.blend(tooFew, out));

    Stitcher unconfigured;
    CHECK(!unconfigured.blend(tooFew, out));
}

void testConfigureRejectsDegenerateHomography() {
    SystemConfig cfg = SystemConfig::defaults();
    cfg.cameras[0].homography = Mat3::zero();

    Stitcher stitcher;
    std::string err;
    CHECK(!stitcher.configure(cfg.stitch, cfg.cameras, &err));
    CHECK(!err.empty());
}

void testExposureCompensationEqualisesCameras() {
    SystemConfig cfg = SystemConfig::defaults();
    const std::vector<Frame> frames = renderFrames(cfg, 0.3);

    Stitcher stitcher;
    std::string err;
    CHECK(stitcher.configure(cfg.stitch, cfg.cameras, &err));

    Image out;
    CHECK(stitcher.blend(frames, out));

    const StitchStats s = stitcher.stats();
    CHECK(s.gains.size() == 2);
    CHECK_NEAR(s.gains[0], 1.0, 1e-9);  // опорная камера не трогается
    // Правая камера темнее опорной, значит её нужно подтянуть вверх.
    CHECK(s.gains[1] > 1.0);
    CHECK(s.gains[1] < 1.6);
}


// Ради этого в отчёте и ставится вторая камера: то, что закрыто головой лазера
// в одном виде, должно подхватываться из другого, а не усредняться с тенью.
void testOcclusionRejectionRecoversHiddenArea() {
    SystemConfig cfg = SystemConfig::defaults();
    const std::vector<Frame> frames = renderFrames(cfg, 1.4);

    // Считаем только внутри стола: чёрный фон за его краями одинаков в обоих
    // режимах и размыл бы разницу.
    const SceneGeometry scene = sceneGeometry();
    const Mat3 toCanvas = planeToCanvas(cfg.stitch, scene.plateWidthMm, scene.plateHeightMm);
    const Vec2 tl = toCanvas.project(Vec2{0.0, 0.0});
    const Vec2 br = toCanvas.project(Vec2{scene.plateWidthMm, scene.plateHeightMm});

    // Тень от головы в усреднённой панораме - это полутон (примерно середина
    // между тёмной головой и светлым столом), поэтому меряем среднюю яркость
    // стола, а не число совсем тёмных пикселей.
    auto meanPlateLuma = [&](const Image& img) {
        double sum = 0.0;
        int count = 0;
        for (int y = static_cast<int>(tl.y) + 2; y < static_cast<int>(br.y) - 2; ++y) {
            for (int x = static_cast<int>(tl.x) + 2; x < static_cast<int>(br.x) - 2; ++x) {
                if (!img.inside(x, y)) continue;
                sum += luma(img.at(x, y));
                ++count;
            }
        }
        return count > 0 ? sum / count : 0.0;
    };

    Image withRejection, withoutRejection;
    std::string err;

    cfg.stitch.occlusionRejection = true;
    Stitcher a;
    CHECK(a.configure(cfg.stitch, cfg.cameras, &err));
    CHECK(a.blend(frames, withRejection));

    cfg.stitch.occlusionRejection = false;
    Stitcher b;
    CHECK(b.configure(cfg.stitch, cfg.cameras, &err));
    CHECK(b.blend(frames, withoutRejection));

    const double withRej = meanPlateLuma(withRejection);
    const double withoutRej = meanPlateLuma(withoutRejection);
    std::printf("    средняя яркость стола: с отбраковкой %.1f, без неё %.1f\n",
                withRej, withoutRej);

    // Простое усреднение размазывает тень головы по панораме и делает стол
    // темнее; отбраковка должна вернуть яркость.
    // Измеренный разрыв около 5 единиц; порог 3 оставляет запас на шум.
    CHECK(withRej > withoutRej + 3.0);
}

}  // namespace

int main() {
    testCoverageAndOverlap();
    testMarkersLandWhereExpected();
    testBlendRejectsWrongFrameCount();
    testConfigureRejectsDegenerateHomography();
    testExposureCompensationEqualisesCameras();
    testOcclusionRejectionRecoversHiddenArea();
    return wztest::report("stitch");
}
