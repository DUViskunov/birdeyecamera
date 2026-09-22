#include "wz/config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "wz/json.hpp"
#include "wz/synthetic.hpp"

namespace wz {
namespace {

JsonValue matToJson(const Mat3& m) {
    JsonValue a = JsonValue::array();
    for (double v : m.m) a.push(JsonValue(v));
    return a;
}

bool matFromJson(const JsonValue* v, Mat3& out) {
    if (!v || !v->isArray() || v->items().size() != 9) return false;
    for (size_t i = 0; i < 9; ++i) out.m[i] = v->items()[i].asNumber(0.0);
    return true;
}

JsonValue intrinsicsToJson(const Intrinsics& in) {
    JsonValue o = JsonValue::object();
    o.set("fx", JsonValue(in.fx));
    o.set("fy", JsonValue(in.fy));
    o.set("cx", JsonValue(in.cx));
    o.set("cy", JsonValue(in.cy));
    o.set("k1", JsonValue(in.k1));
    o.set("k2", JsonValue(in.k2));
    o.set("k3", JsonValue(in.k3));
    o.set("p1", JsonValue(in.p1));
    o.set("p2", JsonValue(in.p2));
    return o;
}

Intrinsics intrinsicsFromJson(const JsonValue* v) {
    Intrinsics in;
    if (!v || !v->isObject()) return in;
    in.fx = v->numberAt("fx", 0.0);
    in.fy = v->numberAt("fy", 0.0);
    in.cx = v->numberAt("cx", 0.0);
    in.cy = v->numberAt("cy", 0.0);
    in.k1 = v->numberAt("k1", 0.0);
    in.k2 = v->numberAt("k2", 0.0);
    in.k3 = v->numberAt("k3", 0.0);
    in.p1 = v->numberAt("p1", 0.0);
    in.p2 = v->numberAt("p2", 0.0);
    return in;
}

}  // namespace

void ProcessingConfig::clampToValidRange() {
    brightness = std::min(100.0f, std::max(-100.0f, brightness));
    contrast = std::min(3.0f, std::max(0.2f, contrast));
    saturation = std::min(3.0f, std::max(0.0f, saturation));
    jpegQuality = std::min(100, std::max(1, jpegQuality));

    cropX = std::min(0.95f, std::max(0.0f, cropX));
    cropY = std::min(0.95f, std::max(0.0f, cropY));
    cropW = std::min(1.0f - cropX, std::max(0.05f, cropW));
    cropH = std::min(1.0f - cropY, std::max(0.05f, cropH));

    if (outputWidth < 0) outputWidth = 0;
    if (outputHeight < 0) outputHeight = 0;
    outputWidth = std::min(outputWidth, 4096);
    outputHeight = std::min(outputHeight, 4096);
}

Mat3 planeToCanvas(const StitchConfig& stitch, double plateWidthMm, double plateHeightMm) {
    if (plateWidthMm < 1e-6 || plateHeightMm < 1e-6) return Mat3::identity();

    // Поле в 4% с каждой стороны: край стола должен быть виден целиком.
    const double margin = 0.04;
    const double usableW = stitch.canvasWidth * (1.0 - 2.0 * margin);
    const double usableH = stitch.canvasHeight * (1.0 - 2.0 * margin);

    const double scale = std::min(usableW / plateWidthMm, usableH / plateHeightMm);
    const double offsetX = (stitch.canvasWidth - plateWidthMm * scale) * 0.5;
    const double offsetY = (stitch.canvasHeight - plateHeightMm * scale) * 0.5;

    return Mat3::translation(offsetX, offsetY) * Mat3::scaling(scale, scale);
}

SystemConfig SystemConfig::defaults() {
    SystemConfig cfg;

    cfg.stitch.canvasWidth = 1280;
    cfg.stitch.canvasHeight = 720;
    cfg.stitch.featherPx = 48.0f;
    cfg.stitch.exposureCompensation = true;

    cfg.sync.toleranceUs = 20000;
    cfg.sync.bufferDepth = 8;

    const SceneGeometry scene = sceneGeometry();
    const Mat3 toCanvas = planeToCanvas(cfg.stitch, scene.plateWidthMm, scene.plateHeightMm);

    for (int view = 0; view < 2; ++view) {
        SourceConfig src;
        src.id = view == 0 ? "cam_left" : "cam_right";
        src.type = "synthetic";
        src.width = 640;  // как в отчёте: 640x480, 30 кадр/с, MJPEG
        src.height = 480;
        src.fps = 30.0;
        src.syntheticView = view;
        // Правая камера намеренно отстаёт на 12 мс: синхронизатору есть что чинить.
        src.startOffsetUs = view == 0 ? 0 : 12000;
        cfg.sources.push_back(src);

        const SyntheticView sv = syntheticView(view, src.width, src.height);

        CameraPlacement cam;
        cam.sourceId = src.id;
        cam.imageWidth = src.width;
        cam.imageHeight = src.height;
        cam.intrinsics = sv.intrinsics;
        // Кадр камеры -> плоскость стола -> холст "вид сверху".
        cam.homography = toCanvas * sv.imageToPlane;
        cam.homography.normalizeH();
        cfg.cameras.push_back(cam);
    }

    return cfg;
}

bool SystemConfig::validate(std::string* error) const {
    if (sources.empty()) {
        if (error) *error = "не задан ни один источник";
        return false;
    }
    if (sources.size() != cameras.size()) {
        if (error) {
            *error = "число камер (" + std::to_string(cameras.size()) +
                     ") не совпадает с числом источников (" + std::to_string(sources.size()) + ")";
        }
        return false;
    }
    if (stitch.canvasWidth < 64 || stitch.canvasHeight < 64 || stitch.canvasWidth > 7680 ||
        stitch.canvasHeight > 4320) {
        if (error) *error = "размер холста вне допустимого диапазона (64..7680 x 64..4320)";
        return false;
    }
    if (server.port < 1 || server.port > 65535) {
        if (error) *error = "недопустимый порт: " + std::to_string(server.port);
        return false;
    }

    for (size_t i = 0; i < sources.size(); ++i) {
        if (sources[i].width <= 0 || sources[i].height <= 0) {
            if (error) *error = "источник " + sources[i].id + ": недопустимый размер кадра";
            return false;
        }
        if (sources[i].fps <= 0.0) {
            if (error) *error = "источник " + sources[i].id + ": частота кадров должна быть > 0";
            return false;
        }
        // Порядок камер обязан совпадать с порядком источников: склейка
        // сопоставляет их по индексу, а не по имени.
        if (!cameras[i].sourceId.empty() && cameras[i].sourceId != sources[i].id) {
            if (error) {
                *error = "камера " + std::to_string(i) + " описывает источник '" +
                         cameras[i].sourceId + "', а по порядку идёт '" + sources[i].id + "'";
            }
            return false;
        }
        Mat3 tmp;
        if (!cameras[i].homography.inverse(tmp)) {
            if (error) *error = "камера " + cameras[i].sourceId + ": вырожденная гомография";
            return false;
        }
    }
    return true;
}

std::string SystemConfig::toJson() const {
    JsonValue root = JsonValue::object();

    JsonValue srcArray = JsonValue::array();
    for (const SourceConfig& s : sources) {
        JsonValue o = JsonValue::object();
        o.set("id", JsonValue(s.id));
        o.set("type", JsonValue(s.type));
        o.set("path", JsonValue(s.path));
        o.set("width", JsonValue(s.width));
        o.set("height", JsonValue(s.height));
        o.set("fps", JsonValue(s.fps));
        o.set("loop", JsonValue(s.loop));
        o.set("startOffsetUs", JsonValue(static_cast<double>(s.startOffsetUs)));
        o.set("syntheticView", JsonValue(s.syntheticView));
        srcArray.push(std::move(o));
    }
    root.set("sources", std::move(srcArray));

    JsonValue camArray = JsonValue::array();
    for (const CameraPlacement& c : cameras) {
        JsonValue o = JsonValue::object();
        o.set("sourceId", JsonValue(c.sourceId));
        o.set("imageWidth", JsonValue(c.imageWidth));
        o.set("imageHeight", JsonValue(c.imageHeight));
        o.set("intrinsics", intrinsicsToJson(c.intrinsics));
        o.set("homography", matToJson(c.homography));
        camArray.push(std::move(o));
    }
    root.set("cameras", std::move(camArray));

    JsonValue st = JsonValue::object();
    st.set("canvasWidth", JsonValue(stitch.canvasWidth));
    st.set("canvasHeight", JsonValue(stitch.canvasHeight));
    st.set("featherPx", JsonValue(stitch.featherPx));
    st.set("exposureCompensation", JsonValue(stitch.exposureCompensation));
    st.set("occlusionRejection", JsonValue(stitch.occlusionRejection));
    st.set("occlusionThreshold", JsonValue(stitch.occlusionThreshold));
    st.set("threads", JsonValue(stitch.threads));
    root.set("stitch", std::move(st));

    JsonValue sy = JsonValue::object();
    sy.set("toleranceUs", JsonValue(static_cast<double>(sync.toleranceUs)));
    sy.set("bufferDepth", JsonValue(static_cast<double>(sync.bufferDepth)));
    root.set("sync", std::move(sy));

    JsonValue sv = JsonValue::object();
    sv.set("bindAddress", JsonValue(server.bindAddress));
    sv.set("port", JsonValue(server.port));
    sv.set("webRoot", JsonValue(server.webRoot));
    sv.set("maxClients", JsonValue(server.maxClients));
    root.set("server", std::move(sv));

    JsonValue pr = JsonValue::object();
    pr.set("brightness", JsonValue(processing.brightness));
    pr.set("contrast", JsonValue(processing.contrast));
    pr.set("saturation", JsonValue(processing.saturation));
    pr.set("jpegQuality", JsonValue(processing.jpegQuality));
    pr.set("cropX", JsonValue(processing.cropX));
    pr.set("cropY", JsonValue(processing.cropY));
    pr.set("cropW", JsonValue(processing.cropW));
    pr.set("cropH", JsonValue(processing.cropH));
    pr.set("outputWidth", JsonValue(processing.outputWidth));
    pr.set("outputHeight", JsonValue(processing.outputHeight));
    root.set("processing", std::move(pr));

    return root.dump(2);
}

bool SystemConfig::save(const std::string& path, std::string* error) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        if (error) *error = "не удалось открыть на запись " + path;
        return false;
    }
    out << toJson() << "\n";
    return true;
}

bool SystemConfig::load(const std::string& path, SystemConfig& out, std::string* error) {
    JsonValue root;
    if (!JsonValue::parseFile(path, root, error)) return false;
    if (!root.isObject()) {
        if (error) *error = "корень конфигурации должен быть объектом";
        return false;
    }

    // За основу берём значения по умолчанию: в файле достаточно указать только
    // отличия, и старые конфигурации не ломаются при добавлении новых полей.
    SystemConfig cfg = SystemConfig::defaults();

    const JsonValue* arr = root.find("sources");
    if (arr && arr->isArray()) {
        cfg.sources.clear();
        for (const JsonValue& v : arr->items()) {
            SourceConfig s;
            s.id = v.stringAt("id", "cam");
            s.type = v.stringAt("type", "synthetic");
            s.path = v.stringAt("path", "");
            s.width = static_cast<int>(v.numberAt("width", 640));
            s.height = static_cast<int>(v.numberAt("height", 480));
            s.fps = v.numberAt("fps", 30.0);
            s.loop = v.boolAt("loop", true);
            s.startOffsetUs = static_cast<int64_t>(v.numberAt("startOffsetUs", 0));
            s.syntheticView = static_cast<int>(v.numberAt("syntheticView", 0));
            cfg.sources.push_back(s);
        }
    }

    const JsonValue* camArr = root.find("cameras");
    if (camArr && camArr->isArray()) {
        cfg.cameras.clear();
        for (const JsonValue& v : camArr->items()) {
            CameraPlacement c;
            c.sourceId = v.stringAt("sourceId", "");
            c.imageWidth = static_cast<int>(v.numberAt("imageWidth", 640));
            c.imageHeight = static_cast<int>(v.numberAt("imageHeight", 480));
            c.intrinsics = intrinsicsFromJson(v.find("intrinsics"));
            if (!matFromJson(v.find("homography"), c.homography)) {
                if (error) {
                    *error = "камера '" + c.sourceId +
                             "': поле homography должно быть массивом из 9 чисел";
                }
                return false;
            }
            cfg.cameras.push_back(c);
        }
    }

    if (const JsonValue* st = root.find("stitch")) {
        cfg.stitch.canvasWidth =
            static_cast<int>(st->numberAt("canvasWidth", cfg.stitch.canvasWidth));
        cfg.stitch.canvasHeight =
            static_cast<int>(st->numberAt("canvasHeight", cfg.stitch.canvasHeight));
        cfg.stitch.featherPx = static_cast<float>(st->numberAt("featherPx", cfg.stitch.featherPx));
        cfg.stitch.exposureCompensation =
            st->boolAt("exposureCompensation", cfg.stitch.exposureCompensation);
        cfg.stitch.occlusionRejection =
            st->boolAt("occlusionRejection", cfg.stitch.occlusionRejection);
        cfg.stitch.occlusionThreshold = static_cast<float>(
            st->numberAt("occlusionThreshold", cfg.stitch.occlusionThreshold));
        cfg.stitch.threads = static_cast<int>(st->numberAt("threads", cfg.stitch.threads));
    }

    if (const JsonValue* sy = root.find("sync")) {
        cfg.sync.toleranceUs = static_cast<int64_t>(
            sy->numberAt("toleranceUs", static_cast<double>(cfg.sync.toleranceUs)));
        cfg.sync.bufferDepth = static_cast<size_t>(
            sy->numberAt("bufferDepth", static_cast<double>(cfg.sync.bufferDepth)));
    }

    if (const JsonValue* sv = root.find("server")) {
        cfg.server.bindAddress = sv->stringAt("bindAddress", cfg.server.bindAddress);
        cfg.server.port = static_cast<int>(sv->numberAt("port", cfg.server.port));
        cfg.server.webRoot = sv->stringAt("webRoot", cfg.server.webRoot);
        cfg.server.maxClients = static_cast<int>(sv->numberAt("maxClients", cfg.server.maxClients));
    }

    if (const JsonValue* pr = root.find("processing")) {
        ProcessingConfig& p = cfg.processing;
        p.brightness = static_cast<float>(pr->numberAt("brightness", p.brightness));
        p.contrast = static_cast<float>(pr->numberAt("contrast", p.contrast));
        p.saturation = static_cast<float>(pr->numberAt("saturation", p.saturation));
        p.jpegQuality = static_cast<int>(pr->numberAt("jpegQuality", p.jpegQuality));
        p.cropX = static_cast<float>(pr->numberAt("cropX", p.cropX));
        p.cropY = static_cast<float>(pr->numberAt("cropY", p.cropY));
        p.cropW = static_cast<float>(pr->numberAt("cropW", p.cropW));
        p.cropH = static_cast<float>(pr->numberAt("cropH", p.cropH));
        p.outputWidth = static_cast<int>(pr->numberAt("outputWidth", p.outputWidth));
        p.outputHeight = static_cast<int>(pr->numberAt("outputHeight", p.outputHeight));
        p.clampToValidRange();
    }

    if (!cfg.validate(error)) return false;
    out = std::move(cfg);
    return true;
}

}  // namespace wz
