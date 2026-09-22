// Источники видео: подменяемый вход конвейера.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "wz/frame.hpp"

namespace wz {

struct SourceConfig {
    std::string id = "cam";
    // synthetic - генератор сцены, jpegdir - каталог кадров,
    // mjpeg - файл с потоком JPEG, camera - реальное устройство.
    std::string type = "synthetic";
    std::string path;
    int width = 640;
    int height = 480;
    double fps = 30.0;
    bool loop = true;
    // Искусственный сдвиг меток времени: так проверяется работа синхронизатора.
    int64_t startOffsetUs = 0;
    // Для synthetic: индекс виртуальной камеры в сцене (0 или 1).
    int syntheticView = 0;
};

class ISource {
public:
    virtual ~ISource() = default;

    virtual bool open(std::string* error) = 0;
    // Блокирующее чтение с выдержкой темпа fps. false - поток закончился.
    virtual bool read(Frame& out) = 0;
    virtual void close() = 0;

    // Прерывает висящий read(). Блокирующее чтение с устройства иначе
    // не отпустит поток, и остановка конвейера подвиснет навсегда.
    virtual void cancel() {}

    virtual const SourceConfig& config() const = 0;
    virtual uint64_t framesRead() const = 0;
};

// Создаёт источник по типу из конфигурации. Неизвестный тип - ошибка.
std::unique_ptr<ISource> createSource(const SourceConfig& cfg, int sourceIndex,
                                      std::string* error);

// Файлы изображений каталога, отсортированные по имени. Тот же обход,
// что у источника jpegdir, - чтобы калибровка видела кадры в том же порядке.
std::vector<std::string> listImageFiles(const std::string& dir);

// Список типов, поддерживаемых сборкой (для подсказок в CLI).
std::vector<std::string> availableSourceTypes();

// Найденное в системе устройство захвата.
struct CameraInfo {
    int index = -1;
    std::string name;
    std::string symbolicLink;  // устойчив между запусками, в отличие от индекса
    std::string modes;         // краткая сводка поддерживаемых режимов
};

// Перечисление камер. Пустой список без ошибки означает, что камер нет.
std::vector<CameraInfo> listCameras(std::string* error = nullptr);

// Источник с реального устройства. SourceConfig::path задаёт либо индекс
// ("0", "1"), либо подстроку символьной ссылки - её видно в listCameras().
std::unique_ptr<ISource> createCameraSource(const SourceConfig& cfg, int sourceIndex,
                                            std::string* error);

}  // namespace wz
