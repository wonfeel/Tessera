#include "engine/core/DefaultApplication.h"
#include "engine/core/TaskScheduler.h"
#include "engine/utils/RleLoader.h"
#include "demo/minimallife_demo/MiniLifeTileMap.h"
#include <memory>
#include <chrono>
#include <cstdio>

int main() {
    TaskScheduler::instance().initialize();

    auto factory = []() -> std::unique_ptr<ChunkedTileMap> {
        auto map = std::make_unique<MiniLifeTileMap>(0x400, 0x400, 18.0f, 256);

        // Загружаем пушку Госпера из файла.
        // Если файл не найден — fallback на случайный шум.
        RlePattern gun = RleLoader::load("patterns/gosper_gun.rle");
        if (gun.width > 0) {
            std::fprintf(stderr, "[demo] Loaded '%s' (%dx%d, rule=%s)\n",
                         "gosper_gun.rle", gun.width, gun.height,
                         gun.ruleName.c_str());

            // Расставляем пушки сеткой по всему полю с отступом.
            // Каждая пушка независима и будет бесконечно стрелять глайдерами.
            const int stepX = gun.width  + 60;
            const int stepY = gun.height + 60;
            int count = 0;
            for (int gy = 20; gy + gun.height < map->getHeight() - 20; gy += stepY) {
                for (int gx = 20; gx + gun.width < map->getWidth() - 20; gx += stepX) {
                    map->stampPattern(gun, gx, gy);
                    ++count;
                }
            }
            std::fprintf(stderr, "[demo] Stamped %d Gosper guns\n", count);
        } else {
            std::fprintf(stderr, "[demo] patterns/gosper_gun.rle not found — random init\n");
            map->randomize(0.3f);
        }

        // Переносим начальное состояние из simBuffer в renderBuffer и грузим в GL,
        // иначе первый кадр будет пустым (чёрным).
        map->commitInitialState();
        return map;
    };

    DefaultApplication app(factory, std::chrono::milliseconds(0), 1920, 1080,
        "Minimal Life — Gosper Glider Gun", true);
    app.run();

    TaskScheduler::instance().shutdown();
    return 0;
}
