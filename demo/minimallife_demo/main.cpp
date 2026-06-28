#include "engine/core/DefaultApplication.h"
#include "engine/core/TaskScheduler.h"
#include "demo/minimallife_demo/MiniLifeTileMap.h"
#include <memory>
#include <chrono>

int main() {
    TaskScheduler::instance().initialize();
    auto factory = []() -> std::unique_ptr<ChunkedTileMap> {
        auto map = std::make_unique<MiniLifeTileMap>(0x400, 0x400, 18.0f, 256);
        map->randomize(0.3f);
        return map;
        };
    DefaultApplication app(factory, std::chrono::milliseconds(0), 1920, 1080,
        "Minimal Life", true);
    app.run();
    TaskScheduler::instance().shutdown();
    return 0;
}