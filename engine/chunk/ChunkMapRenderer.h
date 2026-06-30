// engine/chunk/ChunkMapRenderer.h
#pragma once
#include "engine/chunk/ChunkStore.h"
#include "engine/graphics/Camera2D.h"
#include <glm/glm.hpp>
#include <vector>

// Отрисовка чанковой карты: проходит по видимым (попадающим в кадр камеры)
// чанкам и рисует их, попутно обновляя GL-данные для "грязных" чанков.
// Вынесено из ChunkedTileMap — рисование это отдельная ответственность,
// не связанная ни с хранением чанков, ни с симуляцией.
class ChunkMapRenderer {
public:
    ChunkMapRenderer(ChunkStore& store, int chunkSize, float tileSize)
        : m_store(store), m_chunkSize(chunkSize), m_tileSize(tileSize) {}

    void render(const Camera2D& camera);

    // Установить глобальную палитру индексов цвета (общая для всех ChunkRenderer).
    static void setPalette(const std::vector<glm::vec3>& palette);

private:
    ChunkStore& m_store;
    int   m_chunkSize;
    float m_tileSize;
};
