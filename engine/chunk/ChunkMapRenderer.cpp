// engine/chunk/ChunkMapRenderer.cpp
#include "engine/chunk/ChunkMapRenderer.h"
#include "engine/chunk/ChunkRenderer.h"
#include <shared_mutex>
#include <mutex>

void ChunkMapRenderer::render(const Camera2D& camera) {
    glm::vec2 viewMin, viewMax;
    camera.getVisibleAABB(viewMin, viewMax);
    float margin = m_chunkSize * m_tileSize * 2.0f;
    viewMin -= glm::vec2(margin, margin);
    viewMax += glm::vec2(margin, margin);
    AABB viewBounds{ viewMin, viewMax };

    std::shared_lock<std::shared_mutex> lock(m_store.mutex());
    for (auto& [coord, chunk] : m_store.map()) {
        if (!chunk || !chunk->renderer) continue;
        if (!chunk->bounds.overlaps(viewBounds)) continue;

        if (chunk->dirty.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
            chunk->updateRenderData();
        }
        chunk->renderer->render(camera, chunk->worldOffset, m_tileSize);
    }
}

void ChunkMapRenderer::setPalette(const std::vector<glm::vec3>& palette) {
    ChunkRenderer::setGlobalPalette(palette);
}
