#pragma once
#include "engine/graphics/IRenderable.h"
#include "engine/tilemap/ITileMap.h"
#include "engine/tilemap/IColorTileMap.h"
#include "engine/tilemap/IndexedTileMap.h"
#include "engine/graphics/Shader.h"
#include "engine/graphics/Camera2D.h"
#include <memory>
#include <vector>
#include <glm/glm.hpp>

class GridRenderer : public IRenderable {
public:
    explicit GridRenderer(ITileMap& tileMap);
    ~GridRenderer();

    GridRenderer(const GridRenderer&) = delete;
    GridRenderer& operator=(const GridRenderer&) = delete;

    void render(const Camera2D& camera) override;
    void setPalette(const std::vector<glm::vec3>& palette);

private:
    void setupQuadGeometry();
    void setupInstanceBuffers();
    void updateInstanceBuffers();
    void createDefaultPalette();

    ITileMap& m_tileMap;
    IIndexedTileMap* m_indexedMap = nullptr;  // если карта поддерживает индексы
    IColorTileMap* m_colorMap = nullptr;       // если карта поддерживает цвета

    std::unique_ptr<Shader> m_shader;
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    unsigned int instanceVBO_Index = 0;
    unsigned int instanceVBO_Offset = 0;
    unsigned int instanceVBO_Color = 0;
    int instanceCount = 0;

    unsigned int paletteTextureID = 0;
    static constexpr int PALETTE_SIZE = 256;
    bool useIndices = false;
};