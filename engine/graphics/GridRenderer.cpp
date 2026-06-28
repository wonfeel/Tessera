#include "GridRenderer.h"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

static const float quadVertices[] = {
     0.0f, 0.0f,   0.0f, 0.0f,
     1.0f, 0.0f,   1.0f, 0.0f,
     1.0f, 1.0f,   1.0f, 1.0f,
     0.0f, 1.0f,   0.0f, 1.0f
};
static const unsigned int quadIndices[] = { 0, 1, 2, 2, 3, 0 };

GridRenderer::GridRenderer(ITileMap& tileMap)
    : m_tileMap(tileMap)
{
    m_indexedMap = dynamic_cast<IIndexedTileMap*>(&tileMap);
    m_colorMap = dynamic_cast<IColorTileMap*>(&tileMap);
    useIndices = (m_indexedMap != nullptr);

    if (useIndices) {
        m_shader = std::make_unique<Shader>("Shaders/shader_indexed.vert", "Shaders/shader_palette.frag");
        createDefaultPalette();
    }
    else {
        m_shader = std::make_unique<Shader>("Shaders/shader.vert", "Shaders/shader.frag");
    }

    setupQuadGeometry();
    setupInstanceBuffers();
}

GridRenderer::~GridRenderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    if (instanceVBO_Index) glDeleteBuffers(1, &instanceVBO_Index);
    if (instanceVBO_Offset) glDeleteBuffers(1, &instanceVBO_Offset);
    if (instanceVBO_Color) glDeleteBuffers(1, &instanceVBO_Color);
    if (paletteTextureID) glDeleteTextures(1, &paletteTextureID);
}

void GridRenderer::setupQuadGeometry() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIndices), quadIndices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void GridRenderer::createDefaultPalette() {
    std::vector<glm::vec3> pal(PALETTE_SIZE);
    for (int i = 0; i < PALETTE_SIZE; ++i) {
        float v = i / float(PALETTE_SIZE - 1);
        pal[i] = glm::vec3(v);
    }

    glGenTextures(1, &paletteTextureID);
    glBindTexture(GL_TEXTURE_1D, paletteTextureID);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, PALETTE_SIZE, 0, GL_RGB, GL_FLOAT, pal.data());
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
}

void GridRenderer::setPalette(const std::vector<glm::vec3>& palette) {
    if (!useIndices || palette.size() != PALETTE_SIZE) return;
    glBindTexture(GL_TEXTURE_1D, paletteTextureID);
    glTexSubImage1D(GL_TEXTURE_1D, 0, 0, PALETTE_SIZE, GL_RGB, GL_FLOAT, palette.data());
}

void GridRenderer::setupInstanceBuffers() {
    int w = m_tileMap.getWidth();
    int h = m_tileMap.getHeight();
    instanceCount = w * h;

    if (useIndices && m_indexedMap) {
        glGenBuffers(1, &instanceVBO_Index);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Index);
        glBufferData(GL_ARRAY_BUFFER, instanceCount * sizeof(uint8_t), nullptr, GL_DYNAMIC_DRAW);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Index);
        glVertexAttribIPointer(2, 1, GL_UNSIGNED_BYTE, sizeof(uint8_t), (void*)0);
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);
    }
    else if (m_colorMap) {
        std::vector<glm::vec2> offsets(instanceCount);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                offsets[y * w + x] = glm::vec2(static_cast<float>(x), static_cast<float>(y));

        glGenBuffers(1, &instanceVBO_Offset);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Offset);
        glBufferData(GL_ARRAY_BUFFER, offsets.size() * sizeof(glm::vec2), offsets.data(), GL_STATIC_DRAW);

        glGenBuffers(1, &instanceVBO_Color);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Color);
        glBufferData(GL_ARRAY_BUFFER, instanceCount * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Offset);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);

        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Color);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glEnableVertexAttribArray(3);
        glVertexAttribDivisor(3, 1);
    }
    glBindVertexArray(0);
}

void GridRenderer::updateInstanceBuffers() {
    if (useIndices && m_indexedMap) {
        if (!m_indexedMap->isDirty()) return;
        const auto& indices = m_indexedMap->getTileIndices();
        if (!indices.empty()) {
            glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Index);
            glBufferSubData(GL_ARRAY_BUFFER, 0, indices.size() * sizeof(uint8_t), indices.data());
        }
        m_indexedMap->clearDirty();
    }
    else if (m_colorMap) {
        if (!m_colorMap->isDirty()) return;
        const auto& colors = m_colorMap->getTileColors();
        if (!colors.empty()) {
            glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_Color);
            glBufferSubData(GL_ARRAY_BUFFER, 0, colors.size() * sizeof(glm::vec3), colors.data());
        }
        m_colorMap->clearDirty();
    }
}

void GridRenderer::render(const Camera2D& camera) {
    updateInstanceBuffers();
    m_shader->use();
    m_shader->setMat4("uCamera", camera.getViewProjectionMatrix());
    m_shader->setFloat("uCellSize", m_tileMap.getTileSize());

    if (useIndices) {
        m_shader->setInt("uGridWidth", m_tileMap.getWidth());
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_1D, paletteTextureID);
        m_shader->setInt("uPalette", 0);
    }

    glBindVertexArray(VAO);
    glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, instanceCount);
    glBindVertexArray(0);
}