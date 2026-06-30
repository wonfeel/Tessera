#include "ChunkRenderer.h"
#include "engine/graphics/Shader.h"
#include "engine/graphics/Camera2D.h"
#include <glm/gtc/type_ptr.hpp>
#include <cstring>

std::unique_ptr<Shader> ChunkRenderer::s_shader = nullptr;
unsigned int ChunkRenderer::s_paletteTexture = 0;
bool ChunkRenderer::s_staticInitialized = false;
std::vector<glm::vec3> ChunkRenderer::s_palette;

GLint ChunkRenderer::s_locCamera = -1;
GLint ChunkRenderer::s_locCellSize = -1;
GLint ChunkRenderer::s_locGridWidth = -1;
GLint ChunkRenderer::s_locChunkOffset = -1;
GLint ChunkRenderer::s_locPalette = -1;

static const float quadVertices[] = {
    0.0f, 0.0f,   0.0f, 0.0f,
    1.0f, 0.0f,   1.0f, 0.0f,
    1.0f, 1.0f,   1.0f, 1.0f,
    0.0f, 1.0f,   0.0f, 1.0f
};
static const unsigned int quadIndices[] = { 0, 1, 2, 2, 3, 0 };

ChunkRenderer::ChunkRenderer(int chunkSize) : m_chunkSize(chunkSize),
m_instanceCount(chunkSize* chunkSize) {
    initStatics();
    setupQuad();
    setupInstanceBuffer();
}

ChunkRenderer::~ChunkRenderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    if (instanceVBO) glDeleteBuffers(1, &instanceVBO);
}

void ChunkRenderer::initStatics() {
    if (s_staticInitialized) return;

    s_shader = std::make_unique<Shader>("Shaders/shader_chunk.vert", "Shaders/shader_palette.frag");

    s_locCamera = glGetUniformLocation(s_shader->getID(), "uCamera");
    s_locCellSize = glGetUniformLocation(s_shader->getID(), "uCellSize");
    s_locGridWidth = glGetUniformLocation(s_shader->getID(), "uGridWidth");
    s_locChunkOffset = glGetUniformLocation(s_shader->getID(), "uChunkOffset");
    s_locPalette = glGetUniformLocation(s_shader->getID(), "uPalette");

    std::vector<glm::vec3> pal(PALETTE_SIZE);
    for (int i = 0; i < PALETTE_SIZE; ++i) {
        float v = i / float(PALETTE_SIZE - 1);
        if (i < 64) {
            pal[i] = glm::vec3(0.0f, 0.0f, v);
        }
        else if (i < 128) {
            pal[i] = glm::vec3(0.0f, v, 0.0f);
        }
        else if (i < 192) {
            pal[i] = glm::vec3(v, 0.0f, 0.0f);
        }
        else {
            pal[i] = glm::vec3(v, v, v);
        }
    }

    s_palette = pal;   // запоминаем CPU-копию (нужна для экспорта GIF)

    glGenTextures(1, &s_paletteTexture);
    glBindTexture(GL_TEXTURE_1D, s_paletteTexture);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, PALETTE_SIZE, 0, GL_RGB, GL_FLOAT, pal.data());
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);

    s_staticInitialized = true;
}

void ChunkRenderer::setGlobalPalette(const std::vector<glm::vec3>& palette) {
    initStatics();
    if (palette.size() != PALETTE_SIZE) return;

    s_palette = palette;   // обновляем CPU-копию вместе с GL-текстурой

    glBindTexture(GL_TEXTURE_1D, s_paletteTexture);
    glTexSubImage1D(GL_TEXTURE_1D, 0, 0, PALETTE_SIZE, GL_RGB, GL_FLOAT, palette.data());
}

const std::vector<glm::vec3>& ChunkRenderer::getGlobalPalette() {
    return s_palette;
}

void ChunkRenderer::setupQuad() {
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
}

void ChunkRenderer::setupInstanceBuffer() {
    glGenBuffers(1, &instanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, m_instanceCount * sizeof(uint8_t), nullptr, GL_DYNAMIC_DRAW);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glVertexAttribIPointer(2, 1, GL_UNSIGNED_BYTE, sizeof(uint8_t), (void*)0);
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glBindVertexArray(0);
}

void ChunkRenderer::updateIndices(const uint8_t* data) {
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_instanceCount * sizeof(uint8_t), data);
}

void ChunkRenderer::render(const Camera2D& camera, const glm::ivec2& chunkOffset, float tileSize) {
    if (!s_shader) return;

    s_shader->use();

    if (s_locCamera != -1)
        glUniformMatrix4fv(s_locCamera, 1, GL_FALSE, glm::value_ptr(camera.getViewProjectionMatrix()));
    if (s_locCellSize != -1)
        glUniform1f(s_locCellSize, tileSize);
    if (s_locGridWidth != -1)
        glUniform1i(s_locGridWidth, m_chunkSize);
    if (s_locChunkOffset != -1)
        glUniform2i(s_locChunkOffset, chunkOffset.x, chunkOffset.y);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_1D, s_paletteTexture);
    if (s_locPalette != -1)
        glUniform1i(s_locPalette, 0);

    glBindVertexArray(VAO);
    glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, m_instanceCount);
    glBindVertexArray(0);
}

void ChunkRenderer::shutdownStatics() {
    s_shader.reset();
    if (s_paletteTexture) {
        glDeleteTextures(1, &s_paletteTexture);
        s_paletteTexture = 0;
    }
    s_staticInitialized = false;
}