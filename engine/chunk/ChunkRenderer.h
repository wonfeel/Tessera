#pragma once
#include <memory>
#include <vector>
#include <cstdint>
#include <glm/glm.hpp>
#include <glad/glad.h>    

class Shader;
class Camera2D;

class ChunkRenderer {
public:
    ~ChunkRenderer();

    ChunkRenderer(int chunkSize);
    ChunkRenderer(const ChunkRenderer&) = delete;
    ChunkRenderer& operator=(const ChunkRenderer&) = delete;

    void updateIndices(const uint8_t* data);
    void render(const Camera2D& camera, const glm::ivec2& chunkOffset, float tileSize);

    // GL VBO для CUDA-OpenGL interop: CUDA пишет результат симуляции напрямую
    // в этот буфер, минуя CPU roundtrip через glBufferSubData.
    unsigned int getInstanceVBO() const { return instanceVBO; }

    static void setGlobalPalette(const std::vector<glm::vec3>& palette);
    // CPU-копия активной палитры (value 0..255 -> RGB в диапазоне 0..1).
    // Нужна тем, кто рисует пиксели на CPU (например, экспорт GIF из состояния
    // клеток напрямую, без чтения GL-фреймбуфера).
    static const std::vector<glm::vec3>& getGlobalPalette();
    static void shutdownStatics();

private:
    static void initStatics();
    void setupQuad();
    void setupInstanceBuffer();

    unsigned int VAO = 0, VBO = 0, EBO = 0;
    unsigned int instanceVBO = 0;

    int m_chunkSize;
    int m_instanceCount;
    static constexpr int PALETTE_SIZE = 256;

    static std::unique_ptr<Shader> s_shader;
    static unsigned int s_paletteTexture;
    static bool s_staticInitialized;
    static std::vector<glm::vec3> s_palette;   // CPU-копия палитры

    static GLint s_locCamera;
    static GLint s_locCellSize;
    static GLint s_locGridWidth;
    static GLint s_locChunkOffset;
    static GLint s_locPalette;
};