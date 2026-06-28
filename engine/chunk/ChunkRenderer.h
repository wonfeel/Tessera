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

    static void setGlobalPalette(const std::vector<glm::vec3>& palette);
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

    static GLint s_locCamera;
    static GLint s_locCellSize;
    static GLint s_locGridWidth;
    static GLint s_locChunkOffset;
    static GLint s_locPalette;
};