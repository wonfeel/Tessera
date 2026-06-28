#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoord;

// per-instance: индекс тайла
layout (location = 2) in uint aTileIndex;

uniform mat4 uCamera;
uniform float uCellSize;
uniform int uGridWidth;          // ширина чанка в клетках (= 64)
uniform ivec2 uChunkOffset;      // смещение чанка в клетках

out vec2 TexCoord;
flat out uint TileIndex;

void main() {
    int tileX = gl_InstanceID % uGridWidth;
    int tileY = gl_InstanceID / uGridWidth;
    vec2 worldPos = (vec2(tileX, tileY) + vec2(uChunkOffset)) * uCellSize;

    vec2 vertexPos = worldPos + aPos * uCellSize;
    gl_Position = uCamera * vec4(vertexPos, 0.0, 1.0);
    TexCoord = aTexCoord;
    TileIndex = aTileIndex;
}