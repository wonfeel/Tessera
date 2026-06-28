// Shaders/shader.vert 
#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec2 aOffset;
layout (location = 3) in vec3 aColor;

uniform mat4 uCamera;
uniform float uCellSize;

out vec2 TexCoord;
out vec3 vColor;

void main() {
    vec2 worldPos = aOffset * uCellSize;
    vec2 vertexPos = worldPos + aPos * uCellSize;
    gl_Position = uCamera * vec4(vertexPos, 0.0, 1.0);
    TexCoord = aTexCoord;
    vColor = aColor;
}