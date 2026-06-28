// Shaders/shader.frag 
#version 460 core
in vec2 TexCoord;
in vec3 vColor;

out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}