// Shaders/shader_palette.frag
#version 460 core
in vec2 TexCoord;
flat in uint TileIndex;

uniform sampler1D uPalette;

out vec4 FragColor;

void main() {
    float index = float(TileIndex) / 255.0;
    vec3 color = texture(uPalette, index).rgb;
    FragColor = vec4(color, 1.0);
}