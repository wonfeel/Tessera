// Shaders/worm_body.vert
#version 460 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in float aGlow;   // |кривизна| в этой точке тела, сглажено в [0,1)
layout (location = 2) in float aAlong;  // 0 (голова) .. 1 (хвост)

uniform mat4 uCamera;

out float vGlow;
out float vAlong;

void main() {
    gl_Position = uCamera * vec4(aPos, 0.0, 1.0);
    vGlow = aGlow;
    vAlong = aAlong;
}
