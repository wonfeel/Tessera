#pragma once

// Offscreen FBO с одной цветовой текстурой + mipmaps - для рендера сцены на
// фиксированном разрешении и последующего честного GPU-даунсэмпла через
// GL_LINEAR_MIPMAP_LINEAR (вместо аппаратного пола точечного размера).
class RenderTarget {
public:
    RenderTarget(int width, int height);
    ~RenderTarget();
    RenderTarget(const RenderTarget&) = delete;
    RenderTarget& operator=(const RenderTarget&) = delete;

    // Биндит FBO и выставляет viewport под его разрешение - рендерить нужно
    // сразу после этого вызова.
    void bind() const;
    void generateMipmaps() const;

    unsigned int textureID() const { return m_colorTex; }
    int width() const { return m_width; }
    int height() const { return m_height; }

    // Возвращает рендер на экран (framebuffer 0) и восстанавливает viewport.
    static void unbind(int screenWidth, int screenHeight);

private:
    unsigned int m_fbo = 0, m_colorTex = 0;
    int m_width, m_height;
};
