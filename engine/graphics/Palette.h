// engine/graphics/Palette.h
#pragma once
#include <glm/glm.hpp>
#include <vector>

namespace Palette {

    // ---------- Предопределённые стандартные палитры (256 цветов) ----------

    // Стандартная радужная палитра (используется по умолчанию)
    inline std::vector<glm::vec3> DefaultRainbow() {
        std::vector<glm::vec3> pal(256);
        pal[0] = glm::vec3(0.0f);  // состояние 0 (пусто) — чёрный
        for (int i = 1; i < 256; ++i) {
            float h = (i - 1) / 255.0f;
            float r, g, b;
            if (h < 0.1667f) {
                r = 1.0f; g = h * 6.0f; b = 0.0f;
            }
            else if (h < 0.3333f) {
                r = 1.0f - (h - 0.1667f) * 6.0f; g = 1.0f; b = 0.0f;
            }
            else if (h < 0.5f) {
                r = 0.0f; g = 1.0f; b = (h - 0.3333f) * 6.0f;
            }
            else if (h < 0.6667f) {
                r = 0.0f; g = 1.0f - (h - 0.5f) * 6.0f; b = 1.0f;
            }
            else if (h < 0.8333f) {
                r = (h - 0.6667f) * 6.0f; g = 0.0f; b = 1.0f;
            }
            else {
                r = 1.0f; g = 0.0f; b = 1.0f - (h - 0.8333f) * 6.0f;
            }
            pal[i] = glm::vec3(r, g, b);
        }
        return pal;
    }

    // Чёрно-белый градиент 
    inline std::vector<glm::vec3> Grayscale() {
        std::vector<glm::vec3> pal(256);
        for (int i = 0; i < 256; ++i) {
            float v = i / 255.0f;
            pal[i] = glm::vec3(v, v, v);
        }
        return pal;
    }

    // Классическая палитра на базе 16 ANSI-цветов с плавным продолжением
    inline std::vector<glm::vec3> ANSI() {
        std::vector<glm::vec3> pal(256);
        // Первые 16 — стандартные ANSI
        pal[0] = glm::vec3(0.0f, 0.0f, 0.0f);       // Black
        pal[1] = glm::vec3(0.5f, 0.0f, 0.0f);       // Red
        pal[2] = glm::vec3(0.0f, 0.5f, 0.0f);       // Green
        pal[3] = glm::vec3(0.5f, 0.5f, 0.0f);       // Yellow
        pal[4] = glm::vec3(0.0f, 0.0f, 0.5f);       // Blue
        pal[5] = glm::vec3(0.5f, 0.0f, 0.5f);       // Magenta
        pal[6] = glm::vec3(0.0f, 0.5f, 0.5f);       // Cyan
        pal[7] = glm::vec3(0.75f, 0.75f, 0.75f);    // White
        pal[8] = glm::vec3(0.5f, 0.5f, 0.5f);       // Bright Black
        pal[9] = glm::vec3(1.0f, 0.0f, 0.0f);       // Bright Red
        pal[10] = glm::vec3(0.0f, 1.0f, 0.0f);       // Bright Green
        pal[11] = glm::vec3(1.0f, 1.0f, 0.0f);       // Bright Yellow
        pal[12] = glm::vec3(0.0f, 0.0f, 1.0f);       // Bright Blue
        pal[13] = glm::vec3(1.0f, 0.0f, 1.0f);       // Bright Magenta
        pal[14] = glm::vec3(0.0f, 1.0f, 1.0f);       // Bright Cyan
        pal[15] = glm::vec3(1.0f, 1.0f, 1.0f);       // Bright White
        // Остальные 240 цветов — циклическое повторение с изменением яркости
        for (int i = 16; i < 256; ++i) {
            int base = i % 16;
            float scale = 0.4f + 0.6f * ((i / 16) / 15.0f); // яркость от 0.4 до 1.0
            pal[i] = pal[base] * scale;
        }
        return pal;
    }

    // Монохромная зелёная (как в старых терминалах)
    inline std::vector<glm::vec3> MonochromeGreen() {
        std::vector<glm::vec3> pal(256);
        pal[0] = glm::vec3(0.0f);
        for (int i = 1; i < 256; ++i) {
            float v = i / 255.0f;
            pal[i] = glm::vec3(0.0f, v, 0.0f);
        }
        return pal;
    }

    // Тепловая карта (чёрный -> красный -> жёлтый -> белый)
    inline std::vector<glm::vec3> Heatmap() {
        std::vector<glm::vec3> pal(256);
        pal[0] = glm::vec3(0.0f);
        for (int i = 1; i < 256; ++i) {
            float t = i / 255.0f;
            if (t < 0.33f) {
                pal[i] = glm::vec3(t * 3.0f, 0.0f, 0.0f);
            }
            else if (t < 0.66f) {
                pal[i] = glm::vec3(1.0f, (t - 0.33f) * 3.0f, 0.0f);
            }
            else {
                pal[i] = glm::vec3(1.0f, 1.0f, (t - 0.66f) * 3.0f);
            }
        }
        return pal;
    }

    // Холодная палитра (чёрный -> синий -> циан -> белый)
    inline std::vector<glm::vec3> Cool() {
        std::vector<glm::vec3> pal(256);
        pal[0] = glm::vec3(0.0f);
        for (int i = 1; i < 256; ++i) {
            float t = i / 255.0f;
            if (t < 0.33f) {
                pal[i] = glm::vec3(0.0f, 0.0f, t * 3.0f);
            }
            else if (t < 0.66f) {
                pal[i] = glm::vec3(0.0f, (t - 0.33f) * 3.0f, 1.0f);
            }
            else {
                pal[i] = glm::vec3((t - 0.66f) * 3.0f, 1.0f, 1.0f);
            }
        }
        return pal;
    }
}