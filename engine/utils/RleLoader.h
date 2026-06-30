// engine/utils/RleLoader.h
#pragma once
#include <string>
#include <vector>
#include <cstdint>

struct RlePattern {
    int width  = 0;
    int height = 0;
    std::string ruleName;                 // e.g. "B3/S23", may be empty
    std::vector<uint8_t> cells;           // row-major, 0=dead, 1=alive, size=width*height
};

// Загружает и парсит паттерны в формате RLE (стандарт LifeWiki).
// Поддерживает: комментарии #N/#O/#C, заголовок x/y/rule, run-length коды.
class RleLoader {
public:
    // Загрузить из файла. Возвращает пустой паттерн (width==0) при ошибке.
    static RlePattern load(const std::string& path);

    // Распарсить из строки (весь файл целиком). Удобно для тестов.
    static RlePattern parse(const std::string& src);

    // Список путей к *.rle в каталоге (отсортирован по имени). Пусто, если каталога
    // нет. Удобно для UI выбора паттерна.
    static std::vector<std::string> listFiles(const std::string& dir);
};
