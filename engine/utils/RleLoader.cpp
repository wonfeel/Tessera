// engine/utils/RleLoader.cpp
#include "engine/utils/RleLoader.h"
#include <fstream>
#include <sstream>
#include <cctype>
#include <algorithm>
#include <filesystem>

std::vector<std::string> RleLoader::listFiles(const std::string& dir) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return out;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        const auto& p = entry.path();
        if (p.extension() == ".rle")
            out.push_back(p.generic_string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

RlePattern RleLoader::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return {};
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    return parse(src);
}

RlePattern RleLoader::parse(const std::string& src) {
    RlePattern out;
    std::istringstream ss(src);
    std::string line;

    // --- Шаг 1: пропустить комментарии, найти строку заголовка ---
    std::string headerLine;
    while (std::getline(ss, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;   // #N, #O, #C и т.д.
        headerLine = line;
        break;
    }
    if (headerLine.empty()) return out;

    // --- Шаг 2: распарсить заголовок "x = W, y = H[, rule = ...]" ---
    {
        std::istringstream h(headerLine);
        char c;
        std::string tok;
        // x = W
        while (h >> tok && tok != "x") {}
        h >> c;   // '='
        h >> out.width;
        // , y = H
        while (h >> tok && tok != "y") {}
        h >> c;
        h >> out.height;
        // optional: , rule = ...
        while (h >> tok) {
            if (tok == "rule" || tok == "rule=") {
                // следующий токен или часть этой строки после '='
                std::string ruleVal;
                if (h >> c && c == '=') h >> ruleVal;
                else ruleVal = c + std::string("");
                // иногда rule уже слитый: tok="rule=B3/S23"
                auto eq = tok.find('=');
                if (eq != std::string::npos)
                    out.ruleName = tok.substr(eq + 1);
                else
                    out.ruleName = ruleVal;
                break;
            }
            // tok может быть "rule=B3/S23" целиком (без пробелов)
            auto eq = tok.find('=');
            if (eq != std::string::npos) {
                std::string key = tok.substr(0, eq);
                if (key == "rule") { out.ruleName = tok.substr(eq + 1); break; }
            }
        }
    }

    if (out.width <= 0 || out.height <= 0) return out;

    // --- Шаг 3: собрать тело паттерна (может быть разбито на несколько строк) ---
    std::string body;
    while (std::getline(ss, line)) {
        body += line;
        if (body.find('!') != std::string::npos) break;
    }

    // --- Шаг 4: декодировать RLE в клетки ---
    out.cells.assign(static_cast<size_t>(out.width) * out.height, 0);

    int cx = 0, cy = 0;
    int runLen = 0;

    for (char ch : body) {
        if (ch == '!') break;

        // Пробельные символы пропускаем НЕ трогая накопленный счётчик: по спеку
        // RLE пробелы/переводы строк допустимы где угодно, в т.ч. между числом и
        // тегом ("10 o"). Раньше пробел сбрасывал runLen и счётчик терялся.
        if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t')
            continue;

        if (std::isdigit(static_cast<unsigned char>(ch))) {
            runLen = runLen * 10 + (ch - '0');
            continue;
        }

        int count = (runLen == 0) ? 1 : runLen;
        runLen = 0;

        if (ch == 'b') {
            // мёртвые клетки — просто сдвинуть курсор
            cx += count;
        } else if (ch == 'o') {
            for (int i = 0; i < count; ++i) {
                if (cx < out.width && cy < out.height)
                    out.cells[cy * out.width + cx] = 1;
                ++cx;
            }
        } else if (ch == '$') {
            cy += count;
            cx = 0;
        }
        // любые другие символы (многосостояние, 'A'..'X') — пропускаем
    }

    return out;
}
