// engine/utils/GifExport.h
#pragma once
#include <cstdint>
#include <string>
#include <functional>

class ChunkedTileMap;

// Параметры экспорта анимированного GIF напрямую из состояния клеток.
// Пиксели вычисляются на CPU: каждая клетка региона рисуется блоком
// scale×scale одного цвета (из активной палитры). Никакого чтения
// GL-фреймбуфера, никакой интерполяции — один-в-один, без размытия.
struct GifExportParams {
    int x0 = 0, y0 = 0;   // верхний левый угол региона (координаты сетки, включительно)
    int x1 = 0, y1 = 0;   // нижний правый угол (исключительно): рисуется [x0,x1) × [y0,y1)
    int scale   = 4;      // пикселей на клетку (>=1)
    int frames  = 60;     // сколько кадров записать
    int stride  = 2;      // шагов симуляции между кадрами
    int delayMs = 50;     // задержка между кадрами в миллисекундах
    std::string path;     // путь к выходному .gif
};

// Записывает GIF. Между кадрами вызывает stepFn() ровно `stride` раз — это
// должно продвинуть симуляцию на одно поколение (например,
// map.simulateAndWait(); map.commitReadyChunks(cam);). Клетки читаются через
// map.getTileState(). Возвращает false при неверных параметрах или ошибке файла.
bool ExportGif(ChunkedTileMap& map, const GifExportParams& params,
               const std::function<void()>& stepFn);

// ---------------------------------------------------------------------------
// ПОТОКОВЫЙ GIF-ЗАПИСЧИК ПРОИЗВОЛЬНЫХ RGBA-КАДРОВ (для ScreenRecorder).
//
// Зачем это здесь, а не в ScreenRecorder.cpp: libs/gif/gif.h объявляет
// GifBegin/GifWriteFrame/GifEnd с обычным (не static/не inline) внешним
// связыванием — второй #include этого заголовка в другом .cpp даст
// дублирующиеся символы при линковке EngineLib. Поэтому во всём проекте его
// подключает РОВНО ОДИН файл — этот, и любой другой код, которому нужен GIF,
// проходит через эту тонкую обёртку, а не через сам libs/gif/gif.h напрямую.
//
// GifStream — непрозрачный указатель, реальный тип (GifWriter*) скрыт в .cpp.
struct GifStream;
GifStream* GifStreamBegin(const std::string& path, uint32_t width, uint32_t height,
                          uint32_t delayCs);
// rgba — width*height*4 байт, ряды сверху вниз (как из glReadPixels после
// программного переворота — сам переворот делает вызывающий, здесь его нет).
void GifStreamWriteFrame(GifStream* stream, const uint8_t* rgba, uint32_t width,
                         uint32_t height, uint32_t delayCs);
void GifStreamEnd(GifStream* stream);
