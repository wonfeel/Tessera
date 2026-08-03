// engine/utils/ScreenRecorder.h
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// Запись экрана ЛЮБОГО приложения на Application - встроена в движок, а не в
// конкретную демку, потому что нужна для всех (см. Application::renderLoop,
// точка вызова captureFrame() - сразу после отрисовки кадра, перед
// glfwSwapBuffers).
//
// ДВА БЭКЕНДА, оба пишут в один и тот же класс:
//   - FFMPEG PIPE, если рядом с исполняемым файлом лежит ffmpeg.exe (кладёт
//     туда CMake из libs/ffmpeg/bin, см. CMakeLists.txt) - настоящее видео
//     (H.264 mp4), хорошее сжатие.
//   - АНИМИРОВАННЫЙ GIF через уже подключённую в проекте libs/gif/gif.h, если
//     ffmpeg.exe не найден - работает без единой внешней зависимости, но
//     хуже по качеству и размеру на длинных роликах.
// Выбор бэкенда происходит в start() и не виден вызывающему коду.
//
// ПРОИЗВОДИТЕЛЬНОСТЬ. glReadPixels - синхронная точка: она ждёт, пока GPU
// закончит рисовать кадр. Она вызывается НЕ на каждом кадре рендера, а с
// частотой fps (см. m_lastFrameTime) - и это не только про верную длину
// ролика (см. там же), а ещё и ограничивает цену: на рендере в 170 кадров/с
// без привязки readback дёргался бы 170 раз/с, с привязкой - не чаще fps.
// Что не ждёт GPU вовсе - запись на диск/в канал ffmpeg: она уходит в
// отдельный поток с очередью кадров, чтобы медленный диск не тормозил
// рендер-поток сверх неизбежного readback.
//
// ПОТОКОВОСТЬ. captureFrame() зовётся с рендер-потока (там же, где
// действителен GL-контекст). start()/stop() тоже - см. комментарий у
// Application::toggleRecording.
class ScreenRecorder {
public:
    ~ScreenRecorder();

    // ffmpegPath - путь к ffmpeg.exe рядом с исполняемым файлом, или пустая
    // строка, если его там нет (тогда используется GIF-бэкенд).
    bool start(int width, int height, int fps, const std::string& outPathNoExt,
               const std::string& ffmpegPath);
    void stop();
    void captureFrame();  // вызывать РОВНО ОДИН РАЗ за кадр, из рендер-потока
    bool isRecording() const { return m_recording.load(); }
    const std::string& lastOutputPath() const { return m_outPath; }

private:
    enum class Backend { None, Ffmpeg, Gif };

    void writerThreadFunc();
    void writeFfmpegFrame(const std::vector<uint8_t>& rgb);
    void writeGifFrame(const std::vector<uint8_t>& rgb);
    void closeBackend();

    Backend m_backend = Backend::None;
    std::atomic<bool> m_recording{false};
    int m_width = 0, m_height = 0, m_fps = 30;
    std::string m_outPath;

    // ПРИВЯЗКА ПО ВРЕМЕНИ, а не по кадру рендера. Рендер-цикл ничем не
    // ограничен по FPS (см. Application::renderLoop - там нет ни vsync, ни
    // ограничителя), поэтому captureFrame() дёргается с ЛЮБОЙ частотой,
    // какую выдаёт машина - на этой измерено около 170 раз в секунду. Если
    // отдавать в ffmpeg/GIF КАЖДЫЙ такой вызов, заявляя при этом
    // "-framerate 30", ролик растягивается: 4 реальные секунды дали 681
    // кадр и на выходе 22.7 секунды видео - воспроизведено и измерено на
    // этой машине до того, как появилась эта привязка.
    //
    // Лечится здесь, а не throttling'ом рендер-цикла целиком (тот трогать
    // нельзя - остальным демкам нужен полный FPS): captureFrame() сравнивает
    // время с последним ПРИНЯТЫМ кадром и молча выходит, если ещё не прошло
    // 1/fps секунды - без единого вызова glReadPixels на пропущенных кадрах,
    // то есть попутно снимает и часть цены из класс-комментария выше про
    // просадку FPS во время записи.
    std::chrono::steady_clock::time_point m_lastFrameTime;

    // ffmpeg-бэкенд: сырой канал в дочерний процесс.
    FILE* m_ffmpegPipe = nullptr;

    // gif-бэкенд: состояние держится в .cpp (там же, где единственный
    // #include "libs/gif/gif.h" во всём проекте - см. GifExport.cpp,
    // повторное подключение в другом .cpp дало бы дублирующиеся символы
    // линковки, у gif.h определения функций прямо в заголовке).
    void* m_gifWriter = nullptr;  // на деле GifWriter*, см. .cpp

    // Очередь кадров и поток-писатель - см. класс-комментарий выше про
    // производительность.
    std::thread m_writerThread;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCv;
    std::queue<std::vector<uint8_t>> m_frameQueue;
    std::atomic<bool> m_writerRunning{false};
    // Кадры не читанные писателем к моменту stop() - печатается пользователю,
    // чтобы явно видеть, если запись отстаёт от рендера, а не молчать об этом.
    std::atomic<int> m_droppedFrames{0};
    static constexpr size_t kMaxQueueDepth = 8;  // видеопамять кадра растёт с разрешением - не копим безгранично
};
