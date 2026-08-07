// engine/utils/ScreenRecorder.cpp
#include "engine/utils/ScreenRecorder.h"
#include "engine/utils/GifExport.h"

#include <glad/glad.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
// GL_RGBA + GL_UNSIGNED_BYTE = 4 байта на пиксель - формат общий для обоих
// бэкендов, чтобы не гонять два разных пути конвертации.
constexpr int kBytesPerPixel = 4;

// glReadPixels отдаёт строки СНИЗУ ВВЕРХ (начало координат OpenGL - левый
// нижний угол). И видео, и GIF ожидают привычный порядок сверху вниз.
// Переворот - здесь и только здесь, один раз, чтобы оба бэкенда молча
// получали уже правильный кадр и не решали этот вопрос порознь.
void flipRowsInPlace(std::vector<uint8_t>& buf, int width, int height) {
    const size_t rowBytes = static_cast<size_t>(width) * kBytesPerPixel;
    std::vector<uint8_t> tmp(rowBytes);
    for (int y = 0; y < height / 2; ++y) {
        uint8_t* top = buf.data() + static_cast<size_t>(y) * rowBytes;
        uint8_t* bot = buf.data() + static_cast<size_t>(height - 1 - y) * rowBytes;
        std::memcpy(tmp.data(), top, rowBytes);
        std::memcpy(top, bot, rowBytes);
        std::memcpy(bot, tmp.data(), rowBytes);
    }
}

bool fileExists(const std::string& path) {
    if (path.empty()) return false;
    FILE* f = nullptr;
    fopen_s(&f, path.c_str(), "rb");  // fopen_s, не fopen - без него MSVC шумит про безопасность
    if (!f) return false;
    std::fclose(f);
    return true;
}
}  // namespace

ScreenRecorder::~ScreenRecorder() {
    if (m_recording.load()) stop();
}

bool ScreenRecorder::start(int width, int height, int fps,
                           const std::string& outPathNoExt,
                           const std::string& ffmpegPath) {
    if (m_recording.load()) return false;  // повторный старт - вызывающий код ошибся
    if (width <= 0 || height <= 0 || fps <= 0) return false;

    // Сдвинута в прошлое - первый вызов captureFrame() после старта обязан
    // захватить кадр немедленно, а не ждать один интервал впустую.
    m_lastFrameTime = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    m_width = width;
    m_height = height;
    m_fps = fps;
    m_droppedFrames = 0;

    // TESSERA_FFMPEG_ENABLED=OFF (CMake option, см. CMakeLists.txt) - не
    // трогаем ffmpeg вовсе, даже если ffmpeg.exe случайно оказался рядом
    // (например, остался от предыдущей сборки с другими опциями) - выбор
    // сборки должен быть детерминированным, не зависеть от мусора на диске.
#ifdef TESSERA_FFMPEG_ENABLED
    const bool haveFfmpeg = fileExists(ffmpegPath);
#else
    const bool haveFfmpeg = false;
#endif
    m_backend = haveFfmpeg ? Backend::Ffmpeg : Backend::Gif;
    m_outPath = outPathNoExt + (haveFfmpeg ? ".mp4" : ".gif");

    if (haveFfmpeg) {
        // Сырой RGBA-поток на stdin ffmpeg, кодирование в H.264/mp4. Строка
        // команды формируется здесь; редирект stderr в лог рядом с
        // выходным файлом - чтобы ошибку ffmpeg было где искать, а не
        // терять её в никуда.
        //
        // ffmpegPath СОЗНАТЕЛЬНО НЕ В КАВЫЧКАХ. _popen на Windows запускает
        // команду через "cmd.exe /c <строка>", а у cmd.exe есть отдельная
        // ловушка разбора: если /c-аргумент начинается с кавычки, он иногда
        // срывает С НЕЁ ПЕРВЫЙ И ПОСЛЕДНИЙ символ строки целиком, а не только
        // снимает кавычки вокруг первого токена - и тогда вся команда
        // разваливается ещё ДО того, как что-либо запустится (даже
        // редиректный .ffmpeg.log не создаётся - значит cmd.exe не смог
        // разобрать команду, а не что ffmpeg упал). Воспроизведено и
        // проверено на этой машине: "ffmpeg.exe" -version с ведущей кавычкой
        // - exit code 1, лога нет; то же самое без кавычки - exit code 0.
        // ffmpegPath по соглашению этого файла - всегда простое имя без
        // пробелов ("ffmpeg.exe" рядом с исполняемым файлом, см.
        // Application::kFfmpegRelPath), поэтому кавычки ему и не нужны.
        // Выходной путь МОЖЕТ содержать что угодно и остаётся в кавычках -
        // он не первый токен строки, этой ловушки не касается.
        char cmd[1024];
        std::snprintf(cmd, sizeof(cmd),
            "%s -y -f rawvideo -pixel_format rgba -video_size %dx%d "
            "-framerate %d -i - -c:v libx264 -pix_fmt yuv420p -crf 20 "
            "-movflags +faststart \"%s\" 2> \"%s.ffmpeg.log\"",
            ffmpegPath.c_str(), width, height, fps, m_outPath.c_str(),
            m_outPath.c_str());
        m_ffmpegPipe = _popen(cmd, "wb");
        if (!m_ffmpegPipe) return false;
    } else {
        // GIF-дельта в сотых долях секунды - см. .h. Точность до кадра не
        // критична для превью-ролика в README, поэтому просто округляем.
        const uint32_t delayCs = static_cast<uint32_t>(
            std::max(1.0, std::round(100.0 / static_cast<double>(fps))));
        auto* stream = GifStreamBegin(m_outPath, static_cast<uint32_t>(width),
                                      static_cast<uint32_t>(height), delayCs);
        if (!stream) return false;
        m_gifWriter = stream;
    }

    m_writerRunning = true;
    m_writerThread = std::thread(&ScreenRecorder::writerThreadFunc, this);
    m_recording = true;
    return true;
}

void ScreenRecorder::captureFrame() {
    if (!m_recording.load()) return;

    // Привязка по времени - см. класс-комментарий у m_lastFrameTime в .h за
    // тем, почему это обязательно, а не просто "для точности". Выход БЕЗ
    // единого вызова glReadPixels - это и держит темп ролика равным
    // реальному времени, и не даёт записи тормозить рендер сильнее, чем
    // нужно её собственному целевому FPS.
    const auto now = std::chrono::steady_clock::now();
    const auto interval = std::chrono::duration<double>(1.0 / static_cast<double>(m_fps));
    if (now - m_lastFrameTime < interval) return;
    m_lastFrameTime = now;

    std::vector<uint8_t> buf(static_cast<size_t>(m_width) * m_height * kBytesPerPixel);
    // GL_BACK явно: рендер ещё не ушёл на экран (swap ниже по потоку в
    // Application::renderLoop), это ровно то, что только что нарисовалось.
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, m_width, m_height, GL_RGBA, GL_UNSIGNED_BYTE, buf.data());
    flipRowsInPlace(buf, m_width, m_height);

    std::lock_guard<std::mutex> lock(m_queueMutex);
    if (m_frameQueue.size() >= kMaxQueueDepth) {
        // Писатель отстаёт (медленный диск/кодирование) - роняем кадр, а НЕ
        // блокируем рендер-поток. Блокировка здесь свела бы на нет весь смысл
        // отдельного потока-писателя: рендер снова ждал бы диска, как при
        // синхронной записи. Потеря кадра дешевле потери частоты кадров.
        ++m_droppedFrames;
        return;
    }
    m_frameQueue.push(std::move(buf));
    m_queueCv.notify_one();
}

void ScreenRecorder::writerThreadFunc() {
    for (;;) {
        std::vector<uint8_t> frame;
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCv.wait(lock, [this] {
                return !m_frameQueue.empty() || !m_writerRunning.load();
            });
            if (m_frameQueue.empty() && !m_writerRunning.load()) break;
            if (m_frameQueue.empty()) continue;
            frame = std::move(m_frameQueue.front());
            m_frameQueue.pop();
        }

        if (m_backend == Backend::Ffmpeg) {
            writeFfmpegFrame(frame);
        } else if (m_backend == Backend::Gif) {
            writeGifFrame(frame);
        }
    }
}

void ScreenRecorder::writeFfmpegFrame(const std::vector<uint8_t>& rgba) {
    if (!m_ffmpegPipe) return;
    const size_t written = std::fwrite(rgba.data(), 1, rgba.size(), m_ffmpegPipe);
    if (written != rgba.size()) {
        // Канал закрылся раньше времени (ffmpeg упал) - дальнейшая запись
        // бессмысленна и небезопасна (fwrite в мёртвый pipe). Замечаем это
        // явно вместо тихого продолжения записи в никуда.
        std::fprintf(stderr, "ScreenRecorder: канал ffmpeg закрылся раньше времени\n");
        m_ffmpegPipe = nullptr;
    }
}

void ScreenRecorder::writeGifFrame(const std::vector<uint8_t>& rgba) {
    if (!m_gifWriter) return;
    const uint32_t delayCs = static_cast<uint32_t>(
        std::max(1.0, std::round(100.0 / static_cast<double>(m_fps))));
    GifStreamWriteFrame(static_cast<GifStream*>(m_gifWriter), rgba.data(),
                        static_cast<uint32_t>(m_width),
                        static_cast<uint32_t>(m_height), delayCs);
}

void ScreenRecorder::closeBackend() {
    if (m_backend == Backend::Ffmpeg && m_ffmpegPipe) {
        _pclose(m_ffmpegPipe);
        m_ffmpegPipe = nullptr;
    } else if (m_backend == Backend::Gif && m_gifWriter) {
        GifStreamEnd(static_cast<GifStream*>(m_gifWriter));
        m_gifWriter = nullptr;
    }
    m_backend = Backend::None;
}

void ScreenRecorder::stop() {
    if (!m_recording.load()) return;

    // Сигналим писателю закончить и ДОПИСАТЬ уже накопленную очередь - это
    // не отмена, а корректное завершение: последние кадры, ещё лежащие в
    // очереди, должны попасть в файл, иначе ролик обрывается раньше, чем
    // фактически была нажата кнопка "стоп".
    m_writerRunning = false;
    m_queueCv.notify_all();
    if (m_writerThread.joinable()) m_writerThread.join();

    closeBackend();
    m_recording = false;

    if (m_droppedFrames.load() > 0) {
        std::fprintf(stderr,
            "ScreenRecorder: записано с потерей %d кадров (писатель не успевал) - "
            "проверь свободное место/скорость диска, если ролик дёрганый\n",
            m_droppedFrames.load());
    }
}
