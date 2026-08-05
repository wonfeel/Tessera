cmake_minimum_required(VERSION 3.18)
project(Tessera VERSION 1.0 LANGUAGES CXX C)
enable_testing()

# ===== Настройки C++ =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===== Флаги компиляции =====
# Важно: add_compile_options влияет только на цели, созданные ПОСЛЕ этого вызова,
# поэтому блок должен стоять до add_library/add_executable.
# Флаги навешиваем только на C/C++ ($<COMPILE_LANGUAGE:..>), чтобы они не попали
# в командную строку nvcc при компиляции .cu — он этих ключей не понимает.
if(MSVC)
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:/W4>
                        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4100>
                        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4189>)
else()
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-Wall>
                        $<$<COMPILE_LANGUAGE:C,CXX>:-Wextra>
                        $<$<COMPILE_LANGUAGE:C,CXX>:-Wpedantic>)
endif()

# ===== AddressSanitizer (диагностика) =====
# Ловит порчу памяти (OOB write/read, use-after-free) в момент возникновения,
# а не там, где она случайно проявилась крашем позже — что критично, когда
# крах происходит в коде, который сам ничего не портит (см. springs-демку).
# Включается явно: cmake --preset x64-debug -DFE_ENABLE_ASAN=ON
option(FE_ENABLE_ASAN "Enable AddressSanitizer on all C/C++ targets" OFF)
if(FE_ENABLE_ASAN)
    if(MSVC)
        add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:/fsanitize=address>
                            $<$<COMPILE_LANGUAGE:C,CXX>:/Zi>)
    else()
        add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-fsanitize=address> -g)
        add_link_options(-fsanitize=address)
    endif()
    message(STATUS "AddressSanitizer enabled (FE_ENABLE_ASAN=ON)")
endif()

# ===== Зависимости =====
find_package(OpenGL REQUIRED)

# Пути к библиотекам
set(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs")

# --- GLFW ---
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
    set(GLFW_ROOT "${LIBS_DIR}/glfw")
    find_path(GLFW_INCLUDE_DIR
        NAMES GLFW/glfw3.h
        PATHS "${GLFW_ROOT}/include"
        NO_DEFAULT_PATH
    )
    find_library(GLFW_LIB
        NAMES glfw3 glfw3_mt glfw
        PATHS "${GLFW_ROOT}"
        PATH_SUFFIXES lib lib-vc2022 lib-vc2019 lib-vc2017 lib-mingw lib-mingw-w64 lib/x64 lib/x86
        NO_DEFAULT_PATH
    )
    if(NOT GLFW_INCLUDE_DIR OR NOT GLFW_LIB)
        message(FATAL_ERROR "GLFW not found. Put it in ${LIBS_DIR}/glfw/")
    endif()
else()
    set(GLFW_LIB ${glfw3_LIBRARIES})
    set(GLFW_INCLUDE_DIR ${glfw3_INCLUDE_DIR})
endif()

# --- GLAD ---
set(GLAD_INCLUDE_DIR "${LIBS_DIR}/glad/include")
set(GLAD_SRC "${LIBS_DIR}/glad/src/glad.c")
if(NOT EXISTS "${GLAD_SRC}")
    message(FATAL_ERROR "GLAD not found at ${GLAD_SRC}")
endif()

# --- ImGui (опционально) ---
set(IMGUI_DIR "${LIBS_DIR}/imgui")
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/imgui_impl_opengl3.cpp
)
set(IMGUI_AVAILABLE TRUE)
foreach(src ${IMGUI_SOURCES})
    if(NOT EXISTS "${src}")
        set(IMGUI_AVAILABLE FALSE)
        break()
    endif()
endforeach()

if(IMGUI_AVAILABLE)
    message(STATUS "ImGui found – GUI support enabled")
    add_compile_definitions(TESSERA_IMGUI_ENABLED)
else()
    message(WARNING "ImGui not found – building without GUI support")
    set(IMGUI_SOURCES "")
endif()

# --- GLM ---
set(GLM_INCLUDE_DIR "${LIBS_DIR}/glm")
if(NOT EXISTS "${GLM_INCLUDE_DIR}/glm/glm.hpp")
    message(FATAL_ERROR "GLM not found at ${GLM_INCLUDE_DIR}")
endif()

# --- CUDA (опционально) ---
# Если CUDA Toolkit найден, включаем GPU-бэкенд симуляции. Если нет — проект
# спокойно собирается без него (бэкенд по умолчанию CPU). Это и есть "форк"
# для машин с GPU и без: всё держится на макросе FE_CUDA_ENABLED.
option(FE_USE_CUDA "Build the CUDA GPU simulation backend if available" ON)
set(FE_CUDA_ENABLED FALSE)
if(FE_USE_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        find_package(CUDAToolkit REQUIRED)
        set(FE_CUDA_ENABLED TRUE)
        if(NOT DEFINED CMAKE_CUDA_STANDARD)
            set(CMAKE_CUDA_STANDARD 17)
        endif()
        # Архитектуры GPU. 86 = Ampere (RTX 30xx). При желании расширьте список.
        if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
            set(CMAKE_CUDA_ARCHITECTURES 86-real)
        endif()
        set(CUDA_BACKEND_SRC
            "${CMAKE_CURRENT_SOURCE_DIR}/engine/simulation/CudaLifeBackend.cu"
            "${CMAKE_CURRENT_SOURCE_DIR}/engine/simulation/CudaSpringBackend.cu"
            "${CMAKE_CURRENT_SOURCE_DIR}/engine/simulation/LightWaveCuda.cu")
        message(STATUS "CUDA found – GPU simulation backend enabled (arch ${CMAKE_CUDA_ARCHITECTURES})")
    else()
        message(STATUS "CUDA not found – building CPU-only simulation backend")
    endif()
else()
    message(STATUS "FE_USE_CUDA=OFF – building CPU-only simulation backend")
endif()

# ===== Исходники движка =====
file(GLOB_RECURSE ENGINE_SOURCES
    "engine/*.cpp"
    "engine/*.c"
)
add_library(EngineLib STATIC ${ENGINE_SOURCES} ${GLAD_SRC} ${IMGUI_SOURCES})
target_include_directories(EngineLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${GLAD_INCLUDE_DIR}
    ${GLM_INCLUDE_DIR}
)
if(IMGUI_AVAILABLE)
    target_include_directories(EngineLib PUBLIC ${IMGUI_DIR})
endif()
target_link_libraries(EngineLib PUBLIC OpenGL::GL ${GLFW_LIB})

# --- Подключение CUDA-бэкенда к движку ---
if(FE_CUDA_ENABLED)
    target_sources(EngineLib PRIVATE ${CUDA_BACKEND_SRC})
    target_compile_definitions(EngineLib PUBLIC FE_CUDA_ENABLED)
    target_link_libraries(EngineLib PUBLIC CUDA::cudart)
    # cub/thrust (CCCL) — для CudaSpringBackend.cu (cub::DeviceReduce::Sum).
    if(CUDAToolkit_INCLUDE_DIRS)
        target_include_directories(EngineLib PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    endif()
    # cub требует стандартный (не "traditional") препроцессор MSVC — иначе
    # cl.exe падает с fatal error C1189 прямо в заголовках cub. Ограничиваем
    # флаг только этим файлом — CudaLifeBackend.cu собирается без него.
    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/engine/simulation/CudaSpringBackend.cu"
        PROPERTIES COMPILE_OPTIONS "-Xcompiler=/Zc:preprocessor"
    )
    # Разрешаем .cu видеть те же include-пути, что и движок (для "engine/..." путей)
    set_target_properties(EngineLib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

# ===== Копирование ассетов (один раз, чтобы цели не гонялись за одну папку) =====
# Раньше каждый exe копировал Shaders/patterns в общий build-каталог в своём
# POST_BUILD — при параллельной сборке шаги гонялись и иногда падали. Теперь одна
# цель copy_assets делает это один раз, а приложения от неё зависят.
add_custom_target(copy_assets ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Shaders" "${CMAKE_BINARY_DIR}/Shaders"
    COMMENT "Copying shaders"
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/demo/life/patterns")
    add_custom_command(TARGET copy_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/demo/life/patterns" "${CMAKE_BINARY_DIR}/patterns"
        COMMENT "Copying patterns"
    )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/demo/worm/data")
    add_custom_command(TARGET copy_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/demo/worm/data" "${CMAKE_BINARY_DIR}/worm_data"
        COMMENT "Copying worm connectome data"
    )
endif()
# ffmpeg.exe для ScreenRecorder (engine/utils/ScreenRecorder.cpp) - БОЛЬШЕ НЕ
# вендорится в git: 97MB бинарник почти упирался в жёсткий лимит GitHub на
# 100MB/файл (было 3MB запаса) и раздувал репозиторий каждым обновлением
# бинарника навсегда - см. git-историю (полностью вычищен из истории).
# ЛОКАЛЬНО-ПЕРВЫЙ поиск: libs/ffmpeg/bin/ffmpeg.exe (по-прежнему в
# .gitignore, класть туда руками, никогда не коммитить) проверяется РАНЬШЕ
# скачивания - на медленном/VPN-соединении (см. TIMEOUT ниже - до 30 минут
# на ~100KB/s замерено на одной машине) один раз положить файл в libs/ffmpeg
# и никогда больше не ждать повторной загрузки при чистой пересборке
# (out/build чистится чаще, чем libs/). Если файла там нет - качается ОДИН
# РАЗ в CMAKE_BINARY_DIR (EXISTS-кэш не даёт повторных загрузок на каждый
# cmake configure). В обоих случаях кладётся РЯДОМ С ИСПОЛНЯЕМЫМИ ФАЙЛАМИ тем
# же соглашением, что worm_data/Shaders (ScreenRecorder ищет его по
# относительному пути "ffmpeg.exe" от текущей рабочей директории, см.
# Application::kFfmpegRelPath).
#
# TESSERA_FFMPEG_ENABLED=OFF - совсем не качать (например, оффлайн-сборка
# или сознательно не нужен MP4-бэкенд); локальный libs/ffmpeg по-прежнему
# используется, если лежит - OFF выключает только СКАЧИВАНИЕ, не отменяет
# уже найденный локальный файл. Как и раньше, сборка не падает ни при
# отсутствующем бинарнике, ни при неудаче скачивания - ScreenRecorder просто
# тихо использует GIF-бэкенд вместо видео.
option(TESSERA_FFMPEG_ENABLED "Fetch ffmpeg.exe for MP4 screen recording if not found locally (ScreenRecorder); OFF = don't download (local libs/ffmpeg still used if present), otherwise GIF-only fallback" ON)
set(TESSERA_FFMPEG_LOCAL_EXE "${CMAKE_CURRENT_SOURCE_DIR}/libs/ffmpeg/bin/ffmpeg.exe")
if(EXISTS "${TESSERA_FFMPEG_LOCAL_EXE}")
    message(STATUS "ffmpeg.exe found locally at libs/ffmpeg/bin/ - skipping download entirely")
    add_compile_definitions(TESSERA_FFMPEG_ENABLED)
    set(TESSERA_FFMPEG_CACHE_EXE "${TESSERA_FFMPEG_LOCAL_EXE}")
    add_custom_command(TARGET copy_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${TESSERA_FFMPEG_CACHE_EXE}" "${CMAKE_BINARY_DIR}/ffmpeg.exe"
        COMMENT "Copying ffmpeg.exe for screen recording (local libs/ffmpeg)"
    )
elseif(TESSERA_FFMPEG_ENABLED)
    # Директориальное свойство - применяется ко ВСЕМ таргетам в этом
    # CMakeLists.txt независимо от порядка (EngineLib уже объявлен выше по
    # тексту файла) - та же гарантия, что и у TESSERA_IMGUI_ENABLED. Отражает
    # ВЫБОР пользователя (OFF = не хочет ffmpeg вовсе), а не факт наличия
    # файла на диске - тот отдельно проверяется в рантайме (fileExists в
    # ScreenRecorder.cpp), макрос не даёт ScreenRecorder воспользоваться
    # случайно оставшимся от прошлой сборки ffmpeg.exe при явном OFF.
    add_compile_definitions(TESSERA_FFMPEG_ENABLED)
    set(TESSERA_FFMPEG_CACHE_DIR "${CMAKE_BINARY_DIR}/_deps/ffmpeg")
    set(TESSERA_FFMPEG_CACHE_EXE "${TESSERA_FFMPEG_CACHE_DIR}/ffmpeg.exe")
    if(NOT EXISTS "${TESSERA_FFMPEG_CACHE_EXE}")
        set(TESSERA_FFMPEG_ZIP "${TESSERA_FFMPEG_CACHE_DIR}/ffmpeg.zip")
        file(MAKE_DIRECTORY "${TESSERA_FFMPEG_CACHE_DIR}")
        message(STATUS "TESSERA_FFMPEG_ENABLED=ON: downloading ffmpeg.exe (~90MB essentials build, one-time, cached in build dir - can take a while on a slow/VPN connection, ~100KB/s measured on one machine gave ~19 min)...")
        # TIMEOUT 1800 (30 мин) - 300с оказалось МАЛО: реально замерено на
        # этой машине ~1114с (18.5 мин) на ~100KB/s (VPN) для тех же 111MB.
        # Качается только один раз - EXISTS-кэш выше не даёт повторных
        # попыток при следующих cmake configure, так что щедрый timeout не
        # стоит ничего при нормальном интернете и просто не рвёт медленный.
        file(DOWNLOAD "https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip"
             "${TESSERA_FFMPEG_ZIP}" STATUS TESSERA_FFMPEG_DL_STATUS SHOW_PROGRESS
             TIMEOUT 1800 TLS_VERIFY ON)
        list(GET TESSERA_FFMPEG_DL_STATUS 0 TESSERA_FFMPEG_DL_CODE)
        list(GET TESSERA_FFMPEG_DL_STATUS 1 TESSERA_FFMPEG_DL_MSG)
        if(TESSERA_FFMPEG_DL_CODE EQUAL 0)
            file(ARCHIVE_EXTRACT INPUT "${TESSERA_FFMPEG_ZIP}" DESTINATION "${TESSERA_FFMPEG_CACHE_DIR}")
            file(GLOB_RECURSE TESSERA_FFMPEG_FOUND "${TESSERA_FFMPEG_CACHE_DIR}/*/bin/ffmpeg.exe")
            if(TESSERA_FFMPEG_FOUND)
                list(GET TESSERA_FFMPEG_FOUND 0 TESSERA_FFMPEG_FOUND_ONE)
                file(COPY "${TESSERA_FFMPEG_FOUND_ONE}" DESTINATION "${TESSERA_FFMPEG_CACHE_DIR}")
            endif()
        else()
            message(WARNING "ffmpeg.exe download failed (${TESSERA_FFMPEG_DL_CODE}: ${TESSERA_FFMPEG_DL_MSG}) - ScreenRecorder will fall back to GIF backend.")
        endif()
        file(REMOVE "${TESSERA_FFMPEG_ZIP}")
    endif()
    if(EXISTS "${TESSERA_FFMPEG_CACHE_EXE}")
        add_custom_command(TARGET copy_assets POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${TESSERA_FFMPEG_CACHE_EXE}" "${CMAKE_BINARY_DIR}/ffmpeg.exe"
            COMMENT "Copying ffmpeg.exe for screen recording"
        )
    endif()
endif()

# Собрать одно приложение из main.cpp (+ соседние .cpp/.c в той же папке).
function(fe_add_app MAIN_CPP TARGET_NAME)
    get_filename_component(APP_DIR "${MAIN_CPP}" DIRECTORY)
    file(GLOB_RECURSE APP_EXTRA "${APP_DIR}/*.cpp" "${APP_DIR}/*.c")
    list(REMOVE_ITEM APP_EXTRA "${MAIN_CPP}")
    add_executable(${TARGET_NAME} "${MAIN_CPP}" ${APP_EXTRA})
    add_dependencies(${TARGET_NAME} copy_assets)
    target_link_libraries(${TARGET_NAME} PRIVATE EngineLib)
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${GLFW_INCLUDE_DIR}
        ${GLAD_INCLUDE_DIR}
        ${GLM_INCLUDE_DIR}
    )
    if(IMGUI_AVAILABLE)
        target_include_directories(${TARGET_NAME} PRIVATE ${IMGUI_DIR})
    endif()
    message(STATUS "Added ${TARGET_NAME} (${MAIN_CPP})")
endfunction()

# Обойти дерево, собрать приложения из каждого main.cpp. Имя цели — из пути
# относительно базовой папки: demo/life/full -> Demo_life_full, tests/capture -> Test_capture.
function(fe_discover_apps BASE_DIR PREFIX)
    file(GLOB_RECURSE MAINS "${BASE_DIR}/*.cpp")
    foreach(MAIN ${MAINS})
        get_filename_component(FN "${MAIN}" NAME)
        if(FN STREQUAL "main.cpp")
            get_filename_component(MDIR "${MAIN}" DIRECTORY)
            file(RELATIVE_PATH REL "${BASE_DIR}" "${MDIR}")
            string(REPLACE "/" "_" NAME "${REL}")
            fe_add_app("${MAIN}" "${PREFIX}_${NAME}")
        endif()
    endforeach()
endfunction()

# ===== Демки (игры) и тесты/бенчмарки =====
fe_discover_apps("${CMAKE_CURRENT_SOURCE_DIR}/demo"  "Demo")
fe_discover_apps("${CMAKE_CURRENT_SOURCE_DIR}/tests" "Test")

# ===== CTest-регистрация (только настоящие, argument-free, PASS/FAIL тесты) =====
# fe_discover_apps выше собирает ВСЕ tests/*/main.cpp как исполняемые цели,
# включая десятки одноразовых калибровочных харнессов (свои CLI-аргументы,
# свой формат вывода, не задуманы как pass/fail regression) - регистрировать
# их ВСЕ в ctest бессмысленно и вводит в заблуждение (все "провалятся" без
# нужных аргументов). Регистрируем явно только то, что реально устроено как
# regression-тест: без аргументов, чёткий PASS/FAIL, ненулевой exit code при
# провале. `ctest --test-dir out/build/<preset>` запускает оба одной командой.
if(TARGET Test_worm_locomotion)
    add_test(NAME Test_worm_locomotion COMMAND Test_worm_locomotion)
endif()
if(TARGET Test_worm_regression)
    add_test(NAME Test_worm_regression COMMAND Test_worm_regression)
endif()