cmake_minimum_required(VERSION 3.18)
project(FieldEngine VERSION 1.0 LANGUAGES CXX C)

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
    add_compile_definitions(FLORA_IMGUI_ENABLED)
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
            set(CMAKE_CUDA_ARCHITECTURES 75 86)
        endif()
        set(CUDA_BACKEND_SRC "${CMAKE_CURRENT_SOURCE_DIR}/engine/simulation/CudaLifeBackend.cu")
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
    # Разрешаем .cu видеть те же include-пути, что и движок (для "engine/..." путей)
    set_target_properties(EngineLib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

# ===== Автоматическое обнаружение демок =====
file(GLOB DEMO_DIRS LIST_DIRECTORIES true RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/demo" "${CMAKE_CURRENT_SOURCE_DIR}/demo/*")
foreach(DEMO_DIR ${DEMO_DIRS})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/demo/${DEMO_DIR}")
        set(DEMO_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/demo/${DEMO_DIR}/main.cpp")
        if(EXISTS "${DEMO_MAIN}")
            # Собираем ВСЕ .cpp/.c из папки демки (включая вложенные)
            file(GLOB_RECURSE DEMO_EXTRA_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/demo/${DEMO_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/demo/${DEMO_DIR}/*.c")
            # Убираем main.cpp из списка, чтобы не дублировать
            list(REMOVE_ITEM DEMO_EXTRA_SOURCES "${DEMO_MAIN}")

            set(DEMO_NAME "Demo_${DEMO_DIR}")
            add_executable(${DEMO_NAME} "${DEMO_MAIN}" ${DEMO_EXTRA_SOURCES})
            target_link_libraries(${DEMO_NAME} PRIVATE EngineLib)
            target_include_directories(${DEMO_NAME} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${GLFW_INCLUDE_DIR}
                ${GLAD_INCLUDE_DIR}
                ${GLM_INCLUDE_DIR}
            )
            if(IMGUI_AVAILABLE)
                target_include_directories(${DEMO_NAME} PRIVATE ${IMGUI_DIR})
            endif()
            # Копирование шейдеров
            add_custom_command(TARGET ${DEMO_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/Shaders"
                "$<TARGET_FILE_DIR:${DEMO_NAME}>/Shaders"
                COMMENT "Copying shaders for ${DEMO_NAME}"
            )
            message(STATUS "Added demo: ${DEMO_NAME} (+ ${DEMO_EXTRA_SOURCES})")
        endif()
    endif()
endforeach()