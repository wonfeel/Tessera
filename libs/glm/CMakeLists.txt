cmake_minimum_required(VERSION 3.14)
project(Tessera VERSION 1.0 LANGUAGES CXX C)

# ===== Настройки C++ =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# --- ImGui ---
set(IMGUI_DIR "${LIBS_DIR}/imgui")
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/imgui_impl_opengl3.cpp
)
foreach(src ${IMGUI_SOURCES})
    if(NOT EXISTS "${src}")
        message(FATAL_ERROR "ImGui source missing: ${src}")
    endif()
endforeach()

# --- GLM ---
set(GLM_INCLUDE_DIR "${LIBS_DIR}/glm")
if(NOT EXISTS "${GLM_INCLUDE_DIR}/glm/glm.hpp")
    message(FATAL_ERROR "GLM not found at ${GLM_INCLUDE_DIR}")
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
    ${IMGUI_DIR}
)
target_link_libraries(EngineLib PUBLIC OpenGL::GL ${GLFW_LIB})

# OpenMP (для некоторых демок)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(EngineLib PUBLIC OpenMP::OpenMP_CXX)
    if(MSVC)
        target_compile_options(EngineLib PUBLIC /openmp)
    else()
        target_compile_options(EngineLib PUBLIC -fopenmp)
    endif()
endif()

# ===== Автоматическое обнаружение демок =====
file(GLOB DEMO_DIRS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/demo" "demo/*")
foreach(DEMO_DIR ${DEMO_DIRS})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/demo/${DEMO_DIR}")
        # Ищем main.cpp в папке демки
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/demo/${DEMO_DIR}/main.cpp")
            set(DEMO_NAME "Demo_${DEMO_DIR}")
            add_executable(${DEMO_NAME}
                "demo/${DEMO_DIR}/main.cpp"
            )
            target_link_libraries(${DEMO_NAME} PRIVATE EngineLib)
            target_include_directories(${DEMO_NAME} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${GLFW_INCLUDE_DIR}
                ${GLAD_INCLUDE_DIR}
                ${GLM_INCLUDE_DIR}
                ${IMGUI_DIR}
            )
            # Копирование шейдеров
            add_custom_command(TARGET ${DEMO_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/Shaders"
                "$<TARGET_FILE_DIR:${DEMO_NAME}>/Shaders"
                COMMENT "Copying shaders for ${DEMO_NAME}"
            )
            message(STATUS "Added demo: ${DEMO_NAME}")
        else()
            message(WARNING "No main.cpp found in demo/${DEMO_DIR}, skipping")
        endif()
    endif()
endforeach()

# Общие настройки компиляции
if(MSVC)
    add_compile_options(/W4 /wd4100 /wd4189)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()