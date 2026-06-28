#include "engine/graphics/Shader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

Shader::Shader(const std::string& vertPath, const std::string& fragPath) {
    std::string vertCode = readFile(vertPath);
    std::string fragCode = readFile(fragPath);
    const char* vSrc = vertCode.c_str();
    const char* fSrc = fragCode.c_str();

    unsigned int vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vSrc, nullptr);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");

    unsigned int fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fSrc, nullptr);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");

    programID = glCreateProgram();
    glAttachShader(programID, vertex);
    glAttachShader(programID, fragment);
    glLinkProgram(programID);
    checkCompileErrors(programID, "PROGRAM");

    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

Shader::~Shader() {
    glDeleteProgram(programID);
}

void Shader::use() const {
    glUseProgram(programID);
}

void Shader::setInt(const std::string& name, int value) {
    int loc = getUniformLocation(name);
    if (loc != -1)
        glUniform1i(loc, value);
}

int Shader::getUniformLocation(const std::string& name) const {
    auto it = m_uniformCache.find(name);
    if (it != m_uniformCache.end()) {
        return it->second;
    }
    int location = glGetUniformLocation(programID, name.c_str());
    m_uniformCache[name] = location;
    return location;
}

void Shader::setMat4(const std::string& name, const glm::mat4& mat) {
    int loc = getUniformLocation(name);
    if (loc != -1)
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(mat));
}

void Shader::setFloat(const std::string& name, float value) {
    int loc = getUniformLocation(name);
    if (loc != -1)
        glUniform1f(loc, value);
}

void Shader::checkCompileErrors(unsigned int shader, const std::string& type) {
    int success;
    char infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "Shader compilation error (" << type << "):\n" << infoLog << std::endl;
        }
    }
    else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "Program linking error:\n" << infoLog << std::endl;
        }
    }
}

std::string Shader::readFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << path << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}