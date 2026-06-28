#pragma once
#include <string>
#include <unordered_map>
#include <glm/glm.hpp>

class Shader {
public:
    Shader(const std::string& vertPath, const std::string& fragPath);
    ~Shader();

    void use() const;
    void setMat4(const std::string& name, const glm::mat4& mat);
    void setFloat(const std::string& name, float value);
    void setInt(const std::string& name, int value);   // <-- ДОБАВИТЬ
    unsigned int getID() const { return programID; }

private:
    unsigned int programID;
    mutable std::unordered_map<std::string, int> m_uniformCache;

    int getUniformLocation(const std::string& name) const;
    void checkCompileErrors(unsigned int shader, const std::string& type);
    std::string readFile(const std::string& path);
};