#ifndef SHADER_H
#define SHADER_H
#include "logger.h"
#include "matrix.h"
#include <vector>

class Shader {

private:
   GLuint m_vertexShader;
   GLuint m_fragmentShader;
   GLuint m_shaderProgramme;
   Logger* m_logger;

public:
    Shader(const char* _fileVertexShader, const char* _fileFragmentShader);
    ~Shader();

    bool loadVertexShader(const char* _fileVertexShader);
    bool loadFragmentShader(const char *_fileFragmentShader);
    bool createShaderProgramme();
    void sendModelMatrixToShader(mat4 *_modelMatrix);
    void sendViewMatrixToShader(mat4 *_viewMatrix);
    void sendProjMatrixToShader(mat4 *_projMatrix);
    void sendCameraRayToShader(vec3 _ray);
    void sendColourChoiceToShader(vec3 _colour);
    void sendColourPickedToShader(vec3 _colour);
    void sendVertexIndicesToShader(int _picked, std::vector<int> *_indices);
    void sendChosenIndexToShader(int _chosen);

    GLuint getShaderProgramme() { return m_shaderProgramme; }
};
#endif // SHADER_H
