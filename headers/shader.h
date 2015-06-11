//TO DO: Comment code


#ifndef SHADER_H
#define SHADER_H
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "matrix.h"
#include <vector>

class Shader {

private:
   GLuint m_vertexShader;
   GLuint m_fragmentShader;
   GLuint m_shaderProgramme;

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
    void sendPickedIndexToShader(int _picked);
    void sendLandmarkIndicesToShader(int _size, std::vector<int>* _indices);

    GLuint getShaderProgramme() { return m_shaderProgramme; }
};
#endif // SHADER_H
