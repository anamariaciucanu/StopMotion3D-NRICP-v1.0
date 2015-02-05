#ifndef MYGLFWWINDOW_H
#define MYGLFWWINDOW_H

#include "logger.h"
#include "matrix.h"
#include "camera.h"
#include "shader.h"
#include "NRICP.h"

using namespace std;

class GLFWContainer {
private:
    int m_gl_width; // 640;
    int m_gl_height; // 480;
    float m_objXRot;
    float m_objYRot;
    float m_objZRot;
    Logger* m_logger;
    GLFWwindow* m_window;
    Mesh** m_mesh;
    int m_meshCount;
    Camera* m_camera;
    Shader* m_shader;
    mat4 m_modelMat;
    mat4 m_viewMat;
    mat4 m_projMat;
    NRICP* m_nrICP;


public:
    GLFWContainer(int _width, int _height);
    ~GLFWContainer();

    bool initializeWindow();
    void initializeDrawing();
    void loadMesh(const char *_fileName);
    void loopDrawing();
    void checkKeyPress();
    void checkMeshIntersection(vec3 _ray);
    void update_fps_counter();
    void update_camera_position();
    void calculateTransformation();
    void printConfiguration();

    //Setters and getters
    GLFWwindow* getWindow() { return m_window; }
    void setWindow(GLFWwindow* _window){ m_window = _window; }
    void setWidth(int _width){ m_gl_width = _width; }
    int getWidth(){ return m_gl_width; }
    void setHeight(int _height){ m_gl_height = _height; }
    int getHeight(){ return m_gl_height; }
    Logger* getLogger(){ return m_logger; }
    mat4 getProjMatrix(){ return m_projMat; }
    mat4 getViewMatrix(){ return m_viewMat; }
};

#endif // MYGLFWWINDOW_H
