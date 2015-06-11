//TO DO: mat4 - turn to pointer


#ifndef MYGLFWWINDOW_H
#define MYGLFWWINDOW_H

///@file GLFWContainer.h
///@brief Defines a container which will hold the OpenGL drawing environment window, the elements to be drawn
/// and the procedures which will be performed on them
///@author Anamaria Ciucanu

#include "shader.h"
#include "matrix.h"
#include "camera.h"
#include "NRICP.h"

using namespace std;

class GLFWContainer
{
 private:
 ///@brief Integer variable -> width of the GLFW window
    int m_gl_width;
 ///@brief Integer variable -> height of the GLFW window
    int m_gl_height;
 ///@brief Float variables -> rotation angles around the X, Y and Z axes respectively
 ///These values will be includedin the model matrix (local->world coordinates)
    float m_objXRot;
    float m_objYRot;
    float m_objZRot;
 ///@brief Pointer to GLFW window where OpenGL drawing occurs
    GLFWwindow* m_window;
 ///@brief Pointer to an array of pointers to Mesh class objects
    Mesh** m_mesh;
 ///@brief Pointer to NRICP class object, which contains the nonrigid iterative closest point algorithm implementation for
 /// pose matching between any two meshes from the mesh array
    NRICP* m_nrICP;
 ///@brief Unsigned integer variable holding the number of meshes introduced in the mesh array
    unsigned int m_meshCount;
 ///@brief Unsigned integer variable determining the index from the mesh array of the currently active mesh
 /// This is used for the vertex selection method starting with the mouse click callback function from the main function
    unsigned int m_clickActiveMeshIndex;
 ///@brief Pointer to Camera class object
    Camera* m_camera;
 ///@brief Pointer to Shader class object
    Shader* m_shader;
 ///@brief Model, view and projection matrices
    mat4 m_modelMat;
    mat4 m_viewMat;
    mat4 m_projMat;


 public:
 ///@brief ctor of GLFWContainer class
 ///@param [in] _width -> width of the GLFW window
 ///@param [in] _height -> height of the GLFW window
    GLFWContainer(int _width, int _height);
 ///@brief dtor of GLFWContainer class
    ~GLFWContainer();
 ///@brief initializes the GLFW3 window and reports any errors in the log file
 ///@param [out] boolean value to report success of method
    bool initializeWindow();
 ///@brief initializes the OpenGL drawing context
    void initializeDrawing();
 ///@brief loads a mesh from the obj file
 ///@param [in] _fileName -> name of the obj file where the mesh was saved
 ///@param [in] _transformations -> array of initial transformations applied to the mesh
    void loadMesh(const char *_fileName, float *_transformations);
 ///@brief brings the meshes in the Mesh array to the [-1,1]^3 coordinates
    void normaliseMeshes();
 ///@brief the OpenGL drawing loop
    void loopDrawing();
 ///@brief checks if a key from the keyboard was pressed and performs the actions linked to that key
    void checkKeyPress();
 ///@brief NOT USED - sends an eye level ray (multiplied with the model-view matrix) to the shader to check for intersections with the mesh
    void checkMeshIntersection(vec3 _ray);
 ///@brief NOT USED - calculates and displays frames per second
    void update_fps_counter();
 ///@brief prints the camera position in the title bar every 0.2 seconds
    void update_camera_position();
 ///@brief updates titlebar with stiffness
    void update_titlebar();
 ///@brief prints the OpenGL version
    void printConfiguration();
 ///@brief loads landmarks from files and adds them to the NRICP object
    void loadLandmarks(const char* _templateFile, const char* _targetFile);


 /// Setters and Getters of the private variables
    void setWindow(GLFWwindow* _window){ m_window = _window; }
    GLFWwindow* getWindow() { return m_window; }
    void setWidth(int _width){ m_gl_width = _width; }
    int getWidth(){ return m_gl_width; }
    void setHeight(int _height){ m_gl_height = _height; }
    int getHeight(){ return m_gl_height; }
    mat4 getModelMatrix() { return m_modelMat; }
    mat4 getProjMatrix(){ return m_projMat; }
    mat4 getViewMatrix(){ return m_viewMat; }
    Camera* getCamera(){ return m_camera; }
    NRICP* getNRICP(){ return m_nrICP; }
    Shader* getShader() { return m_shader; }
    void setClickActiveMeshIndex(unsigned int _click) { m_clickActiveMeshIndex = _click; }
    unsigned int getClickActiveMeshIndex() { return m_clickActiveMeshIndex; }
    Mesh* getClickActiveMesh() { return m_mesh[m_clickActiveMeshIndex]; }
    bool getWireframe() {  return m_mesh[m_clickActiveMeshIndex]->isWireframe(); }
    void setWireframe(bool _value)
    {
        m_mesh[m_clickActiveMeshIndex]->setWireframe(_value);
    }

    void printCurvatureActiveVertex()
    {
        Mesh* mesh = m_mesh[m_clickActiveMeshIndex];
        int pickedIndex = mesh->getPickedVertexIndex();
        float vertexCurvature = mesh->calculateVertexCurvature(pickedIndex);

        printf("Curvature of vertex %i, of mesh %i is: ", pickedIndex, m_clickActiveMeshIndex);
        printf(" %f \n", vertexCurvature);
    }
};

#endif // MYGLFWWINDOW_H
