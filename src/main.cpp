/*
Compile with  g++ -o demo main.cpp libGLEW.a libglfw3.a -I include -lGL -lX11 -lXxf86vm -lXrandr -lpthread -lXi -lm -lXinerama -lXcursor
Use valgrind --leak-check=full to check for memory leaks.
*/

//Resources used:
//http://openglbook.com/
//Anton's OpenGL Tutorials Book
//www.opengl-tutorial.org



//TO DO: Memory release
//TO DO: Destroy buffers
//TO DO: Create vector of meshes in glfwcontainer.cpp
//TO DO: Calculate vertex normals from face normals

#include "glfwcontainer.h"


GLFWContainer* glfw_container = new GLFWContainer(1280, 720);


void calculateClickRay(double _mouseX, double _mouseY)
{
    Camera* cam = glfw_container->getCamera();

    //2D Viewport Coordinates -> 3D Normalised Device Coordinates
    float x = (2.0f * _mouseX)/glfw_container->getWidth() - 1.0f;
    float y = 1.0f - (2.0f * _mouseY)/glfw_container->getHeight();
    float z = 1.0f;
    vec3 ray_nds = vec3(x, y, z);
    //-> Homogeneous Clip Coordinates
    vec4 ray_clip = vec4(ray_nds.v[0], ray_nds.v[1], -1.0, 1.0);
    //No need to reverse perspective division here
    //4D Eye (Camera) Coordinates
    vec4 ray_eye = inverse(glfw_container->getProjMatrix())*ray_clip;
    ray_eye = vec4(ray_eye.v[0], ray_eye.v[1], -1.0, 0.0);
    //4D World Coordinates
    vec4 ray_wor = inverse(glfw_container->getViewMatrix())*ray_eye;
    //Local Coordinates
    vec4 ray_loc = inverse(glfw_container->getModelMatrix())*ray_wor;

    vec3 ray_aux = vec3(ray_loc.v[0], ray_loc.v[1], ray_loc.v[2]);
    normalise(ray_aux);
    vec4 cam_wor(cam->x(), cam->y(), cam->z(), 1);
    vec4 cam_loc = inverse(glfw_container->getModelMatrix())*cam_wor;

    Vector3f camera = Vector3f(cam_loc.v[0], cam_loc.v[1], cam_loc.v[2]);
    Vector3f ray = Vector3f(ray_aux.v[0], ray_aux.v[1], ray_aux.v[2]);

    Mesh* activeMesh =  glfw_container->getClickActiveMesh();
    int intersection = activeMesh->whereIsIntersectingMesh(true, -1, camera, ray);

    if(intersection >= 0)
    {
      activeMesh->setPickedVertexIndex(intersection);
    }
}


void glfw_window_size_callback(GLFWwindow* _window, int _width, int _height)
{
    glfw_container->setWidth(_width);
    glfw_container->setHeight(_height);
}

void glfw_error_callback(int error, const char* description)
{
   glfw_container->getLogger()->gl_log_err("GLFW ERROR: code %i msg: %s\n", error, description);
}

void mouseClickEvent(GLFWwindow *_window, int _button, int _action, int _mods)
{

  if(_button == GLFW_MOUSE_BUTTON_1 && _action == GLFW_PRESS)
  {
   double xpos;
   double ypos;
   glfwGetCursorPos(_window, &xpos, &ypos);
   calculateClickRay(xpos, ypos);
  }
}


int main(){
    omp_set_num_threads(8);
    Eigen::setNbThreads(8);
    Eigen::initParallel();

    glfw_container->initializeWindow();
    glfwSetWindowSizeCallback(glfw_container->getWindow(), glfw_window_size_callback);
    glfwSetMouseButtonCallback(glfw_container->getWindow(), mouseClickEvent);
    glfwSetErrorCallback(glfw_error_callback);
    glfw_container->initializeDrawing();
    glfw_container->loopDrawing();

	return 0;
}


