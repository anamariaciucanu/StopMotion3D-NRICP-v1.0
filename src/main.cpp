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
    NRICP* nrICP = glfw_container->getNRICP();

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

    vec3 ray3 = vec3(ray_loc.v[0], ray_loc.v[1], ray_loc.v[2]);
    normalise(ray3);
    vec4 camera(cam->x(), cam->y(), cam->z(), 1);
    vec4 camera_loc = inverse(glfw_container->getModelMatrix())*camera;

    Vector3f _cam = Vector3f(camera_loc.v[0], camera_loc.v[1], camera_loc.v[2]);
    Vector3f _ray = Vector3f(ray3.v[0], ray3.v[1], ray3.v[2]);

    int intersection = nrICP->getTemplate()->whereIsIntersectingMesh(true, -1, _cam, _ray);

    if(intersection >= 0)
    {
        nrICP->setTemplateAuxIndex(intersection);
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


