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


void glfw_window_size_callback(GLFWwindow* _window, int _width, int _height)
{
    glfw_container->setWidth(_width);
    glfw_container->setHeight(_height);
}

void glfw_error_callback(int error, const char* description)
{
   glfw_container->getLogger()->gl_log_err("GLFW ERROR: code %i msg: %s\n", error, description);
}

//TODO -> parameters don't work
void glfw_mouse_click_callback(GLFWwindow* _window, int s, int mouse_x, int mouse_y)
{
//2D Viewport Coordinates -> 3D Normalised Device Coordinates
float x = (2.0f * mouse_x)/glfw_container->getWidth() - 1.0f;
float y = 1.0f - (2.0f * mouse_y)/glfw_container->getHeight();
float z = 1.0f;
vec3 ray_nds = vec3(x, y, z);

//-> Homogeneous Clip Coordinates
vec4 ray_clip = vec4(ray_nds.v[0], ray_nds.v[1], -1.0, 1.0);
//No need to reverse perspective division here
//4D Eye (Camera) Coordinates
vec4 ray_eye = inverse(glfw_container->getProjMatrix())*ray_clip;
ray_eye = vec4(ray_eye.v[0], ray_eye.v[1], -1.0, 0.0);

//4D World Coordinates
vec4 aux_wor= inverse(glfw_container->getViewMatrix())*ray_eye;
vec3 ray_wor = vec3(aux_wor.v[0], aux_wor.v[1], aux_wor.v[2]);
ray_wor = normalise(ray_wor);

//TO DO: Correct this dreadful thing
glfw_container->checkMeshIntersection(ray_wor);
}


int main(){
    omp_set_num_threads(8);
    Eigen::setNbThreads(8);
    Eigen::initParallel();

    glfw_container->initializeWindow();
    glfwSetWindowSizeCallback(glfw_container->getWindow(), glfw_window_size_callback);
    glfwSetErrorCallback(glfw_error_callback);
    //glfwSetMouseButtonCallback(glfw_container->getWindow(), glfw_mouse_click_callback);
    glfw_container->initializeDrawing();
    glfw_container->loopDrawing();

	return 0;
}


