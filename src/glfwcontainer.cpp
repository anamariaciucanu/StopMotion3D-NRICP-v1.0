#include "glfwcontainer.h"
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <unistd.h>

GLFWContainer::GLFWContainer(int _width, int _height)
{
    m_gl_width = _width;
    m_gl_height = _height;
    m_meshCount = 0;
    m_objXRot = 0.0;
    m_objYRot = 0.0;
    m_objZRot = 0.0;
    m_mesh = new Mesh*;
    m_clickActiveMeshIndex = 0;
}

GLFWContainer::~GLFWContainer()
{
    if(m_camera)
    {
        delete m_camera;
    }

    if(m_shader)
    {
        delete m_shader;
    }

    if(m_nrICP)
    {
        delete m_nrICP;
    }

    int i=0;
    while (m_mesh[i] != NULL)
    {
        delete m_mesh[i];
        i++;
    }

//    if(m_window)
//    {
//        delete m_window;
//    }
}

bool GLFWContainer::initializeWindow()
{

  //Start GL context and OS window using the GLFW helper library
  if(!glfwInit())
   {
    fprintf(stderr, "ERROR:could not start GLFW3\n");
    return false;
   }

   m_window = glfwCreateWindow(m_gl_width, m_gl_height, "Stop Motion 3D", NULL, NULL);
   m_camera = new Camera(m_gl_width, m_gl_height);

   if (!m_window)
    {
     fprintf(stderr, "ERROR:could not open window with GLFW3\n");
     glfwTerminate();
     return false;
    }

    glfwMakeContextCurrent(m_window);

    return true;
}

void GLFWContainer::loadMesh(const char* _fileName)
{
    m_mesh[m_meshCount] = new Mesh();
    if(m_mesh[m_meshCount]->loadMesh(_fileName))
    {
   //  if(m_mesh[m_meshCount]->getNormals()->size() < 1)
    // {
         m_mesh[m_meshCount]->calculateNormals();
         m_mesh[m_meshCount]->bindVAO1();
    // }
     m_meshCount++;
    }
    else
    {
      printf("Mesh could not be loaded from file %s", _fileName);
    }
}

void GLFWContainer::normaliseMeshes()
{
    for(unsigned int i = 0; i < m_meshCount; ++i)
    {
        m_mesh[i]->normaliseMesh();
    }
}

void GLFWContainer::update_fps_counter()
{
     static double previous_seconds = glfwGetTime();
     static int frame_count = 0;
     double current_seconds = glfwGetTime();
     double elapsed_seconds = current_seconds - previous_seconds;

     if(elapsed_seconds > 0.25)
     {
          previous_seconds = current_seconds;
          double fps = (double)(frame_count/elapsed_seconds);
          char tmp[128];
          sprintf(tmp, "opengl @ fps: %.2f", fps);
          glfwSetWindowTitle(m_window, tmp);
          frame_count = 0;
     }
     frame_count++;
}

void GLFWContainer::update_camera_position()
 {
     static double previous_seconds = glfwGetTime();
     double current_seconds = glfwGetTime();
     double elapsed_seconds = current_seconds - previous_seconds;

     if(elapsed_seconds > 0.2)
     {
          previous_seconds = current_seconds;
          char tmp[128];
          sprintf(tmp, "Camera (%.2f, %.2f, %.2f)", m_camera->x(), m_camera->y(), m_camera->z());
          glfwSetWindowTitle(m_window, tmp);
     }
 }

void GLFWContainer::update_titlebar()
 {
     static double previous_seconds = glfwGetTime();
     double current_seconds = glfwGetTime();
     double elapsed_seconds = current_seconds - previous_seconds;

     if(elapsed_seconds > 0.2)
     {
          previous_seconds = current_seconds;
          char tmp[128];
          sprintf(tmp, "Stiffness %.2f ", m_nrICP->getStiffness());
          glfwSetWindowTitle(m_window, tmp);
     }
 }

void GLFWContainer::checkKeyPress()
{
    //To Do: put separate
    static double previous_seconds = glfwGetTime();
    double current_seconds = glfwGetTime();
    double elapsed_seconds = current_seconds - previous_seconds;
    previous_seconds = current_seconds;


    //control keys
    if(glfwGetKey(m_window, GLFW_KEY_A)){
      m_camera->translate(0, m_camera->getSpeed() * elapsed_seconds);
      m_camera->setMoved(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_D)){
      m_camera->translate(0, -m_camera->getSpeed() * elapsed_seconds);
      m_camera->setMoved(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_W)){
      m_camera->translate(2, -m_camera->getSpeed() * elapsed_seconds);
      m_camera->setMoved(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_S)){
      m_camera->translate(2, m_camera->getSpeed() * elapsed_seconds);
      m_camera->setMoved(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_E)){
     m_objXRot = m_objXRot < -360.0 ? 0.0 : m_objXRot - 0.1;
     m_camera->setRotated(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_R)){
     m_objXRot = m_objXRot > 360.0 ? 0.0 : m_objXRot + 0.1;
     m_camera->setRotated(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_F)){
     m_objYRot = m_objYRot < -360.0 ? 0.0 : m_objYRot - 0.1;
     m_camera->setRotated(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_G)){
     m_objYRot = m_objYRot > 360.0 ? 0.0 : m_objYRot + 0.1;
     m_camera->setRotated(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_V)){
     m_objZRot = m_objZRot < -360.0 ? 0.0 : m_objZRot - 0.1;
     m_camera->setRotated(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_B)){
     m_objZRot = m_objZRot > 360.0 ? 0.0 : m_objZRot + 0.1;
     m_camera->setRotated(true);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_1)){
     setClickActiveMeshIndex(0);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_2)){
     setClickActiveMeshIndex(1);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_L)){
      m_nrICP->addLandmarkCorrespondence();
      m_nrICP->setLandmarkCorrespChanged(true);
      sleep(1.0);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_K)){
        m_nrICP->clearLandmarkCorrespondences();
        m_nrICP->setLandmarkCorrespChanged(true);      
    }

    else if(glfwGetKey(m_window, GLFW_KEY_P))
    {
      m_mesh[0]->printLandmarkedPoints("../logs/landmarks_template.txt");
      m_mesh[1]->printLandmarkedPoints("../logs/landmarks_target.txt");
      sleep(1.0);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_Y))
    {
     m_nrICP->calculateRigidTransformation();
     sleep(1.0);
    }

    else if(glfwGetKey(m_window, GLFW_KEY_T))
    {
     m_nrICP->calculateNonRigidTransformation();
     m_nrICP->modifyStiffness(-1.0);
     m_nrICP->modifyBeta(-0.001);
     sleep(1.0);
    }
}

void GLFWContainer::checkMeshIntersection(vec3 _ray)
{
  m_shader->sendCameraRayToShader(_ray);
}

void GLFWContainer::loadLandmarks(const char* _templateFile, const char* _targetFile)
{
    ifstream in1(_templateFile, ios::in);
    if (!in1)
    {
     printf("Cannot open %s",_templateFile);
     return;
    }

    ifstream in2(_targetFile, ios::in);
    if (!in2)
    {
     printf("Cannot open %s",_targetFile);
     return;
    }


    string line1, line2;
    int l1, l2;
    while (getline(in1, line1) && getline(in2, line2))
    {
     istringstream s1(line1);
     istringstream s2(line2);

     s1 >> l1;
     s2 >> l2;

     //TO DO: Hardcoded
     m_mesh[0]->setPickedVertexIndex(l1);
     m_mesh[1]->setPickedVertexIndex(l2);
     m_nrICP->addLandmarkCorrespondence();
    }

    m_nrICP->setLandmarkCorrespChanged(true);
}

void GLFWContainer::printConfiguration()
{
    //get version info
    const GLubyte* renderer = glGetString(GL_RENDERER);
    const GLubyte* version = glGetString(GL_VERSION);
    printf("Renderer: %s\n", renderer);
    printf("OpenGL version supported %s\n", version);
}



//Drawing init and loop

void GLFWContainer::initializeDrawing()
{
    //Drawining inits =========================================================================
    //start GLEW extension handler
    glewExperimental = GL_TRUE;
    glewInit();

    //near clipping plane
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS); //smaller value as closer

    //drawing prefs
    glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glEnable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);


    //Load a scene ============================================================================
    float transformations[6] = {0.0, 0.0, 0.0, -0.7, 0.1, 0.0};
    loadMesh("../models/HelliDropter1.obj");
    m_mesh[0]->rotateObject(transformations[0], transformations[1], transformations[2]);
    m_mesh[0]->moveObject(transformations[3], transformations[4], transformations[5]);
    m_mesh[0]->calculateEigenvectors();

    transformations[3] = 0.7;
    loadMesh("../models/HelliDropter2.obj");
    m_mesh[1]->rotateObject(transformations[0], transformations[1], transformations[2]);
    m_mesh[1]->moveObject(transformations[3], transformations[4], transformations[5]);
    m_mesh[1]->calculateEigenvectors();

    //Nonrigid Iterative Closest Point ========================================================
    m_nrICP = new NRICP(m_mesh[0], m_mesh[1]);
    m_nrICP->initializeNRICP();

    loadLandmarks("../logs/landmarks_template.txt", "../logs/landmarks_target.txt");

    //Bind VAOs

    //Transformations =========================================================================
    //Model matrix
    //column major matrix - that's how shaders prefer it
     m_modelMat = identity_mat4();

    //View matrix
    mat4 T = translate(identity_mat4(), vec3(-m_camera->x(), -m_camera->y(), -m_camera->z()));
    mat4 R = rotate_y_deg(identity_mat4(), -m_camera->getYaw());
    m_viewMat = R*T;

    //Projection matrix
    //Change when resizing window
    m_projMat = mat4(
         m_camera->getSx(), 0.0f, 0.0f, 0.0f,
         0.0f, m_camera->getSy(), 0.0f, 0.0f,
         0.0f, 0.0f, m_camera->getSz(), -1.0f,
         0.0f, 0.0f, m_camera->getPz(), 0.0f
     );

    //Shaders ===========================================================================
    m_shader = new Shader("../shaders/test.vert", "../shaders/test.frag");
    m_shader->sendColourPickedToShader(vec3(0.8, 0.1, 0.1));

    //Matrix sending to shader ==========================================================
    m_shader->sendModelMatrixToShader(&m_modelMat);
    m_shader->sendViewMatrixToShader(&m_viewMat);
    m_shader->sendProjMatrixToShader(&m_projMat);
}

void GLFWContainer::loopDrawing()
{
    m_camera->setMoved(false);

    while (!glfwWindowShouldClose(m_window))
    {
     // Chekup loops =================================================

      checkKeyPress();
      update_titlebar();

     //General drawing loop ==========================================
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glViewport(0, 0, m_gl_width, m_gl_height);


     //Update Model Matrix ===========================================
      m_modelMat = identity_mat4();

      mat4 Rx = rotate_x_deg(identity_mat4(),  m_objXRot);
      mat4 Ry = rotate_y_deg(identity_mat4(),  m_objYRot);
      mat4 Rz = rotate_z_deg(identity_mat4(),  m_objZRot);
      m_modelMat = Rz*Ry*Rx*m_modelMat;
      m_shader->sendModelMatrixToShader(&m_modelMat);


      //Update View matrix ===========================================
      if(m_camera->isMoved())
      {
       mat4 T = translate(identity_mat4(), vec3(-m_camera->x(), -m_camera->y(), -m_camera->z()));
       mat4 R = rotate_y_deg(identity_mat4(), -m_camera->getYaw());
       m_viewMat = R*T;
       m_camera->setMoved(false);
       m_shader->sendViewMatrixToShader(&m_viewMat);
      }


     //Draw meshes

          for(unsigned int i=0; i<m_meshCount; ++i)
          {
           std::vector<int>* landmarks = m_mesh[i]->getLandmarkVertexIndices();

           vec3 col1 = (i%2==0) ? vec3(0.9, 0.6, 0.3) : vec3(0.3, 0.6, 0.9);
           m_shader->sendColourChoiceToShader(col1);
           m_shader->sendPickedIndexToShader(m_mesh[i]->getPickedVertexIndex());
           m_shader->sendLandmarkIndicesToShader(landmarks->size(), landmarks);
           glUseProgram(m_shader->getShaderProgramme());

           m_mesh[i]->bindVAO1();

           glBindVertexArray(m_mesh[i]->getVAO1());
           glDrawElements(GL_POINTS, m_mesh[i]->getFaceCount()*3, GL_UNSIGNED_INT, (GLvoid*)0);

           m_shader->sendLandmarkIndicesToShader(0, landmarks);
           if(m_mesh[i]->isWireframe())
           {
            glDrawElements(GL_LINES, m_mesh[i]->getFaceCount()*3, GL_UNSIGNED_INT, (GLvoid*)0);
           }
           else
           {
            glDrawElements(GL_TRIANGLES, m_mesh[i]->getFaceCount()*3, GL_UNSIGNED_INT, (GLvoid*)0);
           }

            m_mesh[i]->unbindVAO1();
          }


       //update input handling events
       glfwPollEvents();

       //Put stuff on display
       glfwSwapBuffers(m_window);

       //Exit on key press
       if(GLFW_PRESS == glfwGetKey(m_window, GLFW_KEY_ESCAPE))
        {
         glfwSetWindowShouldClose(m_window, 1);
        }

       usleep(5000);
    }

    //close GL contextand any other GLFW resources
    glfwTerminate();
}
