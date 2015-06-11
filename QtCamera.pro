#QtCamera - StopMotion3D Laplace Editing v001

TARGET=QtCamera
OBJECTS_DIR=obj
MOC_DIR=moc
CONFIG-=app_bundle
CONFIG += console
QT+= opengl core
SOURCES+= src/main.cpp \
            src/matrix.cpp \
    src/glfwcontainer.cpp \
    src/shader.cpp \
    src/mesh.cpp \
    src/NRICP.cpp

HEADERS+= headers/matrix.h \
    headers/glfwcontainer.h \
    headers/load_functions.h \
    headers/camera.h \
    headers/shader.h \
    headers/mesh.h \
    headers/NRICP.h

INCLUDEPATH +=./headers

DESTDIR=./build
OTHER_FILES+= shaders/test.vert \
        shaders/test.frag \
        logs/gl.log


#Dependencies and Libraries  ===============================================================================
LIBS += -L./libs -lGLEW -lglfw3
LIBS += -L/usr/local/lib -lGL -lX11 -lXxf86vm -lXrandr -lpthread -lXi -lm -lXinerama -lXcursor

#Fonts
#-llibfreetype


#Eigen large matrix library
INCLUDEPATH += $$PWD/../../../Libs/eigen_1/Eigen
INCLUDEPATH += $$PWD/../../../Libs/eigen_1/Eigen/src
DEPENDPATH += $$PWD/../../../Libs/eigen_1/Eigen


#Multithreading
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LIBS += -lgomp



