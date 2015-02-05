#ifndef LOGGER_H
#define LOGGER_H
#define GL_LOG_FILE "../logs/gl.log"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <cstdlib>
#include <time.h>
#include <stdarg.h>
#include <assert.h>
#include <math.h>

class Logger
{
private:
    Logger();
    static Logger* m_logger;

public:
    ~Logger();
    static Logger* getInstance();

    bool restart_gl_log();
    bool gl_log (const char* message, ...);
    bool gl_log_err(const char* message, ...);
    void log_gl_params();
    void print_programme_info_log(GLuint programme);
    void print_shader_info_log(GLuint shader_index);
};

#endif // LOGGER_H
