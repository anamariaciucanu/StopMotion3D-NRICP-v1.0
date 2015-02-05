#include "logger.h"


Logger* Logger::m_logger;
Logger::Logger(){
    //read file

}

Logger* Logger::getInstance(){
    if(!m_logger)
    {
        m_logger = new Logger();
    }
    return m_logger;
}

Logger::~Logger(){
    if (m_logger)
    {
        delete m_logger;
    }
}

//opens log file and writes the date and time in it
bool Logger::restart_gl_log()
{
    FILE* file = fopen(GL_LOG_FILE, "w");
    if (!file)
    {
        fprintf (stderr, "ERROR: could not open GL_LOG_FILE log file %s for writing\n", GL_LOG_FILE);
        return false;
    }

    time_t now = time (NULL);
    char* date = ctime(&now);
    fprintf(file, "GL_LOG_FILE log.local time %s\n", date);
    fclose(file);
    return true;
}


//opens log file and appends a message
bool Logger::gl_log (const char* message, ...)
{
    va_list argptr;
    FILE* file = fopen (GL_LOG_FILE, "a");
    if (!file) {
        fprintf (stderr, "ERROR: could not open GL_LOG_FILE %sfile for appending\n", GL_LOG_FILE);
        return false;
    }
    va_start (argptr, message);
    vfprintf(file, message, argptr);
    va_end(argptr);
    fclose(file);
    return true;
}


//opens log file, appends message and error log
bool Logger::gl_log_err(const char* message, ...){
    va_list argptr;
    FILE* file = fopen(GL_LOG_FILE, "a");
    if(!file)
    {
        fprintf(stderr, "ERROR: could not open GL_LOG_FILE %s file for appending\n", GL_LOG_FILE);
        return false;
    }

    va_start(argptr, message);
    vfprintf(file, message, argptr);
    va_end(argptr);
    va_start(argptr, message);
    vfprintf(stderr, message, argptr);
    va_end(argptr);
    fclose(file);
    return true;
}

//writes the graphics card parameters in the log file
void Logger::log_gl_params()
{
 GLenum params[] = {
     GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS,
     GL_MAX_CUBE_MAP_TEXTURE_SIZE,
     GL_MAX_DRAW_BUFFERS,
     GL_MAX_FRAGMENT_UNIFORM_COMPONENTS,
     GL_MAX_TEXTURE_IMAGE_UNITS,
     GL_MAX_TEXTURE_SIZE,
     GL_MAX_VARYING_FLOATS,
     GL_MAX_VERTEX_ATTRIBS,
     GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS,
     GL_MAX_VERTEX_UNIFORM_COMPONENTS,
     GL_MAX_VIEWPORT_DIMS,
     GL_STEREO
    };

 const char* names[] = {
     "GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS",
     "GL_MAX_CUBE_MAP_TEXTURE_SIZE",
     "GL_MAX_DRAW_BUFFERS",
     "GL_MAX_FRAGMENT_UNIFORM_COMPONENTS",
     "GL_MAX_TEXTURE_IMAGE_UNITS",
     "GL_MAX_TEXTURE_SIZE",
     "GL_MAX_VARYING_FLOATS",
     "GL_MAX_VERTEX_ATTRIBS",
     "GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS",
     "GL_MAX_VERTEX_UNIFORM_COMPONENTS",
     "GL_MAX_VIEWPORT_DIMS",
     "GL_STEREO"
    };

    gl_log("GL Context Parameters:\n");

    //integers - only works if the orderis 0-10 integer return types
    for(int i=0; i<10; i++)
    {
     int v = 0;
     glGetIntegerv(params[i], &v);
     gl_log("%s %i\n", names[i], v);
    }

    //others
    int v[2];
    v[0]=v[1]=0;
    glGetIntegerv(params[10], v);
    gl_log("%s %i %i\n", names[10], v[0], v[1]);

    unsigned char s = 0;
    glGetBooleanv(params[11], &s);
    gl_log("%s %u\n", names[11], (unsigned int)s);
    gl_log("------------------------------------------\n");
}


//print programme log
void Logger::print_programme_info_log(GLuint programme)
{
     int max_length = 2048;
     int actual_length = 0;
     char log[2048];
     glGetProgramInfoLog(programme, max_length, &actual_length, log);
     printf("Program info log for GL index %u:\n%s", programme, log);
}

//print shader log
void Logger::print_shader_info_log(GLuint shader_index)
{
     int max_length = 2048;
     int actual_length = 0;
     char log[2048];
     glGetShaderInfoLog(shader_index, max_length, &actual_length, log);
     printf("Shader info log for GL index %u: \n %s \n", shader_index, log);
}



