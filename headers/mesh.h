#ifndef MESH_H
#define MESH_H
#include <Eigen>
#include "logger.h"
#include "matrix.h"
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <string>

using namespace Eigen;
using namespace std;

//Observation! Assuming triangular faces and 3 coordinates per vertex

class Mesh
{
private:
    std::vector<GLfloat>* m_vertices; //array of vertices
    std::vector<GLfloat>* m_normals; //array of normals
    std::vector<GLfloat>* m_texcoords; //array of texture coordinates
    std::vector<GLuint>* m_faceIndices; //TO DO: add vt/vn index info

    vec3 m_position;
    unsigned int m_edgeCount;
    unsigned int m_vertCount;
    unsigned int m_faceCount;
    unsigned int m_texCoordCount;
    SparseMatrix<int>* m_M;
    SparseMatrix<float>* m_D;

    GLuint m_vboPosition;
    GLuint m_vboIndices;
    GLuint m_vboNormals;
    GLuint m_vboTextureCoord;
    GLuint m_vao;


public:
    Mesh();
    ~Mesh();
    bool loadMesh(const char* _fileName);
    void calculateNormals();
    void normaliseNormals();
    void normaliseMesh();
    void bindVAO();
    void buildArcNodeMatrix();
    void buildVertexMatrix();
    void changeVertsBasedOn_D_Matrix();

    //Setters and getters
    std::vector<GLfloat>* getVertices(){return m_vertices;}
    std::vector<GLfloat>* getNormals(){return m_normals;}
    unsigned int getVertCount() { return m_vertCount;}
    unsigned int getFaceCount() { return m_faceCount;}
    unsigned int getEdgeCount() { return m_edgeCount;}
    unsigned int getTexCoordCount() { return m_texCoordCount;}
    GLuint getVAO() { return m_vao;}
    void setVAO(GLuint _vao) { m_vao = _vao;}
    SparseMatrix<int>* getM(){ return m_M;}
    SparseMatrix<float>* getD(){ return m_D;}
    float x() { return m_position.v[0]; }
    float y() { return m_position.v[1]; }
    float z() { return m_position.v[2]; }
    Vector3f getNormal(unsigned int _vertNo);

};
#endif // MESH_H
