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
    std::vector<GLfloat>* m_vertNormalLines; //array of vertex <-> vertex + normal lines
    std::vector<GLfloat>* m_texcoords; //array of texture coordinates
    std::vector<GLuint>* m_faceIndices; //TO DO: add vt/vn index info

    vec3 m_position;
    unsigned int m_edgeCount;
    unsigned int m_vertCount;
    unsigned int m_faceCount;
    unsigned int m_texCoordCount;
    std::map <std::pair<unsigned int, unsigned int>, short >* m_adjMat;
    SparseMatrix<float>* m_D;

    GLuint m_vbo1Position;
    GLuint m_vbo1Indices;
    GLuint m_vbo1Normals;
    GLuint m_vbo2Position;
    GLuint m_vboTextureCoord;
    GLuint m_vao1;
    GLuint m_vao2;

    //Aux
    unsigned int m_pickedVertexIndex;



public:
    Mesh();
    ~Mesh();
    bool loadMesh(const char* _fileName, float *_transformations);
    void calculateNormals();
    void normaliseMesh();
    void bindVAO1();
    void bindVAO2();
    void buildArcNodeMatrix();
    void buildVertexMatrix();
    void  buildVertexNormalVector();
    void changeVertsBasedOn_D_Matrix();
    int whereIsIntersectingMesh(bool _culling, int _originTemplateIndex, Vector3f _origin, Vector3f _ray);

    //Setters and getters
    std::vector<GLfloat>* getVertices(){return m_vertices;}
    std::vector<GLfloat>* getNormals(){return m_normals;}
    unsigned int getVertCount() { return m_vertCount;}
    unsigned int getFaceCount() { return m_faceCount;}
    unsigned int getEdgeCount() { return m_edgeCount;}
    unsigned int getTexCoordCount() { return m_texCoordCount;}

    GLuint getVAO1() { return m_vao1;}
    void setVAO1(GLuint _vao) { m_vao1 = _vao;}

    GLuint getVAO2() { return m_vao2;}
    void setVAO2(GLuint _vao) { m_vao2 = _vao;}

    std::map <std::pair<unsigned int, unsigned int>, short >* getAdjMat(){ return m_adjMat;}
    SparseMatrix<float>* getD(){ return m_D;}
    float x() { return m_position.v[0]; }
    float y() { return m_position.v[1]; }
    float z() { return m_position.v[2]; }
    Vector3f getNormal(unsigned int _vertNo);
    Vector3f getVertex(unsigned int _vertNo);
    void rotateObject(float _angleX, float _angleY, float _angleZ);
    void moveObject(float _tX, float _tY, float _tZ);
    void normaliseNormals();
    float euclideanDistance(Vector3f _v1, Vector3f _v2);

    //Aux
    int getPickedVertexIndex(){ return m_pickedVertexIndex; }
    void setPickedVertexIndex(int _index) { m_pickedVertexIndex = _index; }

};
#endif // MESH_H
