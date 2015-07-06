//OBS: Assuming triangular faces and 3 coordinates per vertex
//TO DO: m_vertNormalLines doesn't work as desired -> flat lines on mesh instead of normals?
//TO DO: add vt/vn index info from face indices section in obj file
//TO DO: use m_texcoords and m_texCoordCount for texturing
//TO DO: use either Vector3f, Matrix4f or vec3, mat4 etc. (Eigen library versus matrix.h)
//TO DO: affineTransformation input matrix _X should be 4x4 or 4x3 or 3x4?
//TO DO: spherical hierarchy for template meshes

#ifndef MESH_H
#define MESH_H

///@file mesh.h
///@brief Mesh class containing the geometry to be drawn in the scene
///@author Anamaria Ciucanu

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen>
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

struct Segmentation
{
  Vector3i m_plane;
  Segmentation* m_leftSegment;
  Segmentation* m_rightSegment;
  std::vector<GLuint>* m_leftFaces;
  std::vector<GLuint>* m_rightFaces;
  bool m_visited;
};


class Mesh
{
 private:
 ///@brief Pointer to vector of vertices GLfloats
 ///Every group of 3 floats represents a vertex
 ///This array will be sent to the shader
    std::vector<GLfloat>* m_vertices;
 ///@brief Pointer to vector of normals GLfloats
 ///Every group of 3 floats represents a normal
 ///This array will be sent to the shader
    std::vector<GLfloat>* m_normals;
 ///@brief Pointer to vector of normal lines GLfloats
 ///Every group of 6 floats represents a normal line (vertex <-> vertex + normal*t)
 ///This array will be sent to the shader to draw normals
    std::vector<GLfloat>* m_vertNormalLines;
 ///@brief Pointer to vector of texture coordinates GLfloats
 ///Every group of 2 floats represent the 2D coordinates on the texture image, assigned to the respective vertex
 ///This array will be sent to the shader to draw textures on meshes
    std::vector<GLfloat>* m_texcoords;
 ///@brief Pointer to vector of face indices GLunit
 ///Every group of 3 indices represents a triangle from the mesh
    std::vector<GLuint>* m_faceIndices;
 ///@brief Vector of size 3, representing the mesh's average position
    Vector3f m_position;
 ///@brief Unsigned integer variable -> the number of edges
    unsigned int m_edgeCount;
 ///@brief Unsigned integer variable -> the number of vertices
    unsigned int m_vertCount;
 ///@brief Unsigned integer variable -> the number of faces
    unsigned int m_faceCount;
 ///@brief Unsigned integer variable -> the number of texture coordinate pairs?
    unsigned int m_texCoordCount;
 ///@brief Pointer to a map of pairs of unsigned integers -> a list of directed edges (smaller index -> larger index)
 ///corresponds to the arc-node matrix
    std::map <std::pair<unsigned int, unsigned int>, short >* m_adjMat;
 ///@brief Pointer to a Sparse Matrix of GLfloats, from the Eigen library -> contains the vertex floats in a n x 4n matrix (n = m_vertCount)
    SparseMatrix<GLfloat>* m_D;
 ///@brief Neighbour list
    std::vector<std::vector<int> >* m_neighbours;
 ///@brief GLuint addresses for vertex buffer objects -> vertex position, face indices, normals, texture coordinates
    GLuint m_vboPositions;
    GLuint m_vboNormals;
    GLuint m_vboTextureCoords;
    GLuint m_vboIndices;
 ///@brief GLuint addresses for vertex array objects that will contain any of the above buffer objects
    GLuint m_vao;
 ///@brief Pointer to vector of integers -> list of landmarked vertices on current mesh
    std::vector<int>* m_landmarkVertexIndices;
 ///@brief Integer variable representing the currently picked vertex index
    int m_pickedIndex;
///@brief Boolean value saying whether or not the mesh should be displayed in wireframe
    bool m_wireframe;
///@brief Boolean value showing whether the mesh was modified during a NRICP/ICP method
    bool m_modified;
///@brief Principal eigenvectors and eigenvalues
    Matrix3f m_eigenvectors;
    Vector3f m_eigenvalues;
///@brief Segmentation tree
    Segmentation* m_segmentationRoot;
///@brief Segments from tree
    std::vector< std::vector<GLuint>* > m_segments;
    std::vector<GLuint>* m_segmentsIndices;
///@brief Boolean saying if we are displaying segments or the whole mesh
    bool m_segmentationMode;

 public:
 ///@brief ctor for Mesh class
    Mesh();
 ///@brief dtor for Mesh class
    ~Mesh();
 ///@brief loads a mesh from the file
 ///@param [in] _fileName -> the name of the file where the mesh is stored
 ///@param [in] _transformations -> an array of transformations the the mesh will undego before being drawn
 ///@param [out] boolean value representing the success of the mesh loading operation
    bool loadMesh(const char* _fileName);
 ///@brief prints the landmarked vertex indices on the mesh to file
 ///@param [in] _fileName -> the name of the file where the indices will be printed
    void printLandmarkedPoints(const char* _fileName);
 ///@brief calculates the vertex normals using adjacent face normals and stores them in the m_normals vector
    void calculateNormals();
 ///@brief brings the mesh vertices in the [-1, 1]^3 space
    void normaliseMesh();
 ///@brief methods for binding the corresponding vertex array objects
    void bindVAOs();
 ///@brief deallocate buffer data memory
    void unbindVAOs();
 ///@brief creates the arc-node list, m_adjMat
    void buildArcNodeMatrix();
 ///@brief creates the vertex matrix, m_D
    void buildVertexMatrix();
 ///@brief creates the normal lines vector, m_vertNormalLines
    void buildVertexNormalVector();
 ///@brief creates the neighbour list
    void buildNeighbourList();
 ///@brief checks intersection of a ray with the mesh
 ///@param [in] _culling -> boolean variable indicating whether culling is neccessary when checking for intersection
 ///@param [in] _originTemplateIndex -> integer represting the template starting vertex index, if intersection is required from an origin on the template
 ///@param[in] _origin -> vector of size 3 representing the position of the origin of the ray
 ///@param [in] _ray -> vector of size 3 representing the intersection ray
 ///@param [out] Integer value representing the closest vertex to the intersection point in case of success or -1 in case the ray misses the mesh
    int whereIsIntersectingMesh(bool _culling, int _originTemplateIndex, Vector3f _origin, Vector3f _ray);
 ///@brief affine tranformation the mesh (rotation, translation, scale)
 ///@param [in] _X -> tranformation matrix
    void affineTransformation(MatrixXf _X);
///@brief adds the currently picked vertex to the list of landmarked points
    void addLandmarkVertexIndex()
    {
     m_landmarkVertexIndices->push_back(m_pickedIndex);
    }
///@brief clears landmarks from the list
    void clearLandmarkVertexIndices()
    {
     m_landmarkVertexIndices->clear();
    }
///@brief rotates mesh
///@param [in] _angleX, _angleY, _angleZ -> angles around X, Y and Z axes
    void rotateObject(float _angleX, float _angleY, float _angleZ);

    void rotateObject(Matrix3f _R);

///@brief moves mesh
/// @param [in] _tX, _tY, _tZ -> distances to be translated along X, Y and X axes
    void moveObject(float _tX, float _tY, float _tZ);
    void moveObject(Vector3f _trans);
///@brief Normalise normals to [-1,1]^3 interval
    void normaliseNormals();
///@brief Euclidean distance between two vertices
///@param [in] _v1, _v2 first and second vertices
    float euclideanDistance(Vector3f _v1, Vector3f _v2);
///@brief Calculates and returnes the Gaussian curvature of a vertex
///@param [in] _index  the index of the vertex whose curvature we are calculating
    float calculateVertexCurvature(int _index);
///@brief Calculates the average position of the mesh
    void calculatePosition();
///@brief find if neighbour2 is in the list of neighbours of neighbour1
    bool findInListOfNeighbours(int _neighbour1, int _neighbour2);
///@brief calculate principal eigenvectors
    void calculateEigenvectors();
///@brief Moves mesh to centre
    void moveToCentre();
///@brief Rotate using eigenvectors to principal axes
    void rotateByEigenVectors();
///@brief Checks if eigenvectors are perpendicular to each other
    bool areEigenvectorsOrthogonal();
///@brief Segmentation
    void segmentMesh();
    void splitSegmentIntoSubsegments(Segmentation* _segment, std::vector<GLuint>* _parentFaces, Vector3f _planeCentre, Vector3f _planeNormal);
    Segmentation* segmentationProcedure(Vector3i _plane, Vector3f _normal, Segmentation *_root, std::vector<GLuint> *_rootSideFaces);
    void createSegmentList();
    void destroySegments(Segmentation *_segmentation);
    Vector3f calculateCentre(int _p1, int _p2, int _p3);


/// Setters and Getters of the private members
    std::vector<GLfloat>* getVertices(){ return m_vertices; }
    std::vector<GLfloat>* getNormals(){ return m_normals; }
    unsigned int getVertCount() { return m_vertCount; }
    unsigned int getFaceCount() { return m_faceCount; }
    std::vector<GLuint>* getFaceIndices() { return m_faceIndices; }
    unsigned int getEdgeCount() { return m_edgeCount; }
    unsigned int getTexCoordCount() { return m_texCoordCount; }
    GLuint getVAO() { return m_vao; }
    void setVAO(GLuint _vao) { m_vao = _vao; }
    std::map <std::pair<unsigned int, unsigned int>, short >* getAdjMat(){ return m_adjMat; }
    SparseMatrix<float>* getD(){ return m_D; }
    float x() { return m_position[0]; }
    float y() { return m_position[1]; }
    float z() { return m_position[2]; }
    Vector3f getPosition() { return m_position; }
    Vector3f getNormal(unsigned int _vertNo);
    Vector3f getVertex(unsigned int _vertNo);
    void setVertex(unsigned int _vertNo, Vector3f _value);
    std::vector<int>* getLandmarkVertexIndices()
    {
     return m_landmarkVertexIndices;
    }

    int getPickedVertexIndex()
    {
     return m_pickedIndex;
    }

    void setPickedVertexIndex(int _index)
    {
     m_pickedIndex = _index;
    }

    void setWireframe(bool _value){ m_wireframe = _value; }
    bool isWireframe(){ return m_wireframe; }
    std::vector<int> getNeighbours(int _index) { return m_neighbours->at(_index); }
    void setModified(bool _modified) { m_modified = _modified; }
    bool isModified() { return m_modified; }

    void appendVertex(Vector3f _vertex)
    {
        //Might create size problems
        m_vertices->push_back(_vertex[0]);
        m_vertices->push_back(_vertex[1]);
        m_vertices->push_back(_vertex[2]);
        m_vertCount++;
    }

    void appendNormal(Vector3f _normal)
    {
        //To be accompanied by appendVertex
        m_normals->push_back(_normal[0]);
        m_normals->push_back(_normal[1]);
        m_normals->push_back(_normal[2]);
    }

    void appendFace(Vector3i _face)
    {
        m_faceIndices->push_back(_face[0]);
        m_faceIndices->push_back(_face[1]);
        m_faceIndices->push_back(_face[2]);
        m_faceCount++;
    }

    Matrix3f getEigenMatrix()
    {
        return m_eigenvectors;
    }

    bool isInSegmentationMode()
    {
        return m_segmentationMode;
    }

    void setSegmentationMode(bool _segmentationMode)
    {
        m_segmentationMode = _segmentationMode;
    }

    std::vector<GLuint>* getSegment(unsigned int _i)
    {
        if(_i < m_segments.size())
        {
            return m_segments[_i];
        }

        return NULL;
    }

    unsigned int getSegmentCount()
    {
        return m_segments.size();
    }

};
#endif // MESH_H
