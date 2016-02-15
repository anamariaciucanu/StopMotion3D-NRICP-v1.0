//OBS: Assuming triangular faces and 3 coordinates per vertex
//TO DO: m_vertNormalLines doesn't work as desired -> flat lines on mesh instead of normals?
//TO DO: add vt/vn index info from face indices section in obj file
//TO DO: use m_texcoords and m_texCoordCount for texturing
//TO DO: use either Vector3f, Matrix4f or vec3, mat4 etc. (Eigen library versus matrix.h)
//TO DO: affineTransformation input matrix _X should be 4x4 or 4x3 or 3x4?


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
#include <sstream>
#include <fstream>
#include <string>
#include <set>
#include "KDTreePartition.h"

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
 ///@brief Pointer to vector of texture coordinates GLfloats
 ///Every group of 2 floats represent the 2D coordinates on the texture image, assigned to the respective vertex
 ///This array will be sent to the shader to draw textures on meshes
    std::vector<GLfloat>* m_texcoords;
 ///@brief Pointer to vector of face indices GLunit
 ///Every group of 3 indices represents a triangle from the mesh
    std::vector<GLuint>* m_faceIndices;
 ///@brief KDTree partition
    KDTreeNode* m_partitionRoot;
 ///@brief Vector of size 3, representing the mesh's center of gravity
    Vector3f m_centre;
 ///@brief GLuinteger variable -> the number of edges
    GLuint m_edgeCount;
 ///@brief Unsigned integer variable -> the number of vertices
    GLuint m_vertCount;
 ///@brief Unsigned integer variable -> the number of faces
    GLuint m_faceCount;
 ///@brief Unsigned integer variable -> the number of texture coordinate pairs?
    GLuint m_texCoordCount;
 ///@brief Pointer to a set of pairs of unsigned integers -> a list of directed edges (smaller index -> larger index)
 ///corresponds to the arc-node matrix
    std::set <std::pair<GLuint, GLuint> >* m_adjMat;
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
///@brief Principal eigenvectors and eigenvalues
    Matrix3f m_eigenvectors;
    Vector3f m_eigenvalues;
///@brief Segmentation tree
    Segmentation* m_segmentationRoot;
///@brief Segments from tree
    std::vector< std::vector<GLuint>* > m_segments;
///@brief Segmentation planes
    std::vector<Vector3i> m_segmentationPlanes;
///@brief Used for drawing order in VAO
    std::vector<GLuint>* m_segmentsIndices;
///@brief int indicating the active segment to be used and displayed
    int m_activeSegment;
    int m_activeSegmentationPlane;
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
 ///@brief brings the mesh vertices in the [-1, 1]^3 space
    void normaliseMesh();
 ///@brief calculates the vertex normals using adjacent face normals and stores them in the m_normals vector
    void calculateNormals();
 ///@brief Normalise normals to [-1,1]^3 interval
    void normaliseNormals();
 ///@brief Partitions the mesh for optimal storage (In our case, KDTrees are used)
    void partitionMesh();
 ///@brief Recursive prodedure for KDTrees
    void partitionMeshIteration(KDTreeNode* _root, std::vector<int> _pointList, int _depth);
 ///@brief destroy mesh partitions
    void destroyMeshPartitions(KDTreeNode* _root);
 ///@brief Finds closest point from the mesh
 /// [in] _point - Point in space to which we are trying to find the closest distance
 /// [out] int - index of the vertex from the mesh that was found closest to _point
 int findClosestPoint(Vector3f _point);
 void findClosestPointIteration1(Vector3f _query, KDTreeNode* _root, int &_refPoint, float &_refDistance);
 ///@brief methods for binding the corresponding vertex array objects
    void bindVAOs();
 ///@brief deallocate buffer data memory
    void unbindVAOs();
 ///@brief creates the arc-node list, m_adjMat
    void buildArcNodeMatrix();
 ///@brief creates the vertex matrix, m_D
    void buildVertexMatrix();
 ///@brief creates the neighbour list
    void buildNeighbourList();
 ///@brief find if neighbour2 is in the list of neighbours of neighbour1
    bool findInListOfNeighbours(int _neighbour1, int _neighbour2);
 ///@brief checks intersection of a ray with the mesh
 ///@param [in] _culling -> boolean variable indicating whether culling is neccessary when checking for intersection
 ///@param [in] _originTemplateIndex -> integer represting the template starting vertex index, if intersection is required from an origin on the template
 ///@param[in] _origin -> vector of size 3 representing the position of the origin of the ray
 ///@param [in] _ray -> vector of size 3 representing the intersection ray
 ///@param [out] Integer value representing the closest vertex to the intersection point in case of success or -1 in case the ray misses the mesh
    int whereIsIntersectingMesh(bool _culling, int _originTemplateIndex, Vector3f _origin, Vector3f _ray);
 ///@brief Calculates and returnes the Gaussian curvature of a vertex
 ///@param [in] _index  the index of the vertex whose curvature we are calculating
    GLfloat calculateVertexCurvature(int _index);
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
     void rotateObject(GLfloat _angleX, GLfloat _angleY, GLfloat _angleZ);
     void rotateObject(Matrix3f _R);
///@brief moves mesh
///@param [in] _tX, _tY, _tZ -> distances to be translated along X, Y and X axes
     void moveObject(GLfloat _tX, GLfloat _tY, GLfloat _tZ);
     void moveObject(Vector3f _trans);
///@brief Calculates the centre of gravity of the mesh
    void calculateCentre();
///@brief Moves mesh to centre
    void moveToCentre();
///@brief calculate principal eigenvectors
    void calculateEigenvectors();
///@brief Rotate using eigenvectors to principal axes
    void rotateByEigenVectors();
///@brief Checks if eigenvectors are perpendicular to each other
    bool areEigenvectorsOrthogonal();
///@brief Segmentation
    void segmentMesh();
    void splitSegmentIntoSubsegments(Segmentation* _segment, std::vector<GLuint>* _parentFaces, Vector3f _planeCentre, Vector3f _planeNormal);
    Segmentation* segmentationProcedure(Vector3i _plane, Vector3f _normal, Segmentation *_root, std::vector<GLuint> *_parentSideFaces);
    void createSegmentList();
    void destroySegments(Segmentation *_segmentation);
    Vector3f calculateTriangleCentre(int _p1, int _p2, int _p3);
    Vector3f calculateActiveSegmentationPlaneCentre();
    Vector3f calculateActiveSegmentationPlaneNormal();

///@brief prints the landmarked vertex indices on the mesh to file
///@param [in] _fileName -> the name of the file where the indices will be printed
    void printLandmarkedPoints(const char* _fileName);
///@brief Euclidean distance between two vertices
///@param [in] _v1, _v2 first and second vertices
    GLfloat euclideanDistance(Vector3f _v1, Vector3f _v2);

///@brief Setters and Getters of the private members -----------------
    std::vector<GLfloat>* getVertices(){ return m_vertices; }
    std::vector<GLfloat>* getNormals(){ return m_normals; }
    GLuint getVertCount() { return m_vertCount; }
    GLuint getFaceCount() { return m_faceCount; }
    std::vector<GLuint>* getFaceIndices() { return m_faceIndices; }
    GLuint getEdgeCount() { return m_edgeCount; }
    GLuint getTexCoordCount() { return m_texCoordCount; }
    GLuint getVAO() { return m_vao; }
    void setVAO(GLuint _vao) { m_vao = _vao; }
    std::set <std::pair<GLuint, GLuint> >* getAdjMat(){ return m_adjMat; }
    SparseMatrix<GLfloat>* getD(){ return m_D; }
    GLfloat x() { return m_centre[0]; }
    GLfloat y() { return m_centre[1]; }
    GLfloat z() { return m_centre[2]; }
    Vector3f getCentre() { return m_centre; }
    Vector3f getNormal(GLuint _vertNo);
    Vector3f getVertex(GLuint _vertNo);
    void setVertex(GLuint _vertNo, Vector3f _value);
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

    std::vector<GLuint>* getSegment(GLuint _i)
    {
        if(_i < m_segments.size())
        {
            return m_segments[_i];
        }

        return NULL;
    }

    GLuint getSegmentCount()
    {
        return m_segments.size();
    }

    bool isSegmentationMode()
    {
      return m_segmentationMode;
    }

    void setSegmentationMode(bool _segmentationMode)
    {
      m_segmentationMode = _segmentationMode;
    }

    int getActiveSegmentNumber()
    {
        return m_activeSegment;
    }

    void setActiveSegmentNumber(int _value)
    {
        m_activeSegment = _value;
    }


    std::vector<GLuint>* getActiveSegment()
    {
        if(m_activeSegment >= 0)
        {
         return m_segments.at(m_activeSegment);
        }
        return NULL;
    }

    int getActiveSegmentationPlaneNumber()
    {
      return m_activeSegmentationPlane;
    }

    void setActiveSegmentationPlaneNumber(int _value)
    {
        m_activeSegmentationPlane = _value;
    }

    Vector3i getActivePlane()
    {
        if(m_activeSegmentationPlane >= 0)
        {
         return m_segmentationPlanes.at(m_activeSegmentationPlane);
        }

        return Vector3i(-1, -1, -1);
    }

    void setNextActiveSegment()
    {
        m_activeSegment = (m_activeSegment + 1) % m_segments.size();
    }


};
#endif // MESH_H
