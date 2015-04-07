#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <vector>
#include "mesh.h"

using namespace Eigen;


struct VertexInfo
{
 int index;
 bool category; //seed or boundary
 int region; //has region
 float curvature;
 bool visited; //visited
};

class Segmentation
{
private:
  Mesh* m_originalMesh;
  std::vector<Mesh*> m_subMeshes;
  std::vector<VertexInfo*> m_vertInfos;
  std::vector<std::vector<unsigned int>*> m_segmentations;
  std::vector<unsigned int> m_boundaryVerts;
  float m_threshold;
  unsigned int m_minVerts;
  unsigned int m_vertCount;
  unsigned int m_regions;
  unsigned int m_activeSegment;

public:
   Segmentation(Mesh* _originalMesh);
   ~Segmentation();

   void segment();
   void createVertexInfoList();
   void eliminateIsolatedVertices();
   void findRegions();
   int firstUnvisitedNeighbour(unsigned int _index);
   Vector3i findFaceInSegment(unsigned int _v1, unsigned int _v2, unsigned int _v3, unsigned int _j);
   int findClosestLabelledNeighbour(unsigned int _index);
   void createSegments();
   void createMeshes();
   void addVertexNormalInformation(unsigned int _index);
   void bindVAOs();
   void calculateActiveSegment(unsigned int _vertexIndex);
   void updateMeshFromSegments();
   void updateSegmentsFromMesh();

   ///Setters and getters
   unsigned int getNumberOfSegments()
   {
       return m_segmentations.size();
   }
   Mesh* getMesh(unsigned int _index)
   {
     return m_subMeshes.at(_index);
   }

   float euclideanDistance(Vector3f _v1, Vector3f _v2)
   {
    float diff1 = _v1[0] - _v2[0];
    float diff2 = _v1[1] - _v2[1];
    float diff3 = _v1[2] - _v2[2];

    return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
   }

   void setActiveSegment(unsigned int _activeSegment)
   {
       m_activeSegment = _activeSegment;
   }

   unsigned int getActiveSegment()
   {
       return m_activeSegment;
   }

   Mesh* getActiveMesh()
   {
       return m_subMeshes.at(m_activeSegment);
   }
};

#endif // SEGMENTATION_H
