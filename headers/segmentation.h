#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <vector>
#include "mesh.h"

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
  std::vector<std::vector<int>*> m_segmentations;
  float m_threshold;
  unsigned int m_vertCount;
  unsigned int m_regions;

public:
   Segmentation(Mesh* _originalMesh);
   ~Segmentation();

   void segment();
   void createVertexInfoList();
   void eliminateIsolatedVertices();
   void findRegions();
   int firstUnvisitedNeighbour(unsigned int _index);
   void createSegments();
   void createMeshes();
};

#endif // SEGMENTATION_H
