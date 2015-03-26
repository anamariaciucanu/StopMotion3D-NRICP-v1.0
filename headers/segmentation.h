#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <vector>
#include "mesh.h"

class Segmentation
{
private:
  Mesh* m_originalMesh;
  std::vector<Mesh*> m_subMeshes;
  std::vector<float>* m_curvatures;
  std::vector<std::vector<int> >* m_segmentations;
  int m_threshold;

public:
   Segmentation(Mesh* _originalMesh);
   ~Segmentation();

   void segment();
};

#endif // SEGMENTATION_H
