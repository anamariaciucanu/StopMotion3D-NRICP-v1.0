#include "linker.h"

Linker::Linker(Mesh *_mesh, Segmentation *_segmentation)
{
    m_mesh = _mesh;
    m_segmentation = _segmentation;
}

Linker::~Linker(){}

void Linker::updateChanges()
{
  if(m_mesh->isModified())
  {
   //ICP/NRICP was made on this mesh so the segments need to be updated

      if(m_segmentation)
      {
         m_segmentation->updateSegmentsFromMesh();
      }
  }
  else
  {
     //Segments are modified and the mesh needs to be updated
     m_segmentation->updateMeshFromSegments();
  }
}



