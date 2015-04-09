#ifndef LINKER_H
#define LINKER_H
#include "segmentation.h"

class Linker
{
  private:
    Mesh* m_mesh;
    Segmentation* m_segmentation;


  public:
    Linker(Mesh* _mesh, Segmentation* _segmentation);
    ~Linker();

    void updateChanges();
    void addedLandmark();
    void clearedLandmarks();

    void setMesh(Mesh* _mesh) { m_mesh = _mesh; }
    Mesh* getMesh() { return m_mesh; }
    void setSegmentation(Segmentation* _segmentation) { m_segmentation = _segmentation; }
    Segmentation* getSegmentation() { return m_segmentation; }
};
#endif // LINKER_H
