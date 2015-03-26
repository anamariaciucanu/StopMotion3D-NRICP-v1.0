#include "segmentation.h"


Segmentation::Segmentation(Mesh *_originalMesh)
{
    m_originalMesh = _originalMesh;
    m_threshold = -1;
    _originalMesh->buildNeighbourList();
    m_curvatures = new std::vector<float>( m_originalMesh->getVertCount());
}

void Segmentation::segment()
{
    //Step 1: Compute Gaussian curvature for each vertex on the surface
    int noVertices = m_originalMesh->getVertCount();

    for(unsigned int i=0; i<noVertices; ++i)
    {
        m_curvatures->at(i) = m_originalMesh->calculateVertexCurvature(i);
    }

    //Step 2: Label vertices of negative curvature as boundaries using threshold
    //        Label remaining verts as seeds
    //Step 3: Eliminate isolated vertices
    //Step 4: Region growing
    //Step 5: Assign non-labeled vertices to parts and eliminate parts having too few vertices
}

Segmentation::~Segmentation(){}
