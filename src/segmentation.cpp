#include "segmentation.h"


Segmentation::Segmentation(Mesh *_originalMesh)
{
    m_originalMesh = _originalMesh;
    m_threshold = -10;
    m_minVerts = 20;
    m_vertCount = m_originalMesh->getVertCount();
    m_regions = 0;
    m_activeSegment = 0;
    _originalMesh->buildNeighbourList();
    createVertexInfoList();
}

Segmentation::~Segmentation()
{
    for(unsigned int i = 0; i < m_vertCount; ++i)
    {
        delete m_vertInfos[i];
    }

    unsigned int size = m_segmentations.size();

    for(unsigned int j = 0; j < size; ++j)
    {
     delete m_segmentations[j];
    }
}


void Segmentation::segment()
{
    //Step 1: Compute Gaussian curvature for each vertex on the surface
    //Step 2: Label vertices of negative curvature as boundaries using threshold
    //        Label remaining verts as seeds


    for(unsigned int i = 0; i < m_vertCount; ++i)
    {
        float curvature = m_originalMesh->calculateVertexCurvature(i);
        m_vertInfos[i]->curvature = curvature;
        if (curvature > m_threshold)
        {
            m_vertInfos[i]->category = true; //seed
        }
        else
        {
            m_vertInfos[i]->visited = true;
        }
    }


    //Step 3: Eliminate isolated vertices
    //Observation: 2 neighbours marked as isolated
    eliminateIsolatedVertices();

    //Step 4: Region growing --> Depth First Search
    findRegions();


    //Step 5: Assign non-labeled vertices to parts and eliminate parts having too few vertices
    createSegments();
    createMeshes();
    bindVAOs();
}

void Segmentation::createVertexInfoList()
{
    for(unsigned int i=0; i < m_vertCount; ++i)
    {
        VertexInfo* auxVertInfo = new VertexInfo();
        auxVertInfo->index = i;
        auxVertInfo->category = false; //initialize as boundary
        auxVertInfo->curvature = -10000;
        auxVertInfo->region = -1;
        auxVertInfo->visited = false;
        m_vertInfos.push_back(auxVertInfo);
    }
}

void Segmentation::eliminateIsolatedVertices()
{
    for(unsigned int i = 0; i < m_vertCount; ++i)
    {
        std::vector<int> neighbours = m_originalMesh->getNeighbours(i);
        unsigned int size = neighbours.size();
        unsigned int noSameCurvature = 0;

        for(unsigned int j = 0; j < size; ++j)
        {
            if(m_vertInfos[i]->category != m_vertInfos[neighbours[j]]->category)
            {
                noSameCurvature++;
            }
        }

        if(noSameCurvature == size)
        {
           //We have an isolated vertex
            m_vertInfos[i]->category = !m_vertInfos[i]->category;
        }
    }
}

void Segmentation::findRegions()
{
  std::vector<int> unvisitedVertices;
  std::vector<int> stack;
  int index;
  m_regions = 0;

  for(unsigned int i = 0; i < m_vertCount; ++i)
  {
      if(m_vertInfos[i]->category)
      {
       unvisitedVertices.push_back(i);
      }
      else
      {
       m_boundaryVerts.push_back(i);
      }
  }

  unsigned int size = unvisitedVertices.size(); //only seed vertices

  while(size > 0)
  {
    bool regionFound = false;
    index = unvisitedVertices[0];
    stack.push_back(index);
    m_vertInfos[index]->region = m_regions;
    m_vertInfos[index]->visited = true;
    unvisitedVertices.erase(unvisitedVertices.begin());

    while(!regionFound)
    {
       index = firstUnvisitedNeighbour(stack.at(stack.size()-1));

       if(index < 0) //all neighbours are visited
       {
        stack.pop_back();
       }
       else
       {
           stack.push_back(index);
           m_vertInfos[index]->region = m_regions;
           m_vertInfos[index]->visited = true;

           size = unvisitedVertices.size();
           unsigned int j = 0;
           bool eliminated = false;
           while(j < size && !eliminated)
           {
            if(unvisitedVertices[j] == index)
            {
                unvisitedVertices.erase(unvisitedVertices.begin()+j);
                eliminated = true;
            }
            ++j;
           }
       }

       if(stack.size() < 1)
       {
           regionFound = true;
       }
    }

    m_regions++;

    size = unvisitedVertices.size();
  }
}


int Segmentation::firstUnvisitedNeighbour(unsigned int _index)
{
    std::vector<int> neighbours = m_originalMesh->getNeighbours(_index);
    unsigned int size = neighbours.size();
    unsigned int i = 0;
    bool found = false;
    int neighbour = -1;

    while(!found && i < size)
    {
      if(!m_vertInfos[neighbours[i]]->visited)
      {
          neighbour = neighbours[i];
          found = true;
      }
      ++i;
    }

    return neighbour;
}

int Segmentation::findClosestLabelledNeighbour(unsigned int _index)
{
    std::vector<int> neighbours = m_originalMesh->getNeighbours(_index);
    unsigned int size = neighbours.size();
    float minDistance = 1000;
    float distance = 0.0;
    int index = -1;
    int temp;
    Vector3f v1 = m_originalMesh->getVertex(_index);

    for(unsigned int i=0; i<size; ++i)
    {
        temp = neighbours.at(i);
        Vector3f v2 = m_originalMesh->getVertex(temp);

        distance = euclideanDistance(v1, v2);

        if(distance < minDistance && m_vertInfos.at(temp)->region >= 0)
        {
            index = temp;
            minDistance = distance;
        }
    }

    return index;
}

void Segmentation::createSegments()
{

    unsigned int i = 0;
    unsigned int size;

    //Obs! Regions might grow and corrupt the heap
     for (unsigned int i=0; i<m_regions; ++i)
     {
       m_segmentations.push_back(new std::vector<unsigned int>());
     }

    //Create initial segments from region algorithm results ---------------------
     for(unsigned int j=0; j < m_vertCount; ++j)
     {
      int region = m_vertInfos[j]->region;
      int index = m_vertInfos[j]->index;
      if(region >= 0)
      {
       m_segmentations[region]->push_back(index);
      }
     }

     //Add boundries to segments ------------------------------------------------
     size = m_boundaryVerts.size();
     std::vector<unsigned int> boundaryVerts;

     for(unsigned int i=0; i<size; ++i)
     {
       boundaryVerts.push_back(m_boundaryVerts.at(i));
     }

     i = 0;
     while(size > 0 && i < size)
     {
        int index = findClosestLabelledNeighbour(boundaryVerts[i]);

        if(index >= 0)
        {
         int region = m_vertInfos[index]->region;
         m_vertInfos[boundaryVerts[i]]->region = region;
         m_segmentations[region]->push_back(boundaryVerts[i]);
         boundaryVerts.erase(boundaryVerts.begin() + i);
         size = boundaryVerts.size();
         i = 0;
        }
        else
        {
         i++;
        }
     }

     i = 0;
     //Eliminate regions with too few vertices -------------------------------------
     std::vector<unsigned int> reallocateVerts;
     while (i < m_regions)
     {
         std::vector<unsigned int>* segment = m_segmentations.at(i);
         unsigned int size_segs = segment->size();
         if(size_segs < m_minVerts)
         {
             //Reallocate vertices
             for(unsigned int j=0; j<size_segs; ++j)
             {
              unsigned int vertIndex = segment->at(j);
              m_vertInfos[vertIndex]->region = -1;
              reallocateVerts.push_back(vertIndex);
             }
         }
         i++;
     }

     i=0;     
     while(i < m_segmentations.size())
     {
         if(m_segmentations.at(i)->size() < m_minVerts)
         {
             m_segmentations.erase(m_segmentations.begin() + i);
             i=0;
         }
         else
         {
          i++;
         }
     }
     m_regions = m_segmentations.size();

     //Rename regions
     for(unsigned int i=0; i<m_regions; ++i)
     {
       std::vector<unsigned int>* segment = m_segmentations.at(i);
       size = segment->size();
       for(unsigned int j=0; j<size; ++j)
       {
         m_vertInfos.at(segment->at(j))->region = i;
       }
     }


     size = reallocateVerts.size();
     i = 0;
     while(size > 0 && i < size)
     {
         int index = findClosestLabelledNeighbour(reallocateVerts[i]);

         if(index >= 0)
         {
          int region = m_vertInfos[index]->region;
          m_vertInfos[reallocateVerts[i]]->region = region;
          m_segmentations[region]->push_back(reallocateVerts[i]);
          reallocateVerts.erase(reallocateVerts.begin() + i);
          size = reallocateVerts.size();
          i = 0;
         }
         else
         {
          i++;
         }
     }
}

 void Segmentation::createMeshes()
 {
     unsigned int size_segs = m_segmentations.size();

     for(unsigned int i=0; i<size_segs; ++i)
     {
         m_subMeshes.push_back(new Mesh());

         //Vertices and Normals
         addVertexNormalInformation(i);
     }


   //Face indices
    std::vector<GLuint>* faces = m_originalMesh->getFaceIndices();
    unsigned int size_faces = m_originalMesh->getFaceCount();
    unsigned int three_i;
    unsigned int v1, v2, v3;

    for (unsigned int i=0; i<size_faces; ++i)
    {
        three_i = 3*i;
        v1 = faces->at(three_i);
        v2 = faces->at(three_i + 1);
        v3 = faces->at(three_i + 2);

        unsigned int j = 0;
        Vector3i newIndices(-1, -1, -1);
        bool found = false;
        while(!found && j < size_segs)
        {
         newIndices = findFaceInSegment(v1, v2, v3, j);

         if(newIndices[0] >= 0 && newIndices[1] >= 0 && newIndices[2] >= 0)
         {
             //Found face in segment - indices are segment based
             found = true;
         }
         ++j;
        }
        if(found) //we found face v1, v2, v3 in j-1th segment
        {
            m_subMeshes.at(j-1)->appendFace(newIndices);
        }
    }
 }

Vector3i Segmentation::findFaceInSegment(unsigned int _v1, unsigned int _v2, unsigned int _v3, unsigned int _j)
{
    std::vector<unsigned int>* segment = m_segmentations[_j];
    Vector3i newIndices (-1, -1, -1);

    int items_found = 0;
    unsigned int i = 0;
    unsigned int size = segment->size();

    while(items_found < 3 && i<size)
    {
        unsigned int element = segment->at(i);
        if(element == _v1)
        {
            newIndices[0] = i;
            items_found++;
        }

        else if( element == _v2)
        {
            newIndices[1] = i;
            items_found++;
        }

        else if (element == _v3)
        {
            newIndices[2] = i;
            items_found++;
        }

        ++i;
    }

    return newIndices;
}


void Segmentation::addVertexNormalInformation(unsigned int _index)
{
  std::vector<unsigned int>* segment = m_segmentations.at(_index);
  Mesh* submesh = m_subMeshes.at(_index);
  unsigned int size = segment->size();

  for(unsigned int i=0; i<size; ++i)
  {
   unsigned int vertIndex = segment->at(i);
   Vector3f vertex = m_originalMesh->getVertex(vertIndex);
   Vector3f normal = m_originalMesh->getNormal(vertIndex);

   submesh->appendVertex(vertex);
   submesh->appendNormal(normal);
  }
}

void Segmentation::bindVAOs()
{
    unsigned int size_meshes = m_subMeshes.size();

    for(unsigned int i=0; i<size_meshes; ++i)
    {
        m_subMeshes.at(i)->bindVAO1();
    }
}


void Segmentation::calculateActiveSegment(unsigned int _vertexIndex)
{
  Vector3f v1 = m_originalMesh->getVertex(_vertexIndex);

  float minDistance = 1000.0;
  float distance;
  unsigned int minSegmentIndex = 0;
  unsigned int size = m_segmentations.size();
  unsigned int i = 0;

  while (i<size)
  {
      m_subMeshes.at(i)->calculatePosition();
      Vector3f segmentPosition = m_subMeshes.at(i)->getPosition();

      distance = euclideanDistance(v1, segmentPosition);

      if(distance < minDistance)
      {
          minDistance = distance;
          minSegmentIndex = i;
      }
      i++;
  }

   m_activeSegment = minSegmentIndex;
}

void Segmentation::updateMeshFromSegments()
{
  //TO DO: Soften edges!
  unsigned int noMeshes = m_subMeshes.size();

  for(unsigned int i = 0; i < noMeshes; ++i)
  {
      if(m_subMeshes.at(i)->isModified())
      {
         Mesh* mesh_aux = m_subMeshes.at(i);
         std::vector<unsigned int>* segment = m_segmentations.at(i);
         unsigned int vertCount = segment->size(); //might differ from mesh size

         for(unsigned int j=0; j<vertCount; ++j)
         {
            Vector3f changedVertex = mesh_aux->getVertex(j);
            m_originalMesh->setVertex(segment->at(j), changedVertex);
         }

         m_subMeshes.at(i)->setModified(false);
      }
  }
}


void Segmentation::updateSegmentsFromMesh()
{
    if(m_originalMesh->isModified())
    {
       unsigned int noMeshes = m_subMeshes.size();

       for(unsigned int i = 0; i < noMeshes; ++i)
       {
         Mesh* mesh_aux = m_subMeshes.at(i);
         std::vector<unsigned int>* segment = m_segmentations.at(i);
         unsigned int size = segment->size();

         for(unsigned int j = 0; j < size; ++j)
         {
             Vector3f changedVertex = m_originalMesh->getVertex(segment->at(j));
             mesh_aux->setVertex(j,changedVertex);
         }
       }

       m_originalMesh->setModified(false);
    }
}
