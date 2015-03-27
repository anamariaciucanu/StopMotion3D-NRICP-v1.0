#include "segmentation.h"


Segmentation::Segmentation(Mesh *_originalMesh)
{
    m_originalMesh = _originalMesh;
    m_threshold = -1;
    m_vertCount = m_originalMesh->getVertCount();
    m_regions = 0;
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

    // ofstream myfile;
    // myfile.open("../logs/K.txt");
    for(unsigned int i = 0; i < m_vertCount; ++i)
    {
        float curvature = m_originalMesh->calculateVertexCurvature(i);
        m_vertInfos[i]->curvature = curvature;
        if (curvature > m_threshold)
        {
            m_vertInfos[i]->category = true;
        }
        else
        {
            m_vertInfos[i]->visited = true;
        }
      //  myfile << m_curvatures->at(i) << "\n";
    }

     //myfile.close();

    //Step 3: Eliminate isolated vertices
    //Observation: 2 neighbours marked as isolated
    eliminateIsolatedVertices();

    //Step 4: Region growing --> Depth First Search
    findRegions();


    //Step 5: Assign non-labeled vertices to parts and eliminate parts having too few vertices
    createSegments();
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

  for(unsigned int i = 0; i < m_vertCount; ++i)
  {
      if(m_vertInfos[i]->category)
      {
       unvisitedVertices.push_back(i);
      }
  }

  unsigned int size = unvisitedVertices.size(); //only seed vertices

  while(size > 0)
  {
    bool regionFound = false;
    index = unvisitedVertices[0];
    stack.push_back(index);
    int vertsInRegion = 1;

    if(m_vertInfos[index]->category)
    {
       m_vertInfos[index]->region = m_regions;
    }
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
           vertsInRegion++;

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

    if (vertsInRegion > 1) //non-labeled are labeled to previous segment...hmmmmmm
    {
      m_regions++;
    }

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


void Segmentation::createSegments()
{
     for (unsigned int i=0; i<m_regions; ++i)
     {
       m_segmentations.push_back(new std::vector<int>());
     }

     for(unsigned int j=0; j < m_vertCount; ++j)
     {
      int region = m_vertInfos[j]->region;
      int index = m_vertInfos[j]->index;
      if(region >= 0)
      {
       m_segmentations[region]->push_back(index);
      }
     }

     //eliminiate regions with too few elements
     unsigned int k = 0;
     std::vector<int> toEliminate;
     while(k < m_regions)
     {
         unsigned int size = m_segmentations[k]->size();
         if(size <= 5)
         {
            for(unsigned int l = 0; l < size; ++l)
            {
              m_vertInfos[m_segmentations[k]->at(l)]->region = -1;
            }
            toEliminate.push_back(k);
         }
         k++;
     }

     unsigned int size = toEliminate.size();
     for(unsigned int k = 0; k < size; ++k)
     {
         m_segmentations.erase(m_segmentations.begin() + toEliminate[k]-k);
     }

     m_regions -= toEliminate.size();
}



