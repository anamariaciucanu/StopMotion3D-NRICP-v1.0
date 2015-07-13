#include "NRICP_Segment.h"
#include <OrderingMethods>
#include <set>

//TO DO: m_landmarkCorrespChanged not used anywhere


NRICP_Segment::NRICP_Segment(Mesh* _template,  Mesh* _target)
{
    m_template = _template;
    m_target = _target;
    m_beta = 1.0;
    m_epsilon = 4.0;
    m_gamma = 1.0;

    //m_targetPartition = NULL;
    m_W = new VectorXi();
    m_U = new MatrixXf();
    m_X = new MatrixXf();
    m_A = new SparseMatrix<GLfloat>();
    m_B = new SparseMatrix<GLfloat>();
    m_templateSegmentVertIndices = new std::vector<GLuint>();
    m_targetSegmentVertIndices = new std::vector<GLuint>();
    m_templateLandmarks = new std::vector<GLuint>();
    m_targetLandmarks = new std::vector<GLuint>();
    m_D = NULL;
}

NRICP_Segment::~NRICP_Segment()
{
    m_W->resize(0, 0);
    m_U->resize(0, 0);
    m_X->resize(0, 0);
    m_A->resize(0, 0);
    m_B->resize(0, 0);
    m_D->resize(0, 0);

    if (m_templateSegmentVertIndices) { delete [] m_templateSegmentVertIndices; }
    if (m_targetSegmentVertIndices) { delete [] m_targetSegmentVertIndices; }
    if (m_templateLandmarks) { delete [] m_templateLandmarks; }
    if (m_targetLandmarks) { delete []  m_targetLandmarks; }

    m_adjMat->clear();

    destroyPartitions(m_targetPartition);
}


void NRICP_Segment::initializeNRICP()
{
    m_templateSegmentFaces = m_template->getActiveSegment();
    m_targetSegmentFaces = m_target->getActiveSegment();

    buildVertexIndexArrays();
    buildArcNodeMatrix();
    buildVertexMatrix();
    initializeWUXVectors();
    buildLandmarkArrays();

    m_stiffness = 100.0;
    m_stiffnessChanged = true;
    m_nricpStarted = false;

    destroyPartitions(m_targetPartition);
    m_targetPartition = createPartitions(0, m_targetSegmentVertCount-1, m_targetPartition);

    m_A->resize(4 * m_templateSegmentEdgeCount, 4 * m_templateSegmentVertCount);
    m_B->resize(4 * m_templateSegmentEdgeCount, 3);
}


void NRICP_Segment::buildVertexIndexArrays()
{
    //Template segment ===========================================
    std::set<GLuint> templateSegmentVertIndices;
    std::set<GLuint> targetSegmentVertIndices;
    std::set<GLuint>::iterator it1, it2;

    unsigned int size = m_templateSegmentFaces->size();
    for(unsigned int i=0; i<size; ++i)
    {        
        templateSegmentVertIndices.insert(m_templateSegmentFaces->at(i));
    }

    //Target segment ==============================================
    size = m_targetSegmentFaces->size();
    for(unsigned int i=0; i<size; ++i)
    {
        targetSegmentVertIndices.insert(m_targetSegmentFaces->at(i));
    }   


    for(it1 = templateSegmentVertIndices.begin(); it1 != templateSegmentVertIndices.end(); ++it1)
    {
        m_templateSegmentVertIndices->push_back(*it1);
    }

    for(it2 = targetSegmentVertIndices.begin(); it2 != targetSegmentVertIndices.end(); ++it2)
    {
        m_targetSegmentVertIndices->push_back(*it2);
    }

    m_templateSegmentVertCount = m_templateSegmentVertIndices->size();
    m_targetSegmentVertCount = m_targetSegmentVertIndices->size();
}


void NRICP_Segment::buildArcNodeMatrix()
{
  //Create the arc-node adjacency matrix

    if(!m_adjMat)
    {
     m_adjMat = new std::set < std::pair<unsigned int, unsigned int> >();
     std::set < std::pair<unsigned int, unsigned int> >::iterator it;
     unsigned int three_i;
     unsigned int v1, v2, v3;
     unsigned int min, max;
     unsigned int faceCountTemplate = m_templateSegmentFaces->size()/3;

     //Mesh vert indices turned into segment indices
     //TO DO: Need to put in segment indices


     for (unsigned int i = 0; i < faceCountTemplate ; ++i)
     {
        three_i = 3*i;
        v1 = findValue(m_templateSegmentFaces->at(three_i), m_templateSegmentVertIndices);
        v2 = findValue(m_templateSegmentFaces->at(three_i + 1), m_templateSegmentVertIndices);
        v3 = findValue(m_templateSegmentFaces->at(three_i + 2), m_templateSegmentVertIndices);

        min = v1 < v2? v1 : v2;
        max = v1 > v2? v1 : v2;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(min, max));
            m_templateSegmentEdgeCount++;
        }

        min = v2 < v3? v2 : v3;
        max = v2 > v3? v2 : v3;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(min, max));
            m_templateSegmentEdgeCount++;
        }

        min = v1 < v3 ? v1 : v3;
        max = v1 > v3 ? v1 : v3;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(min, max));
            m_templateSegmentEdgeCount++;
        }
     }
   }
}


void NRICP_Segment::buildVertexMatrix()
{
  if(!m_D)
  {
     m_D = new SparseMatrix<float>(m_templateSegmentVertCount , 4 * m_templateSegmentVertCount);
     m_D->reserve(m_templateSegmentVertCount * 4);
  }

  unsigned int three_i = 0;
  unsigned int four_i = 0;
  std::vector<GLfloat>* templateVerts = m_template->getVertices();

  for(unsigned int i = 0; i < m_templateSegmentVertCount; ++i)
  {
   three_i = 3 * m_templateSegmentVertIndices->at(i);
   four_i = 4 * i;
   m_D->insert(i, four_i) = templateVerts->at(three_i);
   m_D->insert(i, four_i + 1) =  templateVerts->at(three_i + 1);
   m_D->insert(i, four_i + 2) = templateVerts->at(three_i + 2);
   m_D->insert(i, four_i + 3) = 1;   
  }

  m_D->makeCompressed();
}


void NRICP_Segment::initializeWUXVectors()
{
    m_W ->resize(m_templateSegmentVertCount);
    m_W->setOnes(m_templateSegmentVertCount);

    m_U->resize(m_templateSegmentVertCount, 3);
    m_U->setZero(m_templateSegmentVertCount, 3);

    m_X->resize(4 * m_templateSegmentVertCount, 3);
    m_X->setZero(4 * m_templateSegmentVertCount, 3);
}

void NRICP_Segment::buildLandmarkArrays()
{
  std::vector<int>* templateLandmarks = m_template->getLandmarkVertexIndices();
  std::vector<int>* targetLandmarks = m_target->getLandmarkVertexIndices();

  int sizeTemplateLandmarks = (templateLandmarks) ? templateLandmarks->size() : 0;
  int sizeTargetLandmarks = (targetLandmarks) ? targetLandmarks->size() : 0;

  m_templateLandmarks->clear();
  m_targetLandmarks->clear();

  for(int i=0; i<sizeTemplateLandmarks; ++i)
  {
    if(findValue(templateLandmarks->at(i), m_templateSegmentVertIndices) >= 0)
    {
        m_templateLandmarks->push_back(templateLandmarks->at(i));
    }
  }

  for(int j=0; j<sizeTargetLandmarks; ++j)
  {
      if(findValue(targetLandmarks->at(j), m_targetSegmentVertIndices) >= 0)
      {
          m_targetLandmarks->push_back(targetLandmarks->at(j));
      }
  }
}

SpherePartition* NRICP_Segment::createPartitions(unsigned int _start, unsigned int _end, SpherePartition* _partition)
{
    unsigned int three_i;

    //Observation: This assumes vertex data is in order
    _partition = new SpherePartition();
    _partition->start = _start;
    _partition->end = _end;
    _partition->average[0] = 0.0;
    _partition->average[1] = 0.0;
    _partition->average[2] = 0.0;

    std::vector<GLfloat>* targetVerts = m_target->getVertices();

    for(unsigned int i=_start; i<=_end; ++i)
    {
      three_i = 3 * m_targetSegmentVertIndices->at(i);
      _partition->average[0] += targetVerts->at(three_i);
      _partition->average[1] += targetVerts->at(three_i + 1);
      _partition->average[2] += targetVerts->at(three_i + 2);
    }

     unsigned int noElem = _end - _start + 1;

    _partition->average[0] /= noElem;
    _partition->average[1] /= noElem;
    _partition->average[2] /= noElem;

    if(noElem > 3)
    {
     _partition->left = createPartitions(_start, _start+noElem/2, _partition->left);
     _partition->right = createPartitions(_start+noElem/2, _end, _partition->right);
    }

    return _partition;
}


void NRICP_Segment::destroyPartitions(SpherePartition* _partition)
{
    if(_partition)
    {
        if(!_partition->left && !_partition->right)
        {
            delete _partition;
        }

        if (_partition->right)
        {
           destroyPartitions(_partition->right);
        }

        if (_partition->left)
        {
           destroyPartitions(_partition->left);
        }
    }
}



//Nonrigid transformations

void NRICP_Segment::calculateNonRigidTransformation()
{
   m_nricpStarted = true;

   float previous_seconds = glfwGetTime();

   MatrixXf* X_prev = new MatrixXf(4 * m_templateSegmentVertCount, 3);
   X_prev->setZero(4 * m_templateSegmentVertCount, 3);
   m_stiffnessChanged = true;

   findCorrespondences();
   determineNonRigidOptimalDeformation();
   deformTemplate();
   float changes = normedDifference(X_prev, m_X);

   while (changes > m_epsilon)
   {
    (*X_prev) = (*m_X);
    findCorrespondences();
    determineNonRigidOptimalDeformation();
    deformTemplate();
    changes = normedDifference(X_prev, m_X);
   }

   delete X_prev;

   float current_seconds = glfwGetTime();
   float elapsed_seconds = current_seconds - previous_seconds;
   printf(" NRICP takes %f seconds \n ", elapsed_seconds);
}


void NRICP_Segment::findCorrespondences()
{
      Vector3f templateVertex;
      float distance_left = 0.0;
      float distance_right = 0.0;
      m_W->setOnes();
      m_U->setZero();

      unsigned int three_i;
      std::vector<GLfloat>* templateVerts = m_template->getVertices();

      for (unsigned int i=0; i<m_templateSegmentVertCount; ++i)
      {
          three_i = 3 * m_templateSegmentVertIndices->at(i);
          templateVertex[0] = templateVerts->at(three_i);
          templateVertex[1] = templateVerts->at(three_i + 1);
          templateVertex[2] = templateVerts->at(three_i + 2);
          SpherePartition* previous = NULL;
          SpherePartition* aux = NULL;
          SpherePartition* current = m_targetPartition;

          while(current)
          {
              Vector3f targetSphere_left(1000.0, 1000.0, 1000.0);
              Vector3f targetSphere_right(1000.0, 1000.0, 1000.0);

              if(current->left)
              {
                targetSphere_left[0] = current->left->average[0];
                targetSphere_left[1] = current->left->average[1];
                targetSphere_left[2] = current->left->average[2];
              }

              if(current->right)
              {
                  targetSphere_right[0] = current->right->average[0];
                  targetSphere_right[1] = current->right->average[1];
                  targetSphere_right[2] = current->right->average[2];
              }

              distance_left = euclideanDistance(templateVertex, targetSphere_left);
              distance_right = euclideanDistance(templateVertex, targetSphere_right);

              if(distance_left < distance_right)
              {
                aux = current;
                previous = current;
                current = aux->left;
              }
              else
              {
                  aux = current;
                  previous = current;
                  current = aux->right;
              }
          }

          if(previous)
          {
            findCorrespondences_Naive(i, previous->start, previous->end);
          }

          else
          {
            (*m_W)(i) = 0.0;
          }
          ++i;
      }
}

void NRICP_Segment::findCorrespondences_Naive(unsigned int _templateIndex, unsigned int _targetStart, unsigned int _targetEnd)
 {
      //Find closest points between template and target mesh
      //Store in U
      //Store values in W - see 4.4.

      Vector3f templateVertex;
      Vector3f targetVertex;
      float distance = 0.0;
      float minDistance = 1000.0;
      bool foundCorrespondence = false;
      Vector3f auxUj(0.0, 0.0, 0.0);
      unsigned int three_j;

      templateVertex = m_template->getVertex(m_templateSegmentVertIndices->at(_templateIndex));
      std::vector<GLfloat>* targetVerts = m_target->getVertices();

      for(unsigned int j=_targetStart; j<=_targetEnd; ++j)
      {
       three_j = 3 * m_targetSegmentVertIndices->at(j);
       targetVertex[0] =  targetVerts->at(three_j);
       targetVertex[1] = targetVerts->at(three_j + 1);
       targetVertex[2] = targetVerts->at(three_j + 2);

       distance = euclideanDistance(templateVertex, targetVertex);
       if(distance < minDistance)
        {
         auxUj[0] = targetVertex[0];
         auxUj[1] = targetVertex[1];
         auxUj[2] = targetVertex[2];
         minDistance = distance;
         foundCorrespondence = true;
        }
       }

        if(foundCorrespondence)
        {
          Vector3f ray = auxUj - templateVertex;
          int intersection = m_template->whereIsIntersectingMesh(false, m_templateSegmentVertIndices->at(_templateIndex), templateVertex, ray);
          if(intersection < 0) //No intersection - GooD!
          {
            (*m_U)(_templateIndex, 0) = auxUj[0];
            (*m_U)(_templateIndex, 1) = auxUj[1];
            (*m_U)(_templateIndex, 2) = auxUj[2];
          }
          else
          {
           (*m_W)(_templateIndex) = 0.0;
          }
        }
        else
        {
          (*m_W)(_templateIndex) = 0.0;
        }
 }


void NRICP_Segment::determineNonRigidOptimalDeformation()
  {
    //Find X = (At*A)-1 * At * B
    //For current stiffness
    unsigned int i;
    unsigned int four_i;
    unsigned int v1;
    unsigned int v2;
    unsigned int four_v1;
    unsigned int four_v2;
    unsigned int four_l1;
    unsigned int auxRowIndex;
    unsigned int sizeRowsMG = 4 * m_templateSegmentEdgeCount;
    unsigned int sizeRowsWD = m_templateSegmentVertCount;
    unsigned int sizeRowsDl = m_templateLandmarks->size();
    unsigned int sizeColsA = 4 * m_templateSegmentVertCount;
    unsigned int weight;

//Resizing
   if(m_nricpStarted)
   {
     m_A->resize(sizeRowsMG + sizeRowsWD + sizeRowsDl, sizeColsA);
     m_B->resize(sizeRowsMG + sizeRowsWD + sizeRowsDl, 3);
     m_A->resizeNonZeros(2 * sizeRowsMG + sizeRowsWD + 4 * sizeRowsDl);
     m_B->resizeNonZeros(3 * (m_templateSegmentVertCount + sizeRowsDl));
   }

 //Alpha * M * G
   if(m_stiffnessChanged)
   {
    i = 0;
    for(std::set< std::pair<unsigned int, unsigned int> >::iterator it = m_adjMat->begin(); it != m_adjMat->end(); ++it)
     {
      four_i = 4 * i; //for all edges
      v1 = it->first;
      v2 = it->second;
      four_v1 = 4 * v1;
      four_v2 = 4 * v2;
      m_A->coeffRef(four_i, four_v1) = (-1) * m_stiffness;
      m_A->coeffRef(four_i + 1, four_v1 + 1) = (-1) * m_stiffness;
      m_A->coeffRef(four_i + 2, four_v1 + 2) = (-1) * m_stiffness;
      m_A->coeffRef(four_i + 3, four_v1 + 3) = (-1) * m_stiffness * m_gamma;
      m_A->coeffRef(four_i, four_v2) = m_stiffness;
      m_A->coeffRef(four_i + 1, four_v2 + 1) = m_stiffness;
      m_A->coeffRef(four_i + 2, four_v2 + 2) = m_stiffness;
      m_A->coeffRef(four_i + 3, four_v2 + 3) = m_stiffness * m_gamma;
      i++;
     }
     m_stiffnessChanged = false;
   }

 //W * D
    for(unsigned int i = 0; i < m_templateSegmentVertCount; ++i)
    {
     auxRowIndex = i + sizeRowsMG;
     four_i = 4 * i;
     weight = (*m_W)(i);

     m_A->coeffRef(auxRowIndex, four_i) = m_D->coeff(i, four_i) * weight;
     m_A->coeffRef(auxRowIndex, four_i + 1) = m_D->coeff(i, four_i + 1) * weight;
     m_A->coeffRef(auxRowIndex, four_i + 2) = m_D->coeff(i, four_i + 2) * weight;
     m_A->coeffRef(auxRowIndex, four_i + 3) = m_D->coeff(i, four_i + 3) * weight;

     //Weight already added
     m_B->coeffRef(auxRowIndex, 0) = (*m_U)(i, 0);
     m_B->coeffRef(auxRowIndex, 1) = (*m_U)(i, 1);
     m_B->coeffRef(auxRowIndex, 2) = (*m_U)(i, 2);
    }

  //Beta * Dl and Ul
     if(m_nricpStarted)
     {
        for(unsigned int i = 0; i < sizeRowsDl; ++i)
        {
         int l1 = m_templateLandmarks->at(i);
         int l2 = m_targetLandmarks->at(i);
         four_l1 = 4 * l1;
         Vector3f point1 = m_template->getVertex(l1);
         Vector3f point2 = m_target->getVertex(l2);

         auxRowIndex = i + sizeRowsWD + sizeRowsMG;

         m_A->coeffRef(auxRowIndex, four_l1) = m_beta * point1[0];
         m_A->coeffRef(auxRowIndex, four_l1 + 1) = m_beta * point1[1];
         m_A->coeffRef(auxRowIndex, four_l1 + 2) = m_beta * point1[2];
         m_A->coeffRef(auxRowIndex, four_l1 + 3) = m_beta;

         m_B->coeffRef(auxRowIndex, 0) = point2[0];
         m_B->coeffRef(auxRowIndex, 1) = point2[1];
         m_B->coeffRef(auxRowIndex, 2) = point2[2];
        }

        m_nricpStarted = false;
     }

     solveLinearSystem();
}

void NRICP_Segment::solveLinearSystem()
{
    //TO DO: This is the slow bit of the algorithm...understand matrices better to optimize!
    //Important stuff down here
        m_A->makeCompressed();
        m_B->makeCompressed();

        SparseLU <SparseMatrix<GLfloat> > solver;
        solver.compute((*m_A).transpose() * (*m_A));
        SparseMatrix<GLfloat> I(4 * m_templateSegmentVertCount, 4 * m_templateSegmentVertCount);
        I.setIdentity();
        SparseMatrix<GLfloat> A_inv = solver.solve(I);
        SparseMatrix<GLfloat> result = A_inv * (*m_A).transpose() * (*m_B);
        result.uncompress();
        (*m_X) = result;
}

void NRICP_Segment::deformTemplate()
  {

    //Changes to verts in segment must be attributed to the appropiate verts in the mesh
    //TO DO: Distance to segmentation plane should also be taken into consideration

      unsigned int three_i, four_i;
      MatrixXf auxMultiplication(m_templateSegmentVertCount, 3);
      auxMultiplication = (*m_D) * (*m_X);
      std::vector<GLfloat>* templateVerts = m_template->getVertices();

      //Change point values in m_D, which will change them in the mesh
      //Change points in the mesh

      for(unsigned int i = 0; i < m_templateSegmentVertCount; ++i)
      {
        three_i = 3 * m_templateSegmentVertIndices->at(i);
        four_i = 4 * i;
        m_D->coeffRef(i, four_i) = auxMultiplication(i, 0);
        m_D->coeffRef(i, four_i + 1) = auxMultiplication(i, 1);
        m_D->coeffRef(i, four_i + 2) = auxMultiplication(i, 2);

        templateVerts->at(three_i) = auxMultiplication(i, 0);
        templateVerts->at(three_i+ 1) =  auxMultiplication(i, 1);
        templateVerts->at(three_i + 2) = auxMultiplication(i, 2);
      }

      m_D->makeCompressed();
  }


void NRICP_Segment::addLandmarkCorrespondence()
{
 int l1 = m_template->getPickedVertexIndex();
 int l2 = m_target->getPickedVertexIndex();

 if(l1 >=0 && l2 >= 0)
 {
   if(findValue((unsigned int) l1, m_templateSegmentVertIndices) >= 0)
     {
       m_templateLandmarks->push_back(l1);
     }

   if(findValue((unsigned int) l2, m_targetSegmentVertIndices) >= 0)
     {
       m_targetLandmarks->push_back(l2);
     }
 }
}

void NRICP_Segment::clearLandmarkCorrespondences()
{
    m_templateLandmarks->clear();
    m_targetLandmarks->clear();
}


float NRICP_Segment::normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj)
  {
    float norm = 0.0;
    float diff = 0.0;
    unsigned int floatCount = 4 * m_templateSegmentVertCount;

    for(unsigned int i=0; i < floatCount; ++i)
    {
        for(unsigned int j=0; j<3; ++j)
        {
            diff = (*_Xj)(i, j) - (*_Xj_1)(i, j);
            norm += diff*diff;
        }
    }
    return sqrt(norm);
  }

float NRICP_Segment::euclideanDistance(Vector3f _v1, Vector3f _v2)
  {
      float diff1 = _v1[0] - _v2[0];
      float diff2 = _v1[1] - _v2[1];
      float diff3 = _v1[2] - _v2[2];

      return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
  }

int NRICP_Segment::findValue(unsigned int _value, std::vector<GLuint>* _vector)
{
    unsigned int size = _vector->size();
    unsigned int i = 0;
    bool found = false;
    int result = -1;

    while (i<size && !found)
    {
        if(_vector->at(i) == _value)
        {
            result = i;
            found = true;
        }

        ++i;
    }

    return result;
}


