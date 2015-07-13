#include "NRICP.h"
#include <OrderingMethods>

//TO DO: m_landmarkCorrespChanged not used anywhere
//TO DO: clean to look like NRICP_Segment

NRICP::NRICP(Mesh* _template,  Mesh* _target)
{
    m_template = _template;
    m_target = _target;
    m_beta = 1.0;
    m_epsilon = 4.0;
    m_gamma = 1.0;

    m_targetPartition = NULL;
    m_W = new VectorXi();
    m_U = new MatrixXf();
    m_X = new MatrixXf();
    m_A = new SparseMatrix<GLfloat>();
    m_B = new SparseMatrix<GLfloat>();

//Aux
   //myfile.open("../logs/times6.txt");
}

void NRICP::initializeNRICP()
{
    m_templateVertCount = m_template->getVertCount();
    m_targetVertCount = m_target->getVertCount();
    m_stiffness = 100.0;
    m_landmarkCorrespondenceCount = m_template->getLandmarkVertexIndices()->size();
    m_stiffnessChanged = true;
    m_nricpStarted = false;

    // Defining adjMat
        m_adjMat = m_template->getAdjMat();
        if(!m_adjMat)
        {
          m_template->buildArcNodeMatrix();
          m_adjMat = m_template->getAdjMat();
        }

    // Defining W
        m_W ->resize(m_templateVertCount);
        m_W->setOnes(m_templateVertCount);

    // Defining D
        m_D = m_template->getD();
        if(!m_D)
        {
            m_template->buildVertexMatrix();
            m_D = m_template->getD();
        }

    // Defining U
        m_U->resize(m_templateVertCount, 3);
        m_U->setZero(m_templateVertCount, 3);

    // Defining X
        m_X->resize(4 * m_templateVertCount, 3);
        m_X->setZero(4 * m_templateVertCount, 3);

    // Defining Sphere Partitions
       destroyPartitions(m_targetPartition);
       m_targetPartition = createPartitions(0, m_targetVertCount-1, m_targetPartition);

    //Sparse matrices A and B
    //Landmarks not included here
       m_templateEdgeCount = m_template->getEdgeCount();
       m_A->resize(4 * m_templateEdgeCount, 4 * m_templateVertCount);
       m_B->resize(4 * m_templateEdgeCount, 3);
}


NRICP::~NRICP()
{
    m_W->resize(0, 0);
    m_U->resize(0, 0);
    m_X->resize(0, 0);
    m_A->resize(0, 0);
    m_B->resize(0, 0);

    destroyPartitions(m_targetPartition);
}

SpherePartition* NRICP::createPartitions(unsigned int _start, unsigned int _end, SpherePartition* _partition)
{
    unsigned int three_i;

    //Observation: This assumes vertex data is in order
    _partition = new SpherePartition();
    _partition->start = _start;
    _partition->end = _end;
    _partition->average[0] = 0.0;
    _partition->average[1] = 0.0;
    _partition->average[2] = 0.0;

    std::vector<GLfloat>* targetVertices = m_target->getVertices();


    for(unsigned int i=_start; i<=_end; ++i)
    {
      three_i = 3*i;
      _partition->average[0] += targetVertices->at(three_i);
      _partition->average[1] += targetVertices->at(three_i + 1);
      _partition->average[2] += targetVertices->at(three_i + 2);
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

void NRICP::destroyPartitions(SpherePartition* _partition)
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

void NRICP::addLandmarkCorrespondence()
{
 int l1 = m_template->getPickedVertexIndex();
 int l2 = m_target->getPickedVertexIndex();

 if(l1 >=0 && l2 >= 0)
 {
   m_template->addLandmarkVertexIndex();
   m_target->addLandmarkVertexIndex();
   m_landmarkCorrespondenceCount ++;
 }
}

void NRICP::clearLandmarkCorrespondences()
{
    m_template->clearLandmarkVertexIndices();
    m_target->clearLandmarkVertexIndices();
    m_landmarkCorrespondenceCount = 0;
}

void NRICP::calculateRigidTransformation()
{
    //float previous_seconds = glfwGetTime();

    //Approximates ICP, unstable
    //determineRigidOptimalDeformation();
    //deformTemplate();
    //printf(" ICP takes %f seconds \n ", elapsed_seconds);

    //PCA
    m_template->moveToCentre();
    m_target->moveToCentre();

    m_template->rotateByEigenVectors();
    m_target->rotateByEigenVectors();

    //TO DO: Fix hardcoded corrections
    m_template->rotateObject(0, 90, 180);
    m_target->rotateObject(0, -90, 0);

    //float current_seconds = glfwGetTime();
    //float elapsed_seconds = current_seconds - previous_seconds;
}


void NRICP::calculateNonRigidTransformation()
{
   m_nricpStarted = true;

   float previous_seconds = glfwGetTime();

   MatrixXf* X_prev = new MatrixXf(4 * m_templateVertCount, 3);
   X_prev->setZero(4 * m_templateVertCount, 3);
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


void NRICP::findCorrespondences()
{
      std::vector<GLfloat>* vertsTemplate = m_template->getVertices();
      Vector3f templateVertex;
      float distance_left = 0.0;
      float distance_right = 0.0;
      unsigned int three_i;
      m_W->setOnes();
      m_U->setZero();

      for (unsigned int i = 0; i < m_templateVertCount; ++i)
      {
          three_i = 3*i;
          templateVertex[0] = vertsTemplate->at(three_i);
          templateVertex[1] = vertsTemplate->at(three_i + 1);
          templateVertex[2] = vertsTemplate->at(three_i + 2);
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
      }
}

void NRICP::findCorrespondences_Naive(unsigned int _templateIndex, unsigned int _targetStart, unsigned int _targetEnd)
 {
      //Find closest points between template and target mesh
      //Store in U
      //Store values in W - see 4.4.

      std::vector<GLfloat>* vertsTemplate = m_template->getVertices();
      std::vector<GLfloat>* vertsTarget = m_target->getVertices();
      Vector3f templateVertex;
      Vector3f targetVertex;
      float distance = 0.0;
      float minDistance = 1000.0;
      bool foundCorrespondence = false;
      Vector3f auxUi(0.0, 0.0, 0.0);
      unsigned int three_j;

      templateVertex[0] = vertsTemplate->at(_templateIndex * 3);
      templateVertex[1] = vertsTemplate->at(_templateIndex * 3 + 1);
      templateVertex[2] = vertsTemplate->at(_templateIndex * 3 + 2);

      for(unsigned int j=_targetStart; j<=_targetEnd; ++j)
      {
       three_j = 3*j;
       targetVertex[0] = vertsTarget->at(three_j);
       targetVertex[1] = vertsTarget->at(three_j + 1);
       targetVertex[2] = vertsTarget->at(three_j + 2);

       distance = euclideanDistance(templateVertex, targetVertex);
       if(distance < minDistance)
        {
         auxUi[0] = targetVertex[0];
         auxUi[1] = targetVertex[1];
         auxUi[2] = targetVertex[2];
         minDistance = distance;
         foundCorrespondence = true;
        }
       }

        if(foundCorrespondence)
        {
          Vector3f ray = auxUi - templateVertex;
          int intersection = m_template->whereIsIntersectingMesh(false, _templateIndex, templateVertex, ray);
          if(intersection < 0) //No intersection - GooD!
          {
        //   Vector3f templateNormal = m_template->getNormal(_templateIndex);
         //  Vector3f targetNormal = m_target->getNormal(targetIndex);
         //  float normalDot = templateNormal.dot(targetNormal);

          // if(normalDot >= 0.0 && normalDot <= 1.0)
        //  {
            (*m_U)(_templateIndex, 0) = auxUi[0];
            (*m_U)(_templateIndex, 1) = auxUi[1];
            (*m_U)(_templateIndex, 2) = auxUi[2];
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
        // }
        // else
        // {
        //  (*m_W)(_templateIndex) = 0.0;
        // }
 }

void NRICP::determineRigidOptimalDeformation()
{
    //Find X = (At*A)-1 * At * B
    unsigned int i;
    unsigned int four_i;
    unsigned int v1;
    unsigned int v2;
    unsigned int four_v1;
    unsigned int four_v2;
    unsigned int auxRowIndex;
    unsigned int sizeRowsMG = 4 * m_templateEdgeCount;
    unsigned int sizeRowsDl = m_landmarkCorrespondenceCount;
    unsigned int sizeColsA = 4 * m_templateVertCount;

//Resizing
        float auxStiffness = m_stiffness;
        m_stiffness = 100.0;
        m_A->resize(sizeRowsMG + sizeRowsDl, sizeColsA);
        m_B->resize(sizeRowsMG + sizeRowsDl, 3);
        m_A->resizeNonZeros(2 * sizeRowsMG + 4 * sizeRowsDl);
        m_B->resizeNonZeros(3 * sizeRowsDl);



 //Alpha * M * G
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

  //Beta * Dl and Ul
        std::vector<int>* landmarks_template = m_template->getLandmarkVertexIndices();
        std::vector<int>* landmarks_target = m_target->getLandmarkVertexIndices();
        for(unsigned int i = 0; i < sizeRowsDl; ++i)
        {
         unsigned int l1 = landmarks_template->at(i);
         unsigned int l2 = landmarks_target->at(i);
         unsigned four_l1 = 4 * l1;
         Vector3f point1 = m_template->getVertex(l1);
         Vector3f point2 = m_target->getVertex(l2);

         auxRowIndex = i + sizeRowsMG;

         m_A->coeffRef(auxRowIndex, four_l1) = m_beta * point1[0];
         m_A->coeffRef(auxRowIndex, four_l1 + 1) = m_beta * point1[1];
         m_A->coeffRef(auxRowIndex, four_l1 + 2) = m_beta * point1[2];
         m_A->coeffRef(auxRowIndex, four_l1 + 3) = m_beta;

         m_B->coeffRef(auxRowIndex, 0) = point2[0];
         m_B->coeffRef(auxRowIndex, 1) = point2[1];
         m_B->coeffRef(auxRowIndex, 2) = point2[2];
        }

        m_stiffness = auxStiffness;
        solveLinearSystem();
}

void NRICP::determineNonRigidOptimalDeformation()
  {
    //Find X = (At*A)-1 * At * B
    //For current stiffness
    unsigned int i;
    unsigned int four_i;
    unsigned int v1;
    unsigned int v2;
    unsigned int four_v1;
    unsigned int four_v2;
    unsigned int auxRowIndex;
    unsigned int sizeRowsMG = 4 * m_templateEdgeCount;
    unsigned int sizeRowsWD = m_templateVertCount;
    unsigned int sizeRowsDl = m_landmarkCorrespondenceCount;
    unsigned int sizeColsA = 4 * m_templateVertCount;
    unsigned int weight;

//Resizing
   if(m_nricpStarted)
   {
     m_A->resize(sizeRowsMG + sizeRowsWD + sizeRowsDl, sizeColsA);
     m_B->resize(sizeRowsMG + sizeRowsWD + sizeRowsDl, 3);
     m_A->resizeNonZeros(2 * sizeRowsMG + sizeRowsWD + 4 * sizeRowsDl);
     m_B->resizeNonZeros(3 * (m_templateVertCount + sizeRowsDl));
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
    for(unsigned int i = 0; i < m_templateVertCount; ++i)
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
         std::vector<int>* landmarks_template = m_template->getLandmarkVertexIndices();
         std::vector<int>* landmarks_target = m_target->getLandmarkVertexIndices();

        for(unsigned int i = 0; i < sizeRowsDl; ++i)
        {
         unsigned int l1 = landmarks_template->at(i);
         unsigned int l2 = landmarks_target->at(i);
         unsigned four_l1 = 4 * l1;
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

void NRICP::solveLinearSystem()
{
    //TO DO: This is the slow bit of the algorithm...understand matrices better to optimize!
    //Important stuff down here
        m_A->makeCompressed();
        m_B->makeCompressed();

        SparseLU <SparseMatrix<GLfloat> > solver;
        solver.compute((*m_A).transpose() * (*m_A));
        SparseMatrix<GLfloat> I(4 * m_templateVertCount, 4 * m_templateVertCount);
        I.setIdentity();      
        SparseMatrix<GLfloat> A_inv = solver.solve(I);
        SparseMatrix<GLfloat> result = A_inv * (*m_A).transpose() * (*m_B);
        result.uncompress();
        (*m_X) = result;
}

void NRICP::deformTemplate()
  {
      unsigned int three_i, four_i;
      std::vector<GLfloat>* vertices = m_template->getVertices();
      MatrixXf auxMultiplication(m_templateVertCount, 3);
      auxMultiplication = (*m_D) * (*m_X);

      //Change point values in m_D, which will change them in the mesh
      //Change points in the mesh

      for(unsigned int i = 0; i < m_templateVertCount; ++i)
      {
        three_i = 3 * i;
        four_i = 4 * i;
        m_D->coeffRef(i, four_i) = auxMultiplication(i, 0);
        m_D->coeffRef(i, four_i + 1) = auxMultiplication(i, 1);
        m_D->coeffRef(i, four_i + 2) = auxMultiplication(i, 2);

        vertices->at(three_i) = auxMultiplication(i, 0);
        vertices->at(three_i+ 1) =  auxMultiplication(i, 1);
        vertices->at(three_i + 2) = auxMultiplication(i, 2);
      }

      m_D->makeCompressed();
  }




float NRICP::normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj)
  {
    float norm = 0.0;
    float diff = 0.0;
    unsigned int floatCount = 4 * m_templateVertCount;

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

float NRICP::euclideanDistance(Vector3f _v1, Vector3f _v2)
  {
      float diff1 = _v1[0] - _v2[0];
      float diff2 = _v1[1] - _v2[1];
      float diff3 = _v1[2] - _v2[2];

      return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
  }


