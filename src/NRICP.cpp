#include "NRICP.h"
#include <OrderingMethods>

//TO DO: m_landmarkCorrespChanged not used anywhere
//TO DO: clean to look like NRICP_Segment
//TO DO: Stiffness will be an array calculated at each iteration for each vertex
//TO DO: Optimize and debug determineNonRigidOptimalDeformation
//TO DO: Adapt KD-tree NN search for different buckets
//TO DO: Play more with various weights
//TO DO: Fix local minima errors that keep appearing

NRICP::NRICP(Mesh* _template,  Mesh* _target)
{
    m_template = _template;
    m_target = _target;
    m_beta = 1.0;
    m_epsilon = 5.0;
    m_gamma = 1.0;

    m_W = new VectorXi();
    m_hasLandmark = new VectorXi();
    m_U = new MatrixXf();
    m_X = new MatrixXf();
    m_A = new SparseMatrix<GLfloat>();
    m_B = new SparseMatrix<GLfloat>();
    m_landmarksChanged = false;
}

void NRICP::initializeNRICP()
{
    m_templateVertCount = m_template->getVertCount();
    m_targetVertCount = m_target->getVertCount();
    m_stiffness = 10.0;
    m_stiffnessChanged = true;

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

    //Defining hasLandmark
        m_hasLandmark->resize(m_templateVertCount);
        m_hasLandmark->setZero();

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

    //Sparse matrices A and B
       m_templateEdgeCount = m_template->getEdgeCount();

    //Mighty matrices
       GLuint sizeRowsMG = 4 * m_templateEdgeCount;
       GLuint sizeColsA = 4 * m_templateVertCount;

       m_A->resize(sizeRowsMG + m_templateVertCount, sizeColsA);
       m_B->resize(sizeRowsMG + m_templateVertCount, 3);
       m_A->reserve(2 * sizeRowsMG + sizeColsA);
       m_B->reserve(3 * m_templateVertCount);
}


NRICP::~NRICP()
{
    m_W->resize(0, 0);
    m_hasLandmark->resize(0);
    m_U->resize(0, 0);
    m_X->resize(0, 0);
    m_A->resize(0, 0);
    m_B->resize(0, 0);
}


void NRICP::addLandmarkCorrespondence()
{
 int l1 = m_template->getPickedVertexIndex();
 int l2 = m_target->getPickedVertexIndex();

 if(l1 >=0 && l1 < (int)m_template->getVertCount()
    && l2 >= 0 && l2 < (int)m_target->getVertCount())
 {
   m_template->addLandmarkVertexIndex();
   m_target->addLandmarkVertexIndex();
   m_landmarksChanged = true;
 }
}

void NRICP::clearLandmarkCorrespondences()
{
    m_template->clearLandmarkVertexIndices();
    m_target->clearLandmarkVertexIndices();
    m_hasLandmark->setZero();
    m_landmarksChanged = true;
}

void NRICP::addLandmarkInformation()
{
    if(m_landmarksChanged)
     {
        m_beta = 1.0;
        m_landmarksChanged = false;
     }

   std::vector<int>* landmarksTemplate = m_template->getLandmarkVertexIndices();
   std::vector<int>* landmarksTarget = m_target->getLandmarkVertexIndices();
   int four_l1, l1, l2;
   int landmarkCorrespondenceCount = landmarksTemplate->size();
   Vector3f l1_point, l2_point;

   for(int i = 0; i < landmarkCorrespondenceCount; ++i)
   {
     l1 = landmarksTemplate->at(i);
     l2 = landmarksTarget->at(i);
     four_l1 = 4*l1;

     l1_point = m_template->getVertex(l1);
     l2_point = m_target->getVertex(l2);

     //Template side in m_D
     m_D->coeffRef(l1, four_l1) = m_beta * l1_point[0];
     m_D->coeffRef(l1, four_l1 + 1) = m_beta * l1_point[1];
     m_D->coeffRef(l1, four_l1 + 2) = m_beta * l1_point[2];
     m_D->coeffRef(l1, four_l1 + 3) = 1.0;

     //Target side in m_U
     (*m_U)(l1, 0) = l2_point[0];
     (*m_U)(l1, 1) = l2_point[1];
     (*m_U)(l1, 2) = l2_point[2];

     //Boolean value
     (*m_hasLandmark)(l1) = 1;
   }
}

GLfloat NRICP::normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj)
  {
    GLfloat norm = 0.0;
    GLfloat diff = 0.0;
    GLuint floatCount = 4 * m_templateVertCount;

    for(GLuint i=0; i < floatCount; ++i)
    {
        for(GLuint j=0; j<3; ++j)
        {
            diff = (*_Xj)(i, j) - (*_Xj_1)(i, j);
            norm += diff*diff;
        }
    }
    return sqrt(norm);
  }

GLfloat NRICP::euclideanDistance(Vector3f _v1, Vector3f _v2)
  {
      GLfloat diff1 = _v1[0] - _v2[0];
      GLfloat diff2 = _v1[1] - _v2[1];
      GLfloat diff3 = _v1[2] - _v2[2];

      return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
  }



//NRICP ===========================================================================


void NRICP::calculateRigidTransformation()
{
    //PCA
    m_template->moveToCentre();
    m_target->moveToCentre();

    m_template->calculateEigenvectors();
    m_target->calculateEigenvectors();

    m_template->rotateByEigenVectors();
    m_target->rotateByEigenVectors();
}

void NRICP::calculateNonRigidTransformation()
{
   GLfloat previous_seconds = glfwGetTime();

   MatrixXf* X_prev = new MatrixXf(4 * m_templateVertCount, 3);
   X_prev->setZero(4 * m_templateVertCount, 3);
   m_stiffnessChanged = true;

   addLandmarkInformation();
   findCorrespondences();
   determineNonRigidOptimalDeformation();
   deformTemplate();
   GLfloat changes = normedDifference(X_prev, m_X);

   while (changes > m_epsilon)
   {
    (*X_prev) = (*m_X);
    findCorrespondences();
    determineNonRigidOptimalDeformation();
    deformTemplate();
    changes = normedDifference(X_prev, m_X);
   }

   delete X_prev;

   GLfloat current_seconds = glfwGetTime();
   GLfloat elapsed_seconds = current_seconds - previous_seconds;
   printf(" NRICP takes %f seconds \n ", elapsed_seconds);
}


void NRICP::findCorrespondences()
{
    //If a landmark is not already in place, we find a correspondence
    //Else we use landmark information

      Vector3f templateVertex;
      Vector3f targetVertex;
      int targetIndex;
      //Very important to reset weights to 1
      m_W->setOnes();


      for (GLuint i = 0; i < m_templateVertCount; ++i)
      {
        if(!(*m_hasLandmark)(i))
         {
              templateVertex = m_template->getVertex(i);
              targetIndex = m_target->findClosestPoint(templateVertex);

              if(targetIndex >= 0)
               {
                 targetVertex = m_target->getVertex((GLuint)targetIndex);
                 Vector3f ray = targetVertex - templateVertex;
                 int intersection = m_template->whereIsIntersectingMesh(false, i, templateVertex, ray);
                 if(intersection < 0) //No intersection - GooD!
                  {
                    // templateNormal = m_template->getNormal(i);
                    // targetNormal = m_target->getNormal(targetIndex);
                    // dotProduct = getDotProduct(templateNormal, targetNormal);
                    // printf("Dot product IZ %f \n", dotProduct);

                    // if(dotProduct >= 0.0 && dotProduct <= 1.0)
                    // {
                       (*m_U)(i, 0) = targetVertex[0];
                       (*m_U)(i, 1) = targetVertex[1];
                       (*m_U)(i, 2) = targetVertex[2];
                    // }
                    // else
                    // {
                    //  (*m_W)(_templateIndex) = 0.0;
                    // }
                   }
                   else //There was a self-intersection in the template
                   {
                    (*m_W)(i) = 0.0;
                   }
                }
                else //Target correspondence was not found
                {
                  (*m_W)(i) = 0.0;
                }
         }
      }
}



void NRICP::determineNonRigidOptimalDeformation()
  {
    //Find X = (At*A)-1 * At * B
    //For current stiffness
    GLuint i, four_i;
    GLuint v1, v2;
    GLuint four_v1, four_v2;
    GLuint auxRowIndex;
    GLuint sizeRowsMG = 4 * m_templateEdgeCount;
    GLuint weight;


 //Alpha * M * G

   if(m_stiffnessChanged)
   {
    i = 0;
    for(std::set< std::pair<GLuint, GLuint> >::iterator it = m_adjMat->begin(); it != m_adjMat->end(); ++it)
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
    for(GLuint i = 0; i < m_templateVertCount; ++i)
    {
      auxRowIndex = i + sizeRowsMG;
      four_i = 4 * i;
      weight = (*m_W)(i);

      m_A->coeffRef(auxRowIndex, four_i) = m_D->coeff(i, four_i) * weight;
      m_A->coeffRef(auxRowIndex, four_i + 1) = m_D->coeff(i, four_i + 1) * weight;
      m_A->coeffRef(auxRowIndex, four_i + 2) = m_D->coeff(i, four_i + 2) * weight;
      m_A->coeffRef(auxRowIndex, four_i + 3) = m_D->coeff(i, four_i + 3) * weight;

      m_B->coeffRef(auxRowIndex, 0) = (*m_U)(i, 0) * weight;
      m_B->coeffRef(auxRowIndex, 1) = (*m_U)(i, 1) * weight;
      m_B->coeffRef(auxRowIndex, 2) = (*m_U)(i, 2) * weight;
    }  

    solveLinearSystem();
}

void NRICP::solveLinearSystem()
{
    //TO DO: This is the slow bit of the algorithm...understand matrices better to optimize!
    //Important stuff down here
        m_A->makeCompressed();
        m_B->makeCompressed();

        SparseLU <SparseMatrix<GLfloat, ColMajor>, COLAMDOrdering<int>  > solver;
        solver.compute((*m_A).transpose() * (*m_A));
        SparseMatrix<GLfloat, ColMajor> I(4 * m_templateVertCount, 4 * m_templateVertCount);
        I.setIdentity();      
        SparseMatrix<GLfloat, ColMajor> A_inv = solver.solve(I);
        SparseMatrix<GLfloat, ColMajor> result = A_inv * (*m_A).transpose() * (*m_B);
        result.uncompress();
        (*m_X) = result;
}

void NRICP::deformTemplate()
  {
      GLuint three_i, four_i;
      std::vector<GLfloat>* vertices = m_template->getVertices();
      MatrixXf auxMultiplication(m_templateVertCount, 3);
      auxMultiplication = (*m_D) * (*m_X);

      //Change point values in m_D, which will change them in the mesh
      //Change points in the mesh

      for(GLuint i = 0; i < m_templateVertCount; ++i)
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





