#define THRESHOLD 0.1
#define RADIUS 0.2
#include "NRICP_Segment.h"
#include <OrderingMethods>
#include <set>

//(!) Observation: Paper proves that matrix A is nondegenerate and has column rank 4n...still, I believe
// that since there are so many zeros in the matrix, values may become unstable, which is why we get so many artefacts
//Indices are related to segment
//TO DO: Debug to see why some transformations are taking so long

NRICP_Segment::NRICP_Segment(Mesh* _template,  Mesh* _target)
{
    m_template = _template;
    m_target = _target;

    m_templateSegmentVertCount = 0;
    m_targetSegmentVertCount = 0;
    m_landmarkSegmentCorrespCount = 0;
    m_templateSegmentEdgeCount = 0;
    m_beta = 1.0;
    m_epsilon = 5.0;
    m_gamma = 1.0;
    m_landmarksChanged = false;

    m_templateSegmentVertIndices = new std::vector<GLuint>();
    m_targetSegmentVertIndices = new std::vector<GLuint>();
    m_templateSegmentLandmarks = new std::vector<GLuint>();
    m_targetSegmentLandmarks = new std::vector<GLuint>();
    m_adjMat = NULL;

    m_hasLandmark = new VectorXi();
    m_W = new VectorXf();
    m_U = new MatrixXf();
    m_X = new MatrixXf();
    m_D = NULL;
    m_A = new SparseMatrix<GLfloat, ColMajor>();
    m_B = new SparseMatrix<GLfloat, ColMajor>();
}


void NRICP_Segment::initializeNRICP()
{
    m_templateSegmentFaces = m_template->getActiveSegment();
    m_targetSegmentFaces = m_target->getActiveSegment();

    m_stiffness = 100.0;
    m_stiffnessChanged = true;

    buildVertexIndexArrays(); //Segment indices holding mesh indices
    buildArcNodeMatrix(); //m_adjMat
    buildVertexMatrix(); //m_D
    initializeWUXSVectors(); //m_W, m_U, m_X
    buildLandmarkArrays(); //landmarks in segment indices

    //Sparse matrices A and B
    m_templateSegmentEdgeCount = m_adjMat->size();

    //Mighty matrices
    GLuint sizeRowsMG = 4 * m_templateSegmentEdgeCount;
    GLuint sizeColsA = 4 * m_templateSegmentVertCount;

    m_A->resize(sizeRowsMG + m_templateSegmentVertCount, sizeColsA);
    m_B->resize(sizeRowsMG + m_templateSegmentVertCount, 3);
    m_A->reserve(2 * sizeRowsMG + sizeColsA);
    m_B->reserve(3 * m_templateSegmentVertCount);
}



NRICP_Segment::~NRICP_Segment()
{
    m_W->resize(0, 0);
    m_U->resize(0, 0);
    m_X->resize(0, 0);
    m_A->resize(0, 0);
    m_B->resize(0, 0);
    m_D->resize(0, 0);
    m_hasLandmark->resize(0);

    if (m_templateSegmentVertIndices) { delete [] m_templateSegmentVertIndices; }
    if (m_targetSegmentVertIndices) { delete [] m_targetSegmentVertIndices; }
    if (m_templateSegmentLandmarks) { delete [] m_templateSegmentLandmarks; }
    if (m_targetSegmentLandmarks) { delete []  m_targetSegmentLandmarks; }

    m_adjMat->clear();
}


void NRICP_Segment::addLandmarkCorrespondence()
{
 int l1 = m_template->getPickedVertexIndex();
 int l2 = m_target->getPickedVertexIndex();
 int laux1, laux2;

 if(l1 >=0 && l2 >= 0)
 {
   laux1 = findValue((GLuint) l1, m_templateSegmentVertIndices);
   laux2 = findValue((GLuint) l2, m_targetSegmentVertIndices);
   if(laux1 > 0 && laux2 > 0)
     {
      m_templateSegmentLandmarks->push_back(laux1);
      m_targetSegmentLandmarks->push_back(laux2);
      m_landmarkSegmentCorrespCount++;
     }
 }
    m_landmarksChanged = true;
}


void NRICP_Segment::clearLandmarkCorrespondences()
{
    m_templateSegmentLandmarks->clear();
    m_targetSegmentLandmarks->clear();
    m_hasLandmark->resize(m_templateSegmentVertCount);
    m_hasLandmark->setZero();
    m_landmarkSegmentCorrespCount = 0;
    m_landmarksChanged = true;
}


void NRICP_Segment::addLandmarkInformation()
{
   if(m_landmarksChanged)
    {
      m_beta = 1.0;
      m_landmarksChanged = false;
      m_hasLandmark->setZero(m_templateSegmentVertCount);
    }

   int four_l1, l1, l2;
   Vector3f l1_point, l2_point;

   for(GLuint i = 0; i < m_landmarkSegmentCorrespCount; ++i)
   {
     l1 = m_templateSegmentLandmarks->at(i);
     l2 = m_targetSegmentLandmarks->at(i);
     four_l1 = 4 * l1;

     l1_point = m_template->getVertex(m_templateSegmentVertIndices->at(l1));
     l2_point = m_target->getVertex(m_targetSegmentVertIndices->at(l2));

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


void NRICP_Segment::buildVertexIndexArrays()
{
    //Template segment ===========================================
    std::set<GLuint> templateSegmentVertIndices;
    std::set<GLuint> targetSegmentVertIndices;
    std::set<GLuint>::iterator it1, it2;

    m_templateSegmentVertIndices->clear();
    m_targetSegmentVertIndices->clear();

    GLuint size = m_templateSegmentFaces->size();
    for(GLuint i = 0; i < size; ++i)
    {        
        templateSegmentVertIndices.insert(m_templateSegmentFaces->at(i));
    }

    //Target segment ==============================================
    size = m_targetSegmentFaces->size();
    for(GLuint i = 0; i < size; ++i)
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

    if(m_adjMat)
    {
     m_adjMat->clear();
     m_templateSegmentEdgeCount = 0;
    }
    else
    {
      m_adjMat = new std::set < std::pair<GLuint, GLuint> >();
    }

     std::set < std::pair<GLuint, GLuint> >::iterator it;
     GLuint three_i;
     GLuint v1, v2, v3;
     GLuint min, max;
     GLuint faceCountTemplate = m_templateSegmentFaces->size()/3;

     //Mesh vert indices turned into segment indices
     for (GLuint i = 0; i < faceCountTemplate ; ++i)
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



void NRICP_Segment::buildVertexMatrix()
{
  if(m_D)
  {
    m_D->resize(m_templateSegmentVertCount, 4 * m_templateSegmentVertCount);
    m_D->reserve(m_templateSegmentVertCount * 4);
  }
  else
  {
     m_D = new SparseMatrix<float>(m_templateSegmentVertCount , 4 * m_templateSegmentVertCount);
     m_D->reserve(m_templateSegmentVertCount * 4);
  }

  GLuint three_i = 0;
  GLuint four_i = 0;
  std::vector<GLfloat>* templateVerts = m_template->getVertices();

  for(GLuint i = 0; i < m_templateSegmentVertCount; ++i)
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


void NRICP_Segment::initializeWUXSVectors()
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
  int segmentLandmarkIndex;

  clearLandmarkCorrespondences();

  //Adding mesh landmarks
  for(int i=0; i<sizeTemplateLandmarks; ++i)
  {
    segmentLandmarkIndex = findValue(templateLandmarks->at(i), m_templateSegmentVertIndices);

    if(segmentLandmarkIndex >= 0)
    {
        m_templateSegmentLandmarks->push_back(segmentLandmarkIndex);
    }
  }

  for(int j=0; j<sizeTargetLandmarks; ++j)
  {
      segmentLandmarkIndex = findValue(targetLandmarks->at(j), m_targetSegmentVertIndices);

      if(segmentLandmarkIndex >= 0)
      {
          m_targetSegmentLandmarks->push_back(segmentLandmarkIndex);
      }
  }
}


//Nonrigid transformations
void NRICP_Segment::calculateNonRigidTransformation()
{
   GLfloat previous_seconds = glfwGetTime();

   MatrixXf* X_prev = new MatrixXf(4 * m_templateSegmentVertCount, 3);
   X_prev->setZero(4 * m_templateSegmentVertCount, 3);
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

   m_template->calculateNormals();
   m_target->calculateNormals();

   GLfloat current_seconds = glfwGetTime();
   GLfloat elapsed_seconds = current_seconds - previous_seconds;
   printf(" Deformation takes %.2f seconds \n ", elapsed_seconds);
}


void NRICP_Segment::findCorrespondences()
{
    Vector3f templateVertex;
    Vector3f targetVertex;
    GLuint templateIndex;
    int targetIndex;

    //Very important to reset weights to 1
    m_W->setOnes();


    for (GLuint i=0; i<m_templateSegmentVertCount; ++i)
    {
      if(!(*m_hasLandmark)(i))
        {
           templateIndex = m_templateSegmentVertIndices->at(i);
           templateVertex = m_template->getVertex(templateIndex);
           targetIndex = m_target->findClosestPoint(templateVertex);

           if(targetIndex >= 0)
            {
              targetVertex = m_target->getVertex((GLuint)targetIndex);
              Vector3f ray = targetVertex - templateVertex;
              int intersection = m_template->whereIsIntersectingMesh(false, templateIndex, templateVertex, ray);
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
                 //  (*m_W)(i) = 0.0;
                 // }


                 //Modify weights based on the distance to the segmentation plane
                 //Weight formula taken from Li et. al (2009)
                   GLfloat dP = calculatePointToPlaneDistance(templateVertex);

                   if(dP <= THRESHOLD)
                    {
                     GLfloat weight = max (0.0, 1 - pow((pow(dP, 2)/pow(RADIUS, 2)), 3));
                     (*m_W)(i) = weight;
                    }
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


void NRICP_Segment::determineNonRigidOptimalDeformation()
  {
    //Find X = (At*A)-1 * At * B
    //For current stiffness
    GLuint i;
    GLuint four_i;
    GLuint v1;
    GLuint v2;
    GLuint four_v1;
    GLuint four_v2;
    GLuint auxRowIndex;
    GLuint sizeRowsMG = 4 * m_templateSegmentEdgeCount;
    GLfloat weight;

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
    for(GLuint i = 0; i < m_templateSegmentVertCount; ++i)
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

void NRICP_Segment::solveLinearSystem()
{
    //TO DO: This is the slow bit of the algorithm...understand matrices better to optimize!
    //Important stuff down here
        m_A->makeCompressed();
        m_B->makeCompressed();

        SparseLU <SparseMatrix<GLfloat, ColMajor>, COLAMDOrdering<int> > solver;
        solver.compute((*m_A).transpose() * (*m_A));        
        SparseMatrix<GLfloat, ColMajor> I(4 * m_templateSegmentVertCount, 4 * m_templateSegmentVertCount);
        I.setIdentity();
        SparseMatrix<GLfloat, ColMajor> A_inv = solver.solve(I);
        SparseMatrix<GLfloat, ColMajor> result = A_inv * (*m_A).transpose() * (*m_B);
        result.uncompress();
        (*m_X) = result;
}

void NRICP_Segment::deformTemplate()
  {

    //Changes to verts in segment must be attributed to the appropiate verts in the mesh
    //TO DO: Distance to segmentation plane should also be taken into consideration

      GLuint three_i, four_i;
      MatrixXf auxMultiplication(m_templateSegmentVertCount, 3);
      auxMultiplication = (*m_D) * (*m_X);
      std::vector<GLfloat>* templateVerts = m_template->getVertices();

      //Change point values in m_D, which will change them in the mesh
      //Change points in the mesh

      for(GLuint i = 0; i < m_templateSegmentVertCount; ++i)
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



//Auxiliaries ---------------------------------------------------------------------------


GLfloat NRICP_Segment::normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj)
  {
    GLfloat norm = 0.0;
    GLfloat diff = 0.0;
    GLuint floatCount = 4 * m_templateSegmentVertCount;

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

GLfloat NRICP_Segment::euclideanDistance(Vector3f _v1, Vector3f _v2)
  {
      GLfloat diff1 = _v1[0] - _v2[0];
      GLfloat diff2 = _v1[1] - _v2[1];
      GLfloat diff3 = _v1[2] - _v2[2];

      return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
  }

int NRICP_Segment::findValue(GLuint _value, std::vector<GLuint>* _vector)
{
    GLuint size = _vector->size();
    GLuint i = 0;
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

GLfloat NRICP_Segment::calculatePointToPlaneDistance(Vector3f _point)
{
  Vector3f planeCenter = m_template->calculateActiveSegmentationPlaneCentre();
  Vector3f pointCentre = _point - planeCenter;
  Vector3f planeNormal = m_template->calculateActiveSegmentationPlaneNormal();
  GLfloat normalMagnitude = euclideanDistance(Vector3f(0, 0, 0), planeNormal);

  GLfloat distance = (fabs(pointCentre.dot(planeNormal)))/normalMagnitude;

  return distance;
}

GLfloat NRICP_Segment::maxDistanceFromPoint(Vector3f _point)
{
  GLfloat maxDistance = -1;

  for(GLuint i = 0; i < m_templateSegmentVertCount; ++i)
  {
      Vector3f vert = m_template->getVertex(m_templateSegmentVertIndices->at(i));
      GLfloat distance = euclideanDistance(vert, _point);

      if(distance > maxDistance)
      {
        maxDistance = distance;
      }
  }

  return maxDistance;
}
