#include "ICP.h"


ICP::ICP(Mesh *_template, Mesh *_target)
{
    m_template = _template;
    m_target = _target;
    m_landmarkCorrespondences = new std::vector<std::pair<unsigned int, unsigned int> >();
    m_landmarkCorrespondenceCount = 0;
}

ICP::~ICP(){}

void ICP::addLandmarkCorrespondence()
{
    int l1 = m_template->getPickedVertexIndex();
    int l2 = m_target->getPickedVertexIndex();

    if(l1 >=0 && l2 >= 0)
    {
      m_landmarkCorrespondences->push_back(make_pair((unsigned int)l1, (unsigned int)l2));
      m_landmarkCorrespondenceCount++;
    }
}

void ICP::calculateTransformations()
{
    if(m_landmarkCorrespondenceCount > 0)
    {
      m_A = new MatrixXf(m_landmarkCorrespondenceCount, 4 * m_landmarkCorrespondenceCount);
      m_X = new MatrixXf(4 * m_landmarkCorrespondenceCount, 3);
      m_B = new MatrixXf(m_landmarkCorrespondenceCount, 3);
      unsigned int four_i;

      for(unsigned int i = 0; i < m_landmarkCorrespondenceCount; ++i)
      {
          four_i = 4 * i;
          unsigned int l1 = m_landmarkCorrespondences->at(i).first;
          unsigned int l2 = m_landmarkCorrespondences->at(i).second;
          Vector3f point1 = m_template->getVertex(l1);
          Vector3f point2 = m_target->getVertex(l2);

          m_A->coeffRef(i, four_i) = point1[0];
          m_A->coeffRef(i, four_i + 1) = point1[1];
          m_A->coeffRef(i, four_i + 2) = point1[2];
          m_A->coeffRef(i, four_i + 3) = 1.0;

          m_B->coeffRef(i, 0) = point2[0];
          m_B->coeffRef(i, 1) = point2[1];
          m_B->coeffRef(i, 2) = point2[2];
      }

      //Calculate m_X
      Eigen::FullPivLU<MatrixXf> solver(*m_A);
      (*m_X) = solver.solve(*m_B);

      //Transform vertices
      unsigned int templateVertCount = m_template->getVertCount();
      SparseMatrix<float>* D = m_template->getD();
      MatrixXf auxMultiplication(templateVertCount, 3);
      auxMultiplication = (*D) * (*m_X);

      //Change point values in m_D, which will change them in the mesh
      //Change points in the mesh

      for(unsigned int i = 0; i < templateVertCount; ++i)
      {
        four_i = 4 * i;
        D->coeffRef(i, four_i) = auxMultiplication(i, 0);
        D->coeffRef(i, four_i + 1) = auxMultiplication(i, 1);
        D->coeffRef(i, four_i + 2) = auxMultiplication(i, 2);
      }

      m_template->changeVertsBasedOn_D_Matrix();

      //Delete m_A, m_X and m_B
      delete m_A;
      delete m_X;
      delete m_B;
    }
}
