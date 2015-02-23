#ifndef NRICP_H
#define NRICP_H

#include "mesh.h"
#include <Eigen>
#include <src/LU/Inverse.h>
#include <iostream>

struct SpherePartition
{
  float average[3];
  unsigned int start;
  unsigned int end;
  SpherePartition* left;
  SpherePartition* right;
};

class NRICP
{
private:
    Mesh* m_template;
    Mesh* m_target;
    float m_stiffness;
    float m_epsilon;
    float m_gamma;
    unsigned int m_templateVertCount;
    unsigned int m_templateEdgeCount;
    unsigned int m_targetVertCount;
    std::map < std::pair<unsigned int, unsigned int>, short >* m_adjMat;
    VectorXi* m_W;
    SparseMatrix<float>* m_D;
    MatrixXf* m_U;
    MatrixXf* m_X;
    SpherePartition* m_targetPartition;
    SparseMatrix<GLfloat>* m_A;
    SparseMatrix<GLfloat>* m_B;

    //Aux
    ofstream myfile;
    unsigned int m_targetAuxIndex;
    unsigned int m_templateAuxIndex;

public:
    NRICP(Mesh* _template,  Mesh* _target);
    ~NRICP();

    SpherePartition *createPartitions(unsigned int _start, unsigned int _end, SpherePartition* _partition);
    void destroyPartitions(SpherePartition *_partition);
    void buildVertexMatrix();
    void calculateTransformation();
    float normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj);
    void findCorrespondences_Naive(unsigned int _templateIndex, unsigned int _targetStart, unsigned int _targetEnd);
    void findCorrespondences();
    void determineOptimalDeformation();
    void deformTemplate();
    void modifyStiffness(float _val)
    {
      m_stiffness += _val;
      if(m_stiffness < 1.0)
      {
          m_stiffness = 1.0;
      }
    }

    //Auxiliary
    float getStiffness() { return m_stiffness;}
    float euclideanDistance(Vector3f _v1, Vector3f _v2);
    Mesh* getTemplate(){ return m_template; }
    Mesh* getTarget() { return m_target; }
    int getTargetAuxIndex(){ return m_targetAuxIndex; }
    int getTemplateAuxIndex(){ return m_templateAuxIndex; }
};
#endif // NRICP_H
