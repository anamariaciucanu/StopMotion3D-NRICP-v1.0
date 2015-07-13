#ifndef NRICP_SEGMENT_H
#define NRICP_SEGMENT_H

#include "mesh.h"
#include <Eigen>
#include <src/LU/Inverse.h>
#include <iostream>
#include "sphere_partition.h"


class NRICP_Segment
{
private:
   Mesh* m_template;
   Mesh* m_target;
   unsigned int m_templateSegmentVertCount;
   unsigned int m_templateSegmentEdgeCount;
   unsigned int m_targetSegmentVertCount;
   std::vector<GLuint>* m_templateSegmentFaces;
   std::vector<GLuint>* m_targetSegmentFaces;
   std::vector<GLuint>* m_templateSegmentVertIndices;
   std::vector<GLuint>* m_targetSegmentVertIndices;
   std::set < std::pair<unsigned int, unsigned int> >* m_adjMat;
   float m_stiffness;
   float m_beta;
   float m_epsilon;
   float m_gamma;
   bool m_stiffnessChanged;
   bool m_nricpStarted;

   VectorXi* m_W;
   SparseMatrix<GLfloat>* m_D;
   MatrixXf* m_U;
   MatrixXf* m_X;
   SpherePartition* m_targetPartition;
   SparseMatrix<GLfloat>* m_A;
   SparseMatrix<GLfloat>* m_B;
   std::vector<unsigned int>* m_templateLandmarks;
   std::vector<unsigned int>* m_targetLandmarks;


public:

   NRICP_Segment(Mesh* _template, Mesh* _target);
   ~NRICP_Segment();
   void initializeNRICP();

   SpherePartition* createPartitions(unsigned int _start, unsigned int _end, SpherePartition* _partition);
   void destroyPartitions(SpherePartition *_partition);

   void buildVertexIndexArrays();
   void buildArcNodeMatrix();
   void buildVertexMatrix();
   void initializeWUXVectors();
   void buildLandmarkArrays();

   void calculateNonRigidTransformation();
   void findCorrespondences_Naive(unsigned int _templateIndex, unsigned int _targetStart, unsigned int _targetEnd);
   void findCorrespondences();
   void determineNonRigidOptimalDeformation();
   void solveLinearSystem();
   void deformTemplate();
   float euclideanDistance(Vector3f _v1, Vector3f _v2);
   float normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj);   

   void modifyStiffness(float _value)
   {
     m_stiffness += _value;
     if(m_stiffness < 1.0)
     {
         m_stiffness = 1.0;
     }
     m_stiffnessChanged = true;
   }
   void modifyBeta(float _value)
   {
     m_beta += _value;
     if(m_beta < 0.0)
     {
         m_beta = 0.0;
     }
   }
   void addLandmarkCorrespondence();
   void clearLandmarkCorrespondences();
   int findValue(unsigned int _value, std::vector<GLuint>* _vector);




   /// Setters and Getters for the private members
   void setNRICPStarted(bool _value)
   {
       m_nricpStarted = _value;
   }

   float getStiffness()
   {
       return m_stiffness;
   }
   void setTemplate(Mesh* _template)
   {
       m_template = _template;
   }

   Mesh* getTemplate()
   {
       return m_template;
   }

   void setTarget(Mesh* _target)
   {
       m_target = _target;
   }

   Mesh* getTarget()
   {
       return m_target;
   }

};

#endif // NRICP_SEGMENT_H
