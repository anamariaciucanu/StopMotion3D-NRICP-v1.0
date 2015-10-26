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
   GLuint m_templateSegmentVertCount;
   GLuint m_templateSegmentEdgeCount;
   GLuint m_targetSegmentVertCount;
   GLuint m_landmarkSegmentCorrespCount;
   GLfloat m_stiffness;
   GLfloat m_beta;
   GLfloat m_epsilon;
   GLfloat m_gamma;
   bool m_stiffnessChanged;
   bool m_landmarksChanged;

   std::vector<GLuint>* m_templateSegmentFaces;
   std::vector<GLuint>* m_targetSegmentFaces;
   std::vector<GLuint>* m_templateSegmentVertIndices;
   std::vector<GLuint>* m_targetSegmentVertIndices;
   std::vector<GLuint>* m_templateSegmentLandmarks;
   std::vector<GLuint>* m_targetSegmentLandmarks;
   std::set < std::pair<GLuint, GLuint> >* m_adjMat;

   SpherePartition* m_targetPartition;
   VectorXi* m_hasLandmark;
   VectorXf* m_W;
   MatrixXf* m_U;
   MatrixXf* m_X;
   SparseMatrix<GLfloat>* m_D;
   SparseMatrix<GLfloat, ColMajor>* m_A;
   SparseMatrix<GLfloat, ColMajor>* m_B;


public:
   NRICP_Segment(Mesh* _template, Mesh* _target);
   ~NRICP_Segment();
   void initializeNRICP();

   SpherePartition* createPartitions(GLuint _start, GLuint _end, SpherePartition* _partition);
   void destroyPartitions(SpherePartition *_partition);

   void buildVertexIndexArrays();
   void buildArcNodeMatrix();
   void buildVertexMatrix();
   void initializeWUXSVectors();
   void buildLandmarkArrays();
   void addLandmarkInformation();
   void calculateNonRigidTransformation();
   void findCorrespondences_Naive(GLuint _templateIndex, GLuint _targetStart, GLuint _targetEnd);
   void findCorrespondences();
   void determineNonRigidOptimalDeformation();
   void solveLinearSystem();
   void deformTemplate();
  GLfloat euclideanDistance(Vector3f _v1, Vector3f _v2);
  GLfloat normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj);

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
   int findValue(GLuint _value, std::vector<GLuint>* _vector);
  GLfloat calculateSegmentPlaneProximity(int _i);
  GLfloat maxDistanceFromPoint(Vector3f _point);


   /// Setters and Getters for the private members

  GLfloat getStiffness()
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
