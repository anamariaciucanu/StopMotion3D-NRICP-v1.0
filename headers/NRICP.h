//TO DO: k-D trees for meshes? or add a sphere hierarchy to template meshes to help with ray intersection

#ifndef NRICP_H
#define NRICP_H

///@file NRICP.h
///@brief Nonrigid iterative closest point algorithm for a template and a target mesh
///@author Anamaria Ciucanu
///@ref Amberg et. al (2007) Nonrigid Iterative Closest Point Algorithm

#include "mesh.h"
#include <Eigen>
#include <src/LU/Inverse.h>
#include <iostream>

///@brief Structure linking the elements of a sphere hierarchy
///@param average -> the average position of he vertices between start and end
///@param start, end -> the start and end vertex indices
///@param left -> pointer to left branch with the vertices on the left
///@param right -> pointer to the right branch with the vertices on the right
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
 ///@brief Unsigned integer -> the number of vertices in the template
    unsigned int m_templateVertCount;
 ///@brief Unsigned integer -> the number of edges in the template
    unsigned int m_templateEdgeCount;
 ///@brief Unsigned integer -> the number of vertices in the target
    unsigned int m_targetVertCount;
 ///@brief Unsigned integer -> the number of landmark corresponences
    unsigned int m_landmarkCorrespondenceCount;
 ///@brief Pointer to Mesh class object -> template mesh
    Mesh* m_template;
 ///@brief Pointer to Mesh class object -> target mesh
    Mesh* m_target;
 ///@brief Float for stiffness of the NRICP algorithm
    float m_stiffness;
 ///@brief Float for beta, the influence of the landmarks from the NRICP algorithm
    float m_beta;
 ///@brief Float for epsilon, the threshold of accepted change between any two consecutive transformations
    float m_epsilon;
 ///@brief Float gor gamma, the skew influence
    float m_gamma;
 ///@brief Pointer to map of pairs -> the edge list
    std::map < std::pair<unsigned int, unsigned int>, short >* m_adjMat;
 ///@brief Boolean value determining whether the landmark correspondences vector has been modified
    bool m_landmarkCorrespChanged;
 ///@brief Boolean value that says whether the stiffness has changed
    bool m_stiffnessChanged;
 ///@brief Checks to see if the NRICP has started
    bool m_nricpStarted;
 ///@brief Pointer to a dynamic integer vector -> weights of target correspondences
    VectorXi* m_W;
 ///@brief Pointer to sparse matrix of floats -> the template vertex information in an n x 4n matrix
    SparseMatrix<GLfloat>* m_D;
 ///@brief  Pointer to dynamic float matrix -> contains the target correspondences
    MatrixXf* m_U;
 ///@brief Pointer to a dynamic float matrix -> contains the transformation per mesh, per vertex matrix, from NRICP algorithm
    MatrixXf* m_X;
 ///@brief Pointer to hierarchical sphere structure of the target vertex information
    SpherePartition* m_targetPartition;
 ///@brief Pointer to a sparse matrix of floats -> m_A matrix from NRICP, used to determine m_X
    SparseMatrix<GLfloat>* m_A;
 ///@brief Pointer to a sparse matrix of floats -> m_B matrix from NRICP, used to determine m_X
    SparseMatrix<GLfloat>* m_B;


 public:
    ///@brief ctor of NRICP class object
    ///@param [in] _template -> template mesh
    ///@param [in] _target -> target mesh
    NRICP(Mesh* _template,  Mesh* _target);    
    ///@brief dtor of NRICP class object
    ~NRICP();
    ///@brief initializes elements needed before the algorithm starts
    void initializeNRICP();
    ///@brief creates sphere hierarchy partitions
    ///@param [in] _start, _end Unsigned integers -> the start and end indices of a partition
    ///@param [in] _partition -> the parent partition for the new hierarchy member to be created
    ///@param [out] Pointer to Sphere Partition hierarchy memebr created
    SpherePartition* createPartitions(unsigned int _start, unsigned int _end, SpherePartition* _partition);
    ///@brief destroys the sphere hierarchy tree
    ///@param [in] Pointer to sphere partition -> the root of the hierarchy
    void destroyPartitions(SpherePartition *_partition);
    ///@brief asks the template mesh to build the vertex matrix, m_D
    void buildVertexMatrix();
    ///@brief performs an NRICP transformation on the template, to morph it closer to the target mesh
    void calculateNonRigidTransformation();
    ///@brief performs a hybrid ICP transformation on the template, to morph it closer to the target mesh
    void calculateRigidTransformation();
    ///@brief finds the closest target vertex to the template vertex, from a restrained group of target vertices
    ///@param [in] _templateIndex Unsigned integer -> the index of the template vertex
    ///@param [in] _targetStart, _targetEnd Unsigned integers -> the start and end vertex indices on the target mesh
    void findCorrespondences_Naive(unsigned int _templateIndex, unsigned int _targetStart, unsigned int _targetEnd);
    ///@brief Fills in m_U with target correspondences for every template vertex
    void findCorrespondences();
    ///@brief Finds m_X transformation matrix for the template mesh for ICP
    void determineRigidOptimalDeformation();
    ///@brief Finds m_X transformation matrix for the template mesh for NRICP
    void determineNonRigidOptimalDeformation();
    ///@brief Solves AX=B
    void solveLinearSystem();
    ///@brief Deforms the template using the m_X matrix generated from the previous step
    void deformTemplate();
    ///@brief Adds a landmark template-target vertex correspondence to the list
    void addLandmarkCorrespondence();
    ///@brief Clears landmark correspondences list
    void clearLandmarkCorrespondences();
    ///@brief Modifies stiffness by a certain value
    ///@param [in] _value Float variable by which the stiffness is modified
    void modifyStiffness(float _value)
    {
      m_stiffness += _value;
      if(m_stiffness < 1.0)
      {
          m_stiffness = 1.0;
      }
      m_stiffnessChanged = true;
    }
    ///@brief Modifies beta by a certain value
    ///@param [in] _value Float variable by which beta is modified
    void modifyBeta(float _value)
    {
      m_beta += _value;
      if(m_beta < 0.0)
      {
          m_beta = 0.0;
      }
    }    
    ///@brief Calculates the euclidean distance between two vertices
    ///@param [in] _v1, _v2 Vectors of size 3, representing the positions of vertices _v1 and _v2 respectively
    ///@param [out] Float variable -> the euclidean distance between the two vertices
    float euclideanDistance(Vector3f _v1, Vector3f _v2);
    ///@brief Calculates the Frobenius norm of the difference between 2 matrices
    ///@param [in] _Xj_1, _Xj Dynamic matrices of floats -> transformation matricesfrom 2 consecutive iterations
    ///@param [out] Float variable representing the norm
    float normedDifference(MatrixXf* _Xj_1, MatrixXf* _Xj);

    /// Setters and Getters for the private members
    void setLandmarkCorrespChanged(bool _value) { m_landmarkCorrespChanged = _value; }
    void setNRICPStarted(bool _value) { m_nricpStarted = _value; }
    float getStiffness() { return m_stiffness; }
    void setTemplate(Mesh* _template){ m_template = _template; }
    Mesh* getTemplate(){ return m_template; }
    void setTarget(Mesh* _target){ m_target = _target; }
    Mesh* getTarget() { return m_target; }
};
#endif // NRICP_H
