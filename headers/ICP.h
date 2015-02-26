#ifndef ICP_H
#define ICP_H
#include "mesh.h"

//TO DO: A lot
class ICP
{
private:
    Mesh* m_template;
    Mesh* m_target;
    std::vector<std::pair<unsigned int, unsigned int> >* m_landmarkCorrespondences;
    unsigned int m_landmarkCorrespondenceCount;
    MatrixXf* m_X;
    MatrixXf* m_A;
    MatrixXf* m_B;

public:
    ICP(Mesh* _template, Mesh* _target);
    ~ICP();
    void addLandmarkCorrespondence();
    void clearLandmarkCorrespondences()
    {
     m_landmarkCorrespondences->clear();
     m_landmarkCorrespondenceCount = 0;
    }

    void calculateTransformations();
};
#endif // ICP_H
