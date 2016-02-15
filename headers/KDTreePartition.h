#ifndef SPHEREPARTITION_H
#define SPHEREPARTITION_H

class KDTreeNode
{
  private:

  public:
    KDTreeNode()
    {
     m_left = NULL;
     m_right = NULL;
     m_vertexIndex = -1;
     m_axis = -1;
     m_median = -1000;
    }

    ~KDTreeNode(){}

    int m_axis;
    float m_median;
    int m_vertexIndex;
    KDTreeNode* m_left;
    KDTreeNode* m_right;
};
#endif // SPHEREPARTITION_H
