#ifndef SPHEREPARTITION_H
#define SPHEREPARTITION_H

class SpherePartition
{
  private:

  public:
    SpherePartition()
    {
     left = NULL;
     right = NULL;
    }

    ~SpherePartition(){}

    float average[3];
    unsigned int start;
    unsigned int end;
    SpherePartition* left;
    SpherePartition* right;
};
#endif // SPHEREPARTITION_H
