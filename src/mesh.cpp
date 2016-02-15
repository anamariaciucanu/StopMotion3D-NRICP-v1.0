#include "mesh.h"
#include <math.h>
#include <boost/tokenizer.hpp>

#define EPSILON 0.000001


//TO DO: Fix normals issue (normal count >> vert count?)
//Temp fix: Calculate my own normals when loading mesh

//Indices are related to mesh

using namespace Eigen;

Mesh::Mesh()
{
    m_vertices = new std::vector<GLfloat>();
    m_normals = new std::vector<GLfloat>();
    m_texcoords = new std::vector<GLfloat>();
    m_faceIndices = new std::vector<GLuint>();
    m_partitionRoot = NULL;
    m_adjMat = NULL;
    m_D = NULL;
    m_centre[0] = 0.0;
    m_centre[1] = 0.0;
    m_centre[2] = 0.0;
    m_vertCount = 0;
    m_edgeCount = 0;
    m_faceCount = 0;
    m_texCoordCount = 0;
    m_pickedIndex = -1;
    m_wireframe = false;
    m_landmarkVertexIndices = new std::vector<int>();
    m_segmentationRoot = NULL;
    m_segmentationMode = false;
    m_segmentsIndices = new std::vector<GLuint>();
    m_activeSegment = -1;
    m_activeSegmentationPlane = -1;
}

Mesh::~Mesh()
{
 if(m_vertices)
 {
   delete [] m_vertices;
 }

 if(m_normals)
 {
   delete [] m_normals;
 }

 if(m_texcoords)
 {
   delete [] m_texcoords;
 }

 if(m_faceIndices)
 {
   delete [] m_faceIndices;
 }

 if(m_adjMat)
 {
   delete [] m_adjMat;
 }

 if(m_D)
 {
   delete m_D;
 }

 if(m_neighbours)
 {
   delete [] m_neighbours;
 }

 if(m_landmarkVertexIndices)
 {
   delete [] m_landmarkVertexIndices;
 }

 destroySegments(m_segmentationRoot);
 destroyMeshPartitions(m_partitionRoot);
}


//Create mesh -----------------------------
bool Mesh::loadMesh(const char *_fileName)
{
    ifstream in(_fileName, ios::in);
    if (!in)
    {
     printf("Cannot open %s",_fileName);
     return false;
    }


    string line;
    while (getline(in, line))
      {
       if (line.substr(0,2) == "v ")
       {
           istringstream s(line.substr(2));
           vec3 vert;
           s >> vert.v[0];
           s >> vert.v[1];
           s >> vert.v[2];

           m_vertices->push_back((GLfloat)vert.v[0]);
           m_vertices->push_back((GLfloat)vert.v[1]);
           m_vertices->push_back((GLfloat)vert.v[2]);
           m_vertCount++;

           continue;
        }
/*
         if (line.substr(0,2) == "vt")
        {
           istringstream s(line.substr(2));
           vec2 texture;
           s >> texture.v[0];
           s >> texture.v[1];

           m_texcoords->push_back((GLfloat)texture.v[0]);
           m_texcoords->push_back((GLfloat)texture.v[1]);
           m_texCoordCount++;

           continue;
        }

         if (line.substr(0,2) == "vn")
        {
           istringstream s(line.substr(2));
           vec3 normal;
           s >> normal.v[0];
           s >> normal.v[1];
           s >> normal.v[2];

           m_normals->push_back((GLfloat)normal.v[0]);
           m_normals->push_back((GLfloat)normal.v[1]);
           m_normals->push_back((GLfloat)normal.v[2]);

           continue;
         }
*/
        if (line.substr(0,2) == "f ")
        {
            istringstream s(line.substr(2));
            //char* tokens;
            string a;
            string b;
            string c;

            s >> a;
            s >> b;
            s >> c;

            boost::char_separator<char> sep("/");
            boost::tokenizer< boost::char_separator<char> > tokens_a(a, sep);
            boost::tokenizer< boost::char_separator<char> > tokens_b(b, sep);
            boost::tokenizer< boost::char_separator<char> > tokens_c(c, sep);
            //for(tokenizer< char_separator<char> >::iterator beg=tokens.begin(); beg!=tokens.end();++beg)
            //{}
            if(tokens_a.begin()!=tokens_a.end())
            {
             m_faceIndices->push_back((GLuint)(std::atoi((*tokens_a.begin()).c_str()) - 1));
            }

            if(tokens_b.begin()!=tokens_b.end())
            {
             m_faceIndices->push_back((GLuint)std::atoi((*tokens_b.begin()).c_str()) -1);
            }

            if(tokens_c.begin()!=tokens_c.end())
            {
             m_faceIndices->push_back((GLuint)std::atoi((*tokens_c.begin()).c_str()) -1);
            }
            m_faceCount++;

          continue;
        }
     }

    moveToCentre();
    calculateCentre();

    return true;
}

void Mesh::normaliseMesh()
{
 //Find min and max for x, y, z
 GLfloat min_x = 1000.0;
 GLfloat min_y = 1000.0;
 GLfloat min_z = 1000.0;
 GLfloat max_x = -1000.0;
 GLfloat max_y = -1000.0;
 GLfloat max_z = -1000.0;
 GLfloat aux;
 GLuint three_i;

 for (GLuint i = 0; i < m_vertCount; ++i)
  {
        three_i = 3*i;

        aux = m_vertices->at(three_i);
        if(aux < min_x) { min_x = aux;}
        else if (aux > max_x) { max_x = aux;}

        aux = m_vertices->at(three_i + 1);
        if(aux < min_y) { min_y = aux;}
        else if (aux > max_y) { max_y = aux;}

        aux = m_vertices->at(three_i + 2);
        if(aux < min_z) { min_z = aux;}
        else if (aux > max_z) { max_z = aux;}
    }

    for (GLuint i = 0; i < m_vertCount; ++i)
    {
        three_i = 3*i;
        GLfloat x  = m_vertices->at(three_i);
        GLfloat y = m_vertices->at(three_i + 1);
        GLfloat z = m_vertices->at(three_i + 2) ;

        m_vertices->at(three_i) = 2.0 * (x - min_x) / (max_x - min_x) - 1.0;
        m_vertices->at(three_i + 1) = 2.0 * (y - min_y) / (max_y - min_y) - 1.0;
        m_vertices->at(three_i + 2) = 2.0 * (z - min_z) / (max_z - min_z) - 1.0;
    }
}

void Mesh::calculateNormals()
{
  if(m_normals->size()<1)
  {
      for(GLuint k=0; k<m_vertices->size(); ++k)
      {
       m_normals->push_back(0.0);
      }
  }
  else
  {
    for(GLuint k=0; k<m_normals->size(); ++k)
    {
     m_normals->at(k) = 0.0;
    }
  }

  GLuint three_i;
  unsigned short v1;
  unsigned short v2;
  unsigned short v3;
  GLuint three_v1;
  GLuint three_v2;
  GLuint three_v3;

  for (GLuint i = 0; i < m_faceCount; ++i)
  {
   three_i = 3 * i;
   v1 = m_faceIndices->at(three_i);
   v2 = m_faceIndices->at(three_i + 1);
   v3 = m_faceIndices->at(three_i + 2);
   three_v1 = 3 * v1;
   three_v2 = 3 * v2;
   three_v3 = 3 * v3;

   Vector3f normal(0.0, 0.0, 0.0);
   Vector3f point1(m_vertices->at(three_v1), m_vertices->at(three_v1 + 1), m_vertices->at(three_v1 + 2));
   Vector3f point2(m_vertices->at(three_v2), m_vertices->at(three_v2 + 1), m_vertices->at(three_v2 + 2));
   Vector3f point3(m_vertices->at(three_v3), m_vertices->at(three_v3 + 1), m_vertices->at(three_v3 + 2));
   Vector3f edge1 = 10*(point2 - point1);
   Vector3f edge2 = 10*(point3 - point1);

   normal = edge1.cross(edge2);

   m_normals->at(three_v1) += normal[0];
   m_normals->at(three_v1 + 1) += normal[1];
   m_normals->at(three_v1 + 2) += normal[2];

   m_normals->at(three_v2) += normal[0];
   m_normals->at(three_v2 + 1) += normal[1];
   m_normals->at(three_v2 + 2) += normal[2];

   m_normals->at(three_v3) += normal[0];
   m_normals->at(three_v3 + 1) += normal[1];
   m_normals->at(three_v3 + 2) += normal[2];
  }

  normaliseNormals();
}

void Mesh::normaliseNormals()
{
    GLuint three_i;
    for(GLuint i = 0; i < m_vertCount; i++)
    {
        GLfloat magnitude = 0.0;
        three_i = 3 * i;
        GLfloat x = m_normals->at(three_i);
        GLfloat y = m_normals->at(three_i + 1);
        GLfloat z = m_normals->at(three_i + 2);
        magnitude = sqrt(x * x + y * y + z * z);

        if(abs(magnitude) > 1.0)
        {
         m_normals->at(three_i) /= magnitude;
         m_normals->at(three_i + 1) /= magnitude;
         m_normals->at(three_i + 2) /= magnitude;
        }
    }
}

void Mesh::partitionMesh()
{
 std::vector<int> pointList;

 for(int i=0; i<(int)m_vertCount; ++i)
 {
   pointList.push_back(i);
 }

 if(m_partitionRoot == NULL)
 {
  m_partitionRoot = new KDTreeNode();
 }

 partitionMeshIteration(m_partitionRoot, pointList, 0);
}

void Mesh::partitionMeshIteration(KDTreeNode* _root, std::vector<int> _pointList, int _depth)
{
    int three_i, three_j, aux_index;
    int pointCount = _pointList.size();
    int medianVertexIndex;
    int axis = _depth % 3;

    //Round robin kDTree approach
    //Wiki says it was invented by Bentley et. al (1975)

    if(pointCount>1)
    {
      std::vector<int> pointListLeft;
      std::vector<int> pointListRight;
      GLfloat term1, term2;

     //Selection sort pointlist by axis component =>N^2 complexity (TO DO: Improve sorting)
      for (int i=0; i<pointCount-1; ++i)
      {
        for(int j=i+1; j<pointCount; ++j)
        {
            three_i = 3 * _pointList.at(i);
            three_j = 3 * _pointList.at(j);
            term1 = m_vertices->at(three_i + axis);
            term2 = m_vertices->at(three_j + axis);
            if(term1 > term2)
            {
                aux_index = _pointList.at(i);
                _pointList.at(i) = _pointList.at(j);
                _pointList.at(j) = aux_index;
            }
        }
       }

     _root->m_axis = axis;
     medianVertexIndex = _pointList.at(pointCount/2);
     _root->m_median = m_vertices->at(medianVertexIndex*3 + axis);

     for (int i=0; i<pointCount/2; ++i)
     {
      pointListLeft.push_back(_pointList.at(i));
     }

     for(int i=pointCount/2; i<pointCount; ++i)
     {
      pointListRight.push_back(_pointList.at(i));
     }

     _root->m_left = new KDTreeNode();
     _root->m_right = new KDTreeNode();

     partitionMeshIteration(_root->m_left, pointListLeft, _depth+1);
     partitionMeshIteration(_root->m_right, pointListRight, _depth+1);
    }
    else
    {
     _root->m_vertexIndex = _pointList.at(pointCount-1);
    }
}

void Mesh::destroyMeshPartitions(KDTreeNode* _root)
{
    if(_root)
    {
        if(!_root->m_left && !_root->m_right)
        {
            delete _root;
        }

        if (_root->m_right)
        {
           destroyMeshPartitions(_root->m_right);
        }

        if (_root->m_left)
        {
           destroyMeshPartitions(_root->m_left);
        }
    }
}

int Mesh::findClosestPoint(Vector3f _query)
{
  int refPoint = 0;
  float refDistance = 1000;
  findClosestPointIteration1(_query, m_partitionRoot, refPoint, refDistance);
  return refPoint;
}

 void Mesh::findClosestPointIteration1(Vector3f _query, KDTreeNode* _root, int &_refPoint, float &_refDistance)
 {
    //Algorithm taken from
    //http://andrewd.ces.clemson.edu/courses/cpsc805/references/nearest_search.pdf

    if(_root->m_left == NULL && _root->m_right == NULL)
    {
      float distance = euclideanDistance(_query, getVertex(_root->m_vertexIndex));

      if(distance < _refDistance)
      {
        _refDistance = distance;
        _refPoint = _root->m_vertexIndex;
      }
    }
    else
    {
      int axis = _root->m_axis;
      if(_query[axis] <= _root->m_median)
      {
        //Search left first
        if(_query[axis] - _refDistance <= _root->m_median)
        {
          findClosestPointIteration1(_query, _root->m_left, _refPoint, _refDistance);
        }
        if(_query[axis] + _refDistance > _root->m_median)
        {
          findClosestPointIteration1(_query, _root->m_right, _refPoint, _refDistance);
        }
      }
      else
      {
        //Search right first
         if(_query[axis] + _refDistance > _root->m_median)
          {
            findClosestPointIteration1(_query, _root->m_right, _refPoint, _refDistance);
          }
          if(_query[axis] - _refDistance <= _root->m_median)
          {
            findClosestPointIteration1(_query, _root->m_left, _refPoint, _refDistance);
          }
      }
    }
 }


//Drawing --------------------------
void Mesh::bindVAOs()
{
    //VAO
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);

 //Bind those vertex positions
  if(m_vertCount > 0)
    {
     glGenBuffers(1, &m_vboPositions);
     glBindBuffer(GL_ARRAY_BUFFER, m_vboPositions);
     glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount * sizeof(GLfloat), &m_vertices->at(0), GL_STATIC_DRAW);
     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL); //0 corresponds to vertex positions in VAO
     glEnableVertexAttribArray(0);
    }
  else
    {
     printf("ERROR: No vertices! \n");
     return;
    }

 //Bind those normals positions
  if(m_normals->size() > 0)
    {
     glGenBuffers(1, &m_vboNormals);
     glBindBuffer(GL_ARRAY_BUFFER, m_vboNormals);
     glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount  * sizeof(GLfloat), &m_normals->at(0), GL_STATIC_DRAW);
     glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL); //1 corresponds to normals in VAO
     glEnableVertexAttribArray(1);
    }
  else
    {
      printf("ERROR: No normals! \n");
      return;
    }

   if(m_vertCount > 0)
    {
      if(!m_segmentationMode)
      {
       glGenBuffers(1, &m_vboIndices);
       glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vboIndices);
       glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * m_faceCount * sizeof(GLuint), &m_faceIndices->at(0), GL_STATIC_DRAW);
      }
      else
      {
       glGenBuffers(1, &m_vboIndices);
       glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vboIndices);
       glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_segmentsIndices->size() * sizeof(GLuint), &m_segmentsIndices->at(0), GL_STATIC_DRAW);
      }
    }
   else
   {
      printf("ERROR: No vertices! \n");
      return;
   }
}

void Mesh::unbindVAOs()
{
 glDeleteBuffers(1, &m_vboPositions);
 glDeleteBuffers(1, &m_vboIndices);
 glDeleteBuffers(1, &m_vboNormals);
 glDeleteVertexArrays(1, &m_vao);
}


//Build sparse matrices -----------------------
void Mesh::buildArcNodeMatrix()
{
  //Create the arc-node adjacency matrix

    if(!m_adjMat)
    {
     m_adjMat = new std::set < std::pair<GLuint, GLuint> >();
     std::set < std::pair<GLuint, GLuint> >::iterator it;
     GLuint three_i;
     GLuint v1;
     GLuint v2;
     GLuint v3;
     GLuint min;
     GLuint max;

     for (GLuint i = 0; i < m_faceCount; ++i)
     {
        three_i = 3*i;
        v1 = m_faceIndices->at(three_i);
        v2 = m_faceIndices->at(three_i + 1);
        v3 = m_faceIndices->at(three_i + 2);

        min = v1 < v2? v1 : v2;
        max = v1 > v2? v1 : v2;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(min, max));
            m_edgeCount++;
        }

        min = v2 < v3? v2 : v3;
        max = v2 > v3? v2 : v3;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(min, max));
            m_edgeCount++;
        }

        min = v1 < v3 ? v1 : v3;
        max = v1 > v3 ? v1 : v3;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(min, max));
            m_edgeCount++;
        }
     }
   }
}

void Mesh::buildVertexMatrix()
{
  if(!m_D)
  {
     m_D = new SparseMatrix<float>(m_vertCount, 4 * m_vertCount);
     m_D->reserve(m_vertCount * 4);
  }

  GLuint three_i = 0;
  GLuint four_i = 0;

  for(GLuint i = 0; i < m_vertCount; i++)
  {
   three_i = 3 * i;
   four_i = 4 * i;
   m_D->insert(i, four_i) = m_vertices->at(three_i);
   m_D->insert(i, four_i + 1) =  m_vertices->at(three_i + 1);
   m_D->insert(i, four_i + 2) = m_vertices->at(three_i + 2);
   m_D->insert(i, four_i + 3) = 1;
  }

  m_D->makeCompressed();
}

void Mesh::buildNeighbourList()
{
 if(m_adjMat)
 {
  int v1, v2;

 //if(m_neighbours)
 // {
  //  delete [] m_neighbours;
 // }
  m_neighbours = new std::vector<std::vector<int> >(m_vertCount);


  for(std::set< std::pair<GLuint, GLuint> >::iterator it = m_adjMat->begin(); it != m_adjMat->end(); ++it)
  {
      v1 = it->first;
      v2 = it->second;

      m_neighbours->at(v1).push_back(v2);
      m_neighbours->at(v2).push_back(v1);
  }
 }
 else
 {
  buildArcNodeMatrix();
  buildNeighbourList();
 }
}


//Mesh analysis ----------------------------
bool Mesh::findInListOfNeighbours(int _neighbour1, int _neighbour2)
{
  std::vector<int> neighbours = m_neighbours->at(_neighbour1);
  GLuint size = neighbours.size();
  bool found = false;

  for(GLuint i=0; i<size && !found; ++i)
  {
      if(neighbours.at(i) == _neighbour2)
      {
          found = true;
      }
  }

  return found;
}

float Mesh::calculateVertexCurvature(int _index)
{

    float curvature = -10000;
    std::vector<int> neighbours = m_neighbours->at(_index);
    GLuint noNeighbours = neighbours.size();
    std::vector<int> edges;
    int neighbour1;
    int neighbour2;
    float angleSum = 0.0;
    float areaSum = 0.0;
    float diff = 0.0;
    float dirac = 0.0;

    //Create triangle list
    for(GLuint i = 0; i < noNeighbours - 1; ++i)
    {
      for(GLuint j = i+1; j < noNeighbours; ++j)
      {
        neighbour1 = neighbours.at(i);
        neighbour2 = neighbours.at(j);

        if(findInListOfNeighbours(neighbour1, neighbour2))
        {
         //Found neighbour2 in list of neighbours of neighbour1
          edges.push_back(neighbour1);
          edges.push_back(neighbour2);
        }
      }
    }

      //we know the 3 indices of the traingle
      //calculate index angle and triangle area
      Vector3f v1 = getVertex(_index);
      GLuint i = 0;

      while(i < edges.size())
       {
           Vector3f v2 = getVertex(edges.at(i));
           Vector3f v3 = getVertex(edges.at(i+1));

           //Dirac delta
           diff = euclideanDistance(v1, v2);
           dirac = 1/diff; //inversely proportional
           //Law of cosines
           float a = euclideanDistance(v1, v2);
           float b = euclideanDistance(v1, v3);
           float c = euclideanDistance(v2, v3);
           float s = (a + b + c)/2;
           float cosine = (a*a + b*b - c*c)/(2*a*b);
           float angle = acos(cosine);
           angleSum += angle;
           areaSum += sqrt(s*(s-a)*(s-b)*(s-c));
           i = i + 2;
        }

     curvature = (3 * (2 * 3.14159 - angleSum) * dirac * dirac) / areaSum;

     return curvature;
}

int Mesh::whereIsIntersectingMesh(bool _culling, int _originTemplateIndex, Vector3f _origin, Vector3f _ray)
{
 ///@ref Fast, Minimum Storage Ray/Triangle Intersection, Möller & Trumbore. Journal of Graphics Tools, 1997.
    Vector3f point1, point2, point3;
    Vector3f edge1, edge2;
    Vector3f P, Q, T;
    float det, inv_det, u, v;
    float t = 0.0f;
    GLuint i = 0;
    GLuint three_i;
    int v1, v2, v3;


    while(i < m_faceCount)
    {
      three_i = 3*i;
      i++;
      v1 = m_faceIndices->at(three_i);
      v2 = m_faceIndices->at(three_i+ 1);
      v3 = m_faceIndices->at(three_i + 2);

      if(v1 != _originTemplateIndex && v2 != _originTemplateIndex && v3 != _originTemplateIndex)
      {
        point1 = getVertex(v1);
        point2 = getVertex(v2);
        point3 = getVertex(v3);

        //Find vectors for two edges sharing V1
        edge1 = point2 - point1;
        edge2 = point3 - point1;

        //Begin calculating determinant - also used to calculate u parameter
         P = _ray.cross(edge2);

        //if determinant is near zero, ray lies in plane of triangle
         det = edge1.dot(P);


         if(_culling)
         {
           //Define test_cull if culling is desired
           if(det < EPSILON) continue;

           //calculate distance from origin to point1
           T = _origin - point1;

           //Calculate u parameter and test bound
           u = T.dot(P);

           //The intersection lies outside of the triangle
           if(u < 0.f || u > det) continue;

           //Prepare to test v parameter
           Q = T.cross(edge1);

           //Calculate V parameter and test bound
           v = _ray.dot(Q);

           //The intersection lies outside of the triangle
           if(v < 0.f || u + v  > det) continue;

           t = edge2.dot(Q);
           inv_det = 1.0f/det;

           //Scale parameters
           t *= inv_det;
           u *= inv_det;
           v *= inv_det;
         }

         else //Non-culling
         {
            if(det > -EPSILON && det < EPSILON) continue;
            inv_det = 1.0f/det;

            //calculate distance from origin to point1
            T = _origin - point1;

            //Calculate u parameter and test bound
            u = T.dot(P) * inv_det;

            //The intersection lies outside of the triangle
            if(u < 0.f || u > 1.0f) continue;

            //Prepare to test v parameter
            Q = T.cross(edge1);

            //Calculate V parameter and test bound
            v = _ray.dot(Q) * inv_det;

            //The intersection lies outside of the triangle
            if(v < 0.f || u + v  > 1.0f) continue;

            t = edge2.dot(Q) * inv_det;
         }

        if(t > EPSILON)
        { //ray intersection
           //Find intersection point
            Vector3f intersection = _origin + t * _ray;
            float min = 100;
            int imin = -1;

            float distance1 = euclideanDistance(intersection, point1);
            float distance2 = euclideanDistance(intersection, point2);
            float distance3 = euclideanDistance(intersection, point3);

            if(distance1 < min)
            {
              min = distance1;
              imin = v1;
            }

            if (distance2 < min)
            {
                min = distance2;
                imin = v2;
            }

            if(distance3 < min)
            {
               min = distance3;
               imin = v3;
            }

            return imin;
        }
      }
    }

 return -1;
}


//Transformations of mesh --------------------
//TO DO: Test them
void Mesh::affineTransformation(MatrixXf _X)
{
    GLuint three_i;
    Vector3f vertex;

    for(GLuint i=0; i < m_vertCount; ++i)
    {
        three_i = 3*i;
        vertex[0] = m_vertices->at(three_i);
        vertex[1] = m_vertices->at(three_i + 1);
        vertex[2] = m_vertices->at(three_i + 2);

        vertex[0] = _X(0,0) * vertex[0] + _X(1, 0)* vertex[1] + _X(2, 0)*vertex[2] + _X(3, 0);
        vertex[1] = _X(0,1) * vertex[0] + _X(1, 1)* vertex[1] + _X(2, 1)*vertex[2] + _X(3, 1);
        vertex[2] = _X(0,2) * vertex[0] + _X(1, 2)* vertex[1] + _X(2, 2)*vertex[2] + _X(3, 2);

        m_vertices->at(three_i) = vertex[0];
        m_vertices->at(three_i + 1) = vertex[1];
        m_vertices->at(three_i + 2) = vertex[2];
    }
    calculateCentre();
}

void Mesh::moveObject(float _tX, float _tY, float _tZ)
{
  int three_i;
  mat4 T = translate(identity_mat4(), vec3(_tX, _tY, _tZ));
  vec4 vertex;
  vertex.v[3] = 1.0;

  for(GLuint i=0; i<m_vertCount; ++i)
  {
     three_i = 3*i;
     vertex.v[0] = m_vertices->at(three_i);
     vertex.v[1] = m_vertices->at(three_i + 1);
     vertex.v[2] = m_vertices->at(three_i + 2);

     vertex = T * vertex;

     m_vertices->at(three_i) = vertex.v[0];
     m_vertices->at(three_i + 1) = vertex.v[1];
     m_vertices->at(three_i + 2) = vertex.v[2];
  }

  calculateCentre();
}

void Mesh::moveObject(Vector3f _trans)
{
  moveObject(_trans[0], _trans[1], _trans[2]);
  calculateCentre();
}

void Mesh::calculateCentre()
{
    GLuint three_i;
    m_centre[0] = 0.0;
    m_centre[1] = 0.0;
    m_centre[2] = 0.0;

    for(GLuint i=0; i<m_vertCount; ++i)
    {
        three_i = 3*i;

        m_centre[0] += m_vertices->at(three_i);
        m_centre[1] += m_vertices->at(three_i + 1);
        m_centre[2] += m_vertices->at(three_i + 2);
    }

    m_centre[0] /= m_vertCount;
    m_centre[1] /= m_vertCount;
    m_centre[2] /= m_vertCount;
}

void Mesh::moveToCentre()
{
   calculateCentre();
   moveObject(-m_centre[0], -m_centre[1], -m_centre[2]);
   calculateCentre();
}

void Mesh::rotateObject(float _angleX, float _angleY, float _angleZ)
{
  GLuint three_i;
  mat4 Rx = rotate_x_deg(identity_mat4(), _angleX);
  mat4 Ry= rotate_y_deg(identity_mat4(), _angleY);
  mat4 Rz = rotate_z_deg(identity_mat4(), _angleZ);
  mat4 R = Rz * Ry * Rx;
  vec4 vertex;
  vertex.v[3] = 1.0;

  for(GLuint i=0; i<m_vertCount; ++i)
  {
     three_i = 3*i;
     vertex.v[0] = m_vertices->at(three_i);
     vertex.v[1] = m_vertices->at(three_i + 1);
     vertex.v[2] = m_vertices->at(three_i + 2);

     vertex = R * vertex;

     m_vertices->at(three_i) = vertex.v[0];
     m_vertices->at(three_i + 1) = vertex.v[1];
     m_vertices->at(three_i + 2) = vertex.v[2];
  }

  calculateCentre();
}

void Mesh::rotateObject(Matrix3f _R)
{
    GLuint three_i;
    Vector3f vertex;

    for(GLuint i=0; i<m_vertCount; ++i)
    {
       three_i = 3*i;
       vertex[0] = m_vertices->at(three_i);
       vertex[1] = m_vertices->at(three_i + 1);
       vertex[2] = m_vertices->at(three_i + 2);

       vertex = _R * vertex;

       m_vertices->at(three_i) = vertex[0];
       m_vertices->at(three_i + 1) = vertex[1];
       m_vertices->at(three_i + 2) = vertex[2];
    }

    calculateCentre();
}


//Eigenvectors -----------------------
bool Mesh::areEigenvectorsOrthogonal()
{
    Vector3f v1;
    Vector3f v2;
    Vector3f v3;

    v1[0] = m_eigenvectors(0, 0);
    v1[1] = m_eigenvectors(1, 0);
    v1[2] = m_eigenvectors(2, 0);

    v2[0] = m_eigenvectors(0, 1);
    v2[1] = m_eigenvectors(1, 1);
    v2[2] = m_eigenvectors(2, 1);

    v3[0] = m_eigenvectors(0, 2);
    v3[1] = m_eigenvectors(1, 2);
    v3[2] = m_eigenvectors(2, 2);

    double dot12 = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    double dot23 = v2[0]*v3[0] + v2[1]*v3[1] + v2[2]*v3[2];

    bool isDot12 = (dot12 < 0.1)&&(dot12 > -0.1);
    bool isDot23 = (dot23 < 0.1)&&(dot23 > -0.1);

    if(isDot12 && isDot23)
    {
      return true;
    }

    return false;
}

void Mesh::rotateByEigenVectors()
{
    //after moving to centre of OpenGL screen

  if (areEigenvectorsOrthogonal())
  {
      Matrix3f A = m_eigenvectors;
      Matrix3f B;
      B.setZero();

      for(GLuint k=0; k<3; ++k)
      {
        for(GLuint l=0; l<3; ++l)
        {
            if(A(k,l) > 0.8)
            {
              B(k,l) = 1.0;
            }
            else
            if(A(k, l) < -0.8)
            {
             B(k, l) = -1.0;
            }
        }
      }


      Matrix3f R = B*A.inverse();
      Vector3f vertex;
      int three_i;

      for(GLuint i=0; i<m_vertCount; ++i)
      {
        three_i = 3*i;

        vertex[0] = m_vertices->at(three_i);
        vertex[1] = m_vertices->at(three_i + 1);
        vertex[2] = m_vertices->at(three_i + 2);

        vertex = R * vertex;

        m_vertices->at(three_i) = vertex[0];
        m_vertices->at(three_i + 1) = vertex[1];
        m_vertices->at(three_i + 2) = vertex[2];
      }

      calculateCentre();
   }
  else
  {
      //Eigenvectors are not orthonormal
      //Build new ones?
  }
}

void Mesh::calculateEigenvectors()
{
    Matrix3f covarianceMatrix;
    covarianceMatrix.setZero();
    GLuint three_i;
    Vector3f v;

    //Covariance Matrix
    for(GLuint i=0; i<m_vertCount; ++i)
    {
       three_i = 3*i;
       v[0] = m_vertices->at(three_i);
       v[1] = m_vertices->at(three_i+1);
       v[2] = m_vertices->at(three_i+2);

       for(GLuint j=0; j<3; ++j)
       {
         for(GLuint k=0; k<3; ++k)
         {
           covarianceMatrix(j, k) = covarianceMatrix(j, k) + v[j]*v[k];
         }
       }
    }
    covarianceMatrix /= m_vertCount;

    SelfAdjointEigenSolver<Matrix3f> es;
    es.compute(covarianceMatrix);

    //Eigenvectors and eigenvalues
    m_eigenvectors = es.eigenvectors();
    m_eigenvalues = es.eigenvalues();
}


//Segmentation -----------------------
void Mesh::segmentMesh()
{
    GLuint size = m_landmarkVertexIndices->size();

    if(size >= 3)
    {
        Vector3i plane;
        Vector3f normal;
        Vector3f point1;
        Vector3f point2;
        Vector3f point3;

        for(GLuint i=0; i<3; ++i)
        {
          plane[i] = m_landmarkVertexIndices->at(size-1);
          m_landmarkVertexIndices->pop_back();
          size--;
        }

        for(GLuint i=0; i<3; ++i)
        {
          point1[i] = m_vertices->at(3*plane[0] + i);
          point2[i] = m_vertices->at(3*plane[1] + i);
          point3[i] = m_vertices->at(3*plane[2] + i);
        }

        normal = (point2 - point1).cross(point3 - point1);
        normal.normalize();

        m_segmentationRoot = segmentationProcedure(plane, normal, m_segmentationRoot, NULL);
        createSegmentList();
        m_segmentationMode = true;
        m_activeSegment = 0;
        m_activeSegmentationPlane = m_segmentationPlanes.size()-1;
    }
    else
    {
      //Do nothing, tralalalalalala
    }
}

void Mesh::splitSegmentIntoSubsegments(Segmentation* _root, std::vector<GLuint>* _parentSideFaces, Vector3f _planeCentre, Vector3f _planeNormal)
{
    GLuint size = _parentSideFaces->size();
    GLuint three_i;
    float dot_directions;
    Vector3f faceCentre;
    Vector3i face;
    Vector3f planeFaceDirection;

    for(GLuint i=0; i<size/3; ++i)
    {
     three_i = 3*i;
     face[0] = _parentSideFaces->at(three_i);
     face[1] = _parentSideFaces->at(three_i + 1);
     face[2] = _parentSideFaces->at(three_i + 2);

     faceCentre = calculateTriangleCentre(face[0], face[1], face[2]);
     planeFaceDirection = faceCentre - _planeCentre;
     planeFaceDirection.normalize();
     dot_directions = _planeNormal.dot(planeFaceDirection);

   if(dot_directions > 0 && dot_directions <= 1)
     {
       //Above plane - segment left
       _root->m_leftFaces->push_back(face[0]);
       _root->m_leftFaces->push_back(face[1]);
       _root->m_leftFaces->push_back(face[2]);
     }
     else
     {
       //Below plane - segment right
       _root->m_rightFaces->push_back(face[0]);
       _root->m_rightFaces->push_back(face[1]);
       _root->m_rightFaces->push_back(face[2]);
     }
    }
}

Segmentation* Mesh::segmentationProcedure(Vector3i _plane, Vector3f _normal, Segmentation* _root, std::vector<GLuint>* _parentSideFaces)
{
    if(_root)
    {
       _root->m_leftSegment = segmentationProcedure(_plane, _normal, _root->m_leftSegment, _root->m_leftFaces);
       _root->m_rightSegment = segmentationProcedure(_plane, _normal, _root->m_rightSegment, _root->m_rightFaces);
    }

    else
    {
     //Segment is null, so it will contain plane and new segments
        _root = new Segmentation();
        _root->m_plane = _plane;
        _root->m_visited = false;
        _root->m_leftFaces = new std::vector<GLuint>();
        _root->m_rightFaces = new std::vector<GLuint>();
        Vector3f planeCentre = calculateTriangleCentre(_plane[0], _plane[1], _plane[2]);

      if(_parentSideFaces)
        {
          //Parent exists
          //Side
          splitSegmentIntoSubsegments(_root, _parentSideFaces, planeCentre, _normal);


          //Clear parent
          _parentSideFaces->clear();
         }
      else
         {
           splitSegmentIntoSubsegments(_root, m_faceIndices, planeCentre, _normal);
         }
     }

    return _root;
}

void Mesh::createSegmentList()
{
  m_segments.clear();
  m_segmentsIndices->clear();


  std::vector<Segmentation*> stack;
  Segmentation* current;
  stack.push_back(m_segmentationRoot);


  while(stack.size() > 0)
  {
      current = stack.back();

      if(current->m_leftSegment != NULL && current->m_rightSegment != NULL)
      {
        if(!current->m_visited)
        {
          current->m_visited = true;
          stack.push_back(current->m_rightSegment);
          stack.push_back(current->m_leftSegment);
        }
        else
        {
          stack.pop_back();
        }
      }
      else
      {
          m_segments.push_back(current->m_leftFaces);
          m_segments.push_back(current->m_rightFaces);
          m_segmentationPlanes.push_back(current->m_plane);
          stack.pop_back();
      }
  }

  for(GLuint i=0; i<m_segments.size(); ++i)
  {
      std::vector<GLuint>* segment = m_segments[i];
      for(GLuint j=0; j<segment->size(); ++j)
      {
        m_segmentsIndices->push_back(segment->at(j));
      }
  }
}

void Mesh::destroySegments(Segmentation* _segmentation)
{
  if(_segmentation)
    {

      if(!_segmentation->m_leftSegment && !_segmentation->m_rightSegment)
            {
              delete [] _segmentation->m_leftFaces;
              delete [] _segmentation->m_rightFaces;
              delete _segmentation;
            }

            if (_segmentation->m_leftSegment)
            {
               destroySegments(_segmentation->m_leftSegment);
            }

            if (_segmentation->m_rightSegment)
            {
               destroySegments(_segmentation->m_rightSegment);
            }

            m_segments.clear();
            delete [] m_segmentsIndices;
     }
}


//Auxiliaries ------------------------
Vector3f Mesh::calculateTriangleCentre(int _p1, int _p2, int _p3)
{
   Vector3f point1;
   Vector3f point2;
   Vector3f point3;

   for(GLuint i=0; i<3; ++i)
   {
       point1[i] = m_vertices->at(3*_p1 + i);
       point2[i] = m_vertices->at(3*_p2 + i);
       point3[i] = m_vertices->at(3*_p3 + i);
   }

   return (point1 + point2 + point3)/3;
}

Vector3f Mesh::calculateActiveSegmentationPlaneCentre()
{
    Vector3i activePlane = getActivePlane();
    return calculateTriangleCentre(activePlane[0], activePlane[1], activePlane[2]);
}

Vector3f Mesh::calculateActiveSegmentationPlaneNormal()
{
    Vector3i activePlane = getActivePlane();
    Vector3f point1, point2, point3;

    for(GLuint i=0; i<3; ++i)
    {
      point1[i] = m_vertices->at(3*activePlane[0] + i);
      point2[i] = m_vertices->at(3*activePlane[1] + i);
      point3[i] = m_vertices->at(3*activePlane[2] + i);
    }

    return (point2 - point1).cross(point3 - point1);
}

float Mesh::euclideanDistance(Vector3f _v1, Vector3f _v2)
{
 float diff1 = _v1[0] - _v2[0];
 float diff2 = _v1[1] - _v2[1];
 float diff3 = _v1[2] - _v2[2];

 return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
}


void Mesh::printLandmarkedPoints(const char*_fileName)
{
  ofstream file;
  file.open(_fileName);
  GLuint size = m_landmarkVertexIndices->size();

  for(GLuint i=0; i<size; ++i)
  {
   file << m_landmarkVertexIndices->at(i);
   file << "\n";
  }

  file.flush();
  file.close();
}


//Setters and Getters ------------------------------------
void Mesh::setVertex(GLuint _vertNo, Vector3f _value)
{
    GLuint three_i = 3 * _vertNo;
    GLuint four_i = 4 * _vertNo;
    m_vertices->at(three_i) = _value[0];
    m_vertices->at(three_i + 1) = _value[1];
    m_vertices->at(three_i + 2) = _value[2];

    if(m_D)
    {
        m_D->coeffRef(_vertNo, four_i) = _value[0];
        m_D->coeffRef(_vertNo, four_i + 1) = _value[1];
        m_D->coeffRef(_vertNo, four_i + 2) = _value[2];
    }
}


Vector3f Mesh::getVertex(GLuint _vertNo)
{
    Vector3f vert(0.0, 0.0, 0.0);

    if(m_vertCount > _vertNo)
    {
     vert[0] = m_vertices->at(_vertNo * 3);
     vert[1] = m_vertices->at(_vertNo * 3 + 1);
     vert[2] = m_vertices->at(_vertNo * 3 + 2);
    }
    return vert;
}


Vector3f Mesh::getNormal(GLuint _vertNo)
{
    Vector3f normal(0.0, 0.0, 0.0);

    if(m_vertCount > _vertNo)
    {
        normal[0] = m_normals->at(_vertNo * 3);
        normal[1] = m_normals->at(_vertNo * 3 + 1);
        normal[2] = m_normals->at(_vertNo * 3 + 2);
    }
    return normal;
}








