#include "mesh.h"
#include <math.h>
#include <boost/tokenizer.hpp>
#define EPSILON 0.000001

Mesh::Mesh()
{
    m_vertices = new std::vector<GLfloat>();
    m_vertNormalLines = new std::vector<GLfloat>();
    m_normals = new std::vector<GLfloat>();
    m_texcoords = new std::vector<GLfloat>();
    m_faceIndices = new std::vector<GLuint>();
    m_adjMat = NULL;
    m_D = NULL;
    m_position.v[0] = 0.0;
    m_position.v[1] = 0.0;
    m_position.v[2] = 0.0;
    m_vertCount = 0;
    m_edgeCount = 0;
    m_faceCount = 0;
    m_texCoordCount = 0;
    m_landmarkVertexIndices = new std::vector<int>();
    m_pickedIndex = -1;
}

Mesh::~Mesh()
{
 if(m_vertices)
 {
   delete [] m_vertices;
 }

 if(m_vertNormalLines)
 {
   delete [] m_vertNormalLines;
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
   delete m_adjMat;
 }

 if(m_D)
 {
   delete m_D;
 }

 if(m_landmarkVertexIndices)
 {
   delete m_landmarkVertexIndices;
 }
}

bool Mesh::loadMesh(const char *_fileName, float* _transformations)
{    
    ifstream in(_fileName, ios::in);
    if (!in)
    {
     printf("Cannot open %s",_fileName);
     return false;
    }

    m_position.v[0] = 0.0;
    m_position.v[1] = 0.0;
    m_position.v[2] = 0.0;

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
           m_position.v[0] += (GLfloat)vert.v[0];
           m_position.v[1] += (GLfloat)vert.v[1];
           m_position.v[2] += (GLfloat)vert.v[2];

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
*/
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
    m_position = m_position/m_vertCount;

    for(unsigned int i=0; i<m_vertCount; i++)
    {
     m_vertices->at(3*i) = m_vertices->at(3*i) - m_position.v[0];
     m_vertices->at(3*i+1) = m_vertices->at(3*i+1) - m_position.v[1];
     m_vertices->at(3*i+2) = m_vertices->at(3*i+2) - m_position.v[2];
    }

    moveObject(_transformations[3], _transformations[4], _transformations[5]);
    calculateNormals();
    // buildVertexNormalVector();

    return true;
}

void Mesh::printLandmarkedPoints(const char*_fileName)
{
  ofstream file;
  file.open(_fileName);
  int size = m_landmarkVertexIndices->size();

  for(unsigned int i=0; i<size; ++i)
  {
   file << m_landmarkVertexIndices->at(i);
   file << "\n";
  }

  file.flush();
  file.close();
}

void Mesh::calculateNormals()
{
  if(m_normals->size()<1)
  {
      for(unsigned int k=0; k<m_vertices->size(); ++k)
      {
       m_normals->push_back(0.0);
      }
  }
  else
  {
    for(unsigned int k=0; k<m_normals->size(); ++k)
    {
     m_normals->at(k) = 0.0;
    }
  }

  unsigned int three_i;
  unsigned short v1;
  unsigned short v2;
  unsigned short v3;
  unsigned int three_v1;
  unsigned int three_v2;
  unsigned int three_v3;

  for (unsigned int i = 0; i < m_faceCount; ++i)
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
    unsigned int three_i;
    for(unsigned int i = 0; i < m_vertCount; i++)
    {
        float magnitude = 0.0;
        three_i = 3 * i;
        float x = m_normals->at(three_i);
        float y = m_normals->at(three_i + 1);
        float z = m_normals->at(three_i + 2);
        magnitude = sqrt(x * x + y * y + z * z);

        if(abs(magnitude) > 1.0)
        {
         m_normals->at(three_i) /= magnitude;
         m_normals->at(three_i + 1) /= magnitude;
         m_normals->at(three_i + 2) /= magnitude;
        }
    }
}

void Mesh::buildVertexNormalVector()
 {
     if(m_vertNormalLines->size()<1)
     {
         for(unsigned int k=0; k < m_vertCount * 6; ++k)
         {
           m_vertNormalLines->push_back(0.0);
         }
     }
     else
     {
       for(unsigned int k=0; k<m_vertNormalLines->size(); ++k)
       {
         m_vertNormalLines->at(k) = 0.0;
       }
     }

     unsigned int three_i;
     float alpha = 1.01;
     float beta = 0.5;

     for (unsigned int i=0; i < m_vertCount; ++i)
     {
         three_i = 3*i;
         m_vertNormalLines->at(three_i) = alpha * m_vertices->at(three_i);
         m_vertNormalLines->at(three_i + 1) = alpha * m_vertices->at(three_i + 1);
         m_vertNormalLines->at(three_i + 2) = alpha * m_vertices->at(three_i + 2);
         m_vertNormalLines->at(three_i + 3) = m_vertices->at(three_i) + beta * m_normals->at(three_i);
         m_vertNormalLines->at(three_i + 4) = m_vertices->at(three_i + 1) + beta * m_normals->at(three_i + 1);
         m_vertNormalLines->at(three_i + 5) = m_vertices->at(three_i + 2) + beta * m_normals->at(three_i + 2);
     }
 }

void Mesh::normaliseMesh()
{
 //Find min and max for x, y, z
 float min_x = 1000.0;
 float min_y = 1000.0;
 float min_z = 1000.0;
 float max_x = -1000.0;
 float max_y = -1000.0;
 float max_z = -1000.0;
 float aux;
 unsigned int three_i;

 for (unsigned int i = 0; i < m_vertCount; ++i)
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

    for (unsigned int i = 0; i < m_vertCount; ++i)
    {
        three_i = 3*i;
        float x  = m_vertices->at(three_i);
        float y = m_vertices->at(three_i + 1);
        float z = m_vertices->at(three_i + 2) ;

        m_vertices->at(three_i) = 2.0 * (x - min_x) / (max_x - min_x) - 1.0;
        m_vertices->at(three_i + 1) = 2.0 * (y - min_y) / (max_y - min_y) - 1.0;
        m_vertices->at(three_i + 2) = 2.0 * (z - min_z) / (max_z - min_z) - 1.0;
    }
}

void Mesh::bindVAO1()
{
    m_vao1 = 0;
    glGenVertexArrays(1, &m_vao1);
    glBindVertexArray(m_vao1);

 //Copy mesh data into VBO ======================================================================
    if(m_vertCount > 0)
    {
        glGenBuffers(1, &m_vbo1Position);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1Position);
        glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount * sizeof(GLfloat), &m_vertices->at(0), GL_STATIC_DRAW);

        glGenBuffers(1, &m_vbo1Indices);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1Indices);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * m_faceCount*sizeof(GLuint), &m_faceIndices->at(0), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL); //0 corresponds to vertex positions in VAO
        glEnableVertexAttribArray(0);
    }

    if(m_normals->size() > 0)
    {
        glGenBuffers(1, &m_vbo1Normals);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1Normals);
        glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount  * sizeof(GLfloat), &m_normals->at(0), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL); //1 corresponds to normals in VAO
        glEnableVertexAttribArray(1);
    }

    if(m_vertCount > 0 && m_normals->size() > 0)
    {
        glGenBuffers(1, &m_vbo2Position);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2Position);
        glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount * sizeof(GLfloat), &m_vertices->at(0), GL_STATIC_DRAW);
    }

//Obs! 14 coordinates instead of 8
/*
    if(m_texCoordCount > 0)
    {
        glGenBuffers(1, &m_vboTextureCoord);
        glBindBuffer(GL_ARRAY_BUFFER, m_vboTextureCoord);
        glBufferData(GL_ARRAY_BUFFER, 2 * m_texCoordCount * sizeof(GLfloat), &m_texcoords->at(0), GL_STATIC_DRAW);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, NULL); //2 corresponds to texture coordinates
        glEnableVertexAttribArray(2);
    }
*/
}

void Mesh::bindVAO2()
{
    unsigned int size = m_vertNormalLines->size();

    m_vao2 = 0;
    glGenVertexArrays(1, &m_vao2);
    glBindVertexArray(m_vao2);

 //Copy mesh data into VBO ======================================================================
    if(size > 0)
    {
        glGenBuffers(1, &m_vbo2Position);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2Position);
        glBufferData(GL_ARRAY_BUFFER, size * sizeof(GLfloat), &m_vertNormalLines->at(0), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL); //0 corresponds to vertex positions in VAO
        glEnableVertexAttribArray(0);
    }
}

void Mesh::buildArcNodeMatrix()
{
  //Create the arc-node adjacency matrix

    if(!m_adjMat)
    {
     m_adjMat = new std::map<std::pair<unsigned int, unsigned int>, short >();
     std::map <std::pair<unsigned int, unsigned int>, short >::iterator it;
     unsigned int three_i;
     unsigned int v1;
     unsigned int v2;
     unsigned int v3;
     unsigned int min;
     unsigned int max;

     for (unsigned int i = 0; i < m_faceCount; ++i)
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
            m_adjMat->insert(make_pair(std::pair<unsigned int, unsigned int>(min, max), 0));
            m_edgeCount++;
        }

        min = v2 < v3? v2 : v3;
        max = v2 > v3? v2 : v3;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(std::pair<unsigned int, unsigned int>(min, max), 0));
            m_edgeCount++;
        }

        min = v1 < v3 ? v1 : v3;
        max = v1 > v3 ? v1 : v3;
        it = m_adjMat->find(make_pair(min, max));
        if(it == m_adjMat->end())
        {
            m_adjMat->insert(make_pair(std::pair<unsigned int, unsigned int>(min, max), 0));
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

  unsigned int three_i = 0;
  unsigned int four_i = 0;

  for(unsigned int i = 0; i < m_vertCount; i++)
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

Vector3f Mesh::getNormal(unsigned int _vertNo)
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

Vector3f Mesh::getVertex(unsigned int _vertNo)
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

void Mesh::rotateObject(float _angleX, float _angleY, float _angleZ)
{
  int three_i;
  mat4 Rx = rotate_x_deg(identity_mat4(), _angleX);
  mat4 Ry= rotate_y_deg(identity_mat4(), _angleY);
  mat4 Rz = rotate_z_deg(identity_mat4(), _angleZ);
  vec4 vertex;
  vertex.v[3] = 1.0;

  for(unsigned int i=0; i<m_vertCount; ++i)
  {
     three_i = 3*i;
     vertex.v[0] = m_vertices->at(three_i);
     vertex.v[1] = m_vertices->at(three_i + 1);
     vertex.v[2] = m_vertices->at(three_i + 2);

     vertex = Rz * Ry * Rx * vertex;

     m_vertices->at(three_i) = vertex.v[0];
     m_vertices->at(three_i + 1) = vertex.v[1];
     m_vertices->at(three_i + 2) = vertex.v[2];
  }
}

void Mesh::moveObject(float _tX, float _tY, float _tZ)
{
  int three_i;
  mat4 T = translate(identity_mat4(), vec3(_tX, _tY, _tZ));
  vec4 vertex;
  vertex.v[3] = 1.0;

  for(unsigned int i=0; i<m_vertCount; ++i)
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
}

int Mesh::whereIsIntersectingMesh(bool _culling, int _originTemplateIndex, Vector3f _origin, Vector3f _ray)
{
 ///@ref Fast, Minimum Storage Ray/Triangle Intersection, Möller & Trumbore. Journal of Graphics Tools, 1997.
    Vector3f point1, point2, point3;
    Vector3f edge1, edge2;
    Vector3f P, Q, T;
    float det, inv_det, u, v;
    float t = 0.0f;
    unsigned int i = 0;
    unsigned int three_i;
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
            int imin;

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

void Mesh::affineTransformation(MatrixXf _X)
{
    unsigned int three_i;
    Vector3f vertex;

    for(unsigned int i=0; i < m_vertCount; ++i)
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

}

float Mesh::euclideanDistance(Vector3f _v1, Vector3f _v2)
{
 float diff1 = _v1[0] - _v2[0];
 float diff2 = _v1[1] - _v2[1];
 float diff3 = _v1[2] - _v2[2];

 return sqrt(diff1 * diff1 + diff2 * diff2 + diff3 * diff3);
}
