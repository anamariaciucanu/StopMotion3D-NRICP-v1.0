#include "mesh.h"
#include <math.h>
#include <boost/tokenizer.hpp>


Mesh::Mesh()
{
    m_vertices = new std::vector<GLfloat>();
    m_normals = new std::vector<GLfloat>();
    m_texcoords = new std::vector<GLfloat>();
    m_faceIndices = new std::vector<GLuint>();
    m_M = NULL;
    m_D = NULL;
    m_position.v[0] = 0.0;
    m_position.v[1] = 0.0;
    m_position.v[2] = 0.0;
    m_vertCount = 0;
    m_edgeCount = 0;
    m_faceCount = 0;
    m_texCoordCount = 0;
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

 if(m_M)
 {
     delete m_M;
 }

 if(m_D)
 {
     delete m_D;
 }
}

bool Mesh::loadMesh(const char *_fileName)
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

    return true;
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

   normal = (point2 - point1).cross(point3 - point2);

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

      if(abs(magnitude) > 1.5)
      {
        m_normals->at(three_i) /= magnitude;
        m_normals->at(three_i + 1) /= magnitude;
        m_normals->at(three_i + 2) /= magnitude;
      }
    }
}


void Mesh::bindVAO()
{
    m_vao = 0;
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);

 //Copy mesh data into VBO ======================================================================
    if(m_vertCount > 0)
    {
        glGenBuffers(1, &m_vboPosition);
        glBindBuffer(GL_ARRAY_BUFFER, m_vboPosition);
        glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount * sizeof(GLfloat), &m_vertices->at(0), GL_STATIC_DRAW);

        glGenBuffers(1, &m_vboIndices);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vboIndices);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3*m_faceCount*sizeof(GLuint), &m_faceIndices->at(0), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL); //0 corresponds to vertex positions in VAO
        glEnableVertexAttribArray(0);
    }

    if(m_normals->size() > 0)
    {   glGenBuffers(1, &m_vboNormals);
        glBindBuffer(GL_ARRAY_BUFFER, m_vboNormals);
        glBufferData(GL_ARRAY_BUFFER, 3 * m_vertCount  * sizeof(GLfloat), &m_normals->at(0), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL); //1 corresponds to normals in VAO
        glEnableVertexAttribArray(1);
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

void Mesh::buildArcNodeMatrix()
{
  //Create the arc-node adjacency matrix

    std::map <std::pair<unsigned int, unsigned int>, int > adjMap;
    std::map <std::pair<unsigned int, unsigned int>, int >::iterator it;
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

        min = v1 < v2? v1:v2;
        max = v1 > v2? v1:v2;
        it = adjMap.find(make_pair(min, max));
        if(it == adjMap.end())
        {
            adjMap.insert(make_pair(std::pair<unsigned int, unsigned int>(min, max), 0));
            m_edgeCount++;
        }

        min = v2 < v3? v2:v3;
        max = v2 > v3? v2:v3;
        it = adjMap.find(make_pair(min, max));
        if(it == adjMap.end())
        {
            adjMap.insert(make_pair(std::pair<unsigned int, unsigned int>(min, max), 0));
            m_edgeCount++;
        }

        min = v1 < v3 ? v1 : v3;
        max = v1 > v3 ? v1 : v3;
        it = adjMap.find(make_pair(min, max));
        if(it == adjMap.end())
        {
            adjMap.insert(make_pair(std::pair<unsigned int, unsigned int>(min, max), 0));
            m_edgeCount++;
        }
    }

    m_M = new SparseMatrix<int>(m_edgeCount, m_vertCount);
    m_M->reserve(m_edgeCount * 2);
    unsigned int k = 0;

    for(it = adjMap.begin(); it != adjMap.end(); ++it)
    {

        v1 = it->first.first;
        v2 = it->first.second;

         m_M->coeffRef(k, v1) = -1;
         m_M->coeffRef(k, v2) = 1;
    }
    m_M->makeCompressed();
}

void Mesh::buildVertexMatrix()
{
  //TO DO: Release memory
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

void Mesh::changeVertsBasedOn_D_Matrix()
{    
    if(m_D)
    {
     unsigned int three_i = 0;
     unsigned int four_i = 0;

     for (unsigned int i=0; i < m_vertCount;i++)
     {
       three_i = 3 * i;
       four_i = 4 * i;
       m_vertices->at(three_i) = m_D->coeff(i, four_i);
       m_vertices->at(three_i+ 1) = m_D->coeff(i, four_i + 1);
       m_vertices->at(three_i + 2) = m_D->coeff(i, four_i + 2);
     }
    }
}

Vector3f Mesh::getNormal(unsigned int _vertNo)
{
    Vector3f normal(0.0, 0.0, 0.0);

    if(m_vertCount >= _vertNo)
    {
        normal[0] = m_normals->at(_vertNo * 3);
        normal[1] = m_normals->at(_vertNo * 3 + 1);
        normal[2] = m_normals->at(_vertNo * 3 + 2);
    }
    return normal;
}
