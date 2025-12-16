#define _USE_MATH_DEFINES

#include "Mesh.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <exception>
#include <ios>
#include <string>
#include <memory>

#undef max
#undef min

Mesh::~Mesh()
{
  clear();
}

void Mesh::computeBoundingSphere(glm::vec3 &center, float &radius) const
{
  center = glm::vec3(0.0);
  radius = 0.f;
  for(const auto &p : _vertexPositions)
    center += p;
  center /= _vertexPositions.size();
  for(const auto &p : _vertexPositions)
    radius = std::max(radius, distance(center, p));
}

void Mesh::recomputePerVertexNormals(bool angleBased)
{
  _vertexNormals.clear();
  // Change the following code to compute a proper per-vertex normal
  _vertexNormals.resize(_vertexPositions.size(), glm::vec3(0.0, 0.0, 0.0));

  for(unsigned int tIt=0 ; tIt < _triangleIndices.size() ; ++tIt) {
    glm::uvec3 t = _triangleIndices[tIt];
    glm::vec3 n_t = glm::cross(
      _vertexPositions[t[1]] - _vertexPositions[t[0]],
      _vertexPositions[t[2]] - _vertexPositions[t[0]]);
    _vertexNormals[t[0]] += n_t;
    _vertexNormals[t[1]] += n_t;
    _vertexNormals[t[2]] += n_t;
  }
  for(unsigned int nIt = 0 ; nIt < _vertexNormals.size() ; ++nIt) {
    glm::normalize(_vertexNormals[nIt]);
  }
}

void Mesh::recomputePerVertexTextureCoordinates()
{
  _vertexTexCoords.clear();
  // Change the following code to compute a proper per-vertex texture coordinates
  _vertexTexCoords.resize(_vertexPositions.size(), glm::vec2(0.0, 0.0));

  float xMin = FLT_MAX, xMax = FLT_MIN;
  float yMin = FLT_MAX, yMax = FLT_MIN;
  for(glm::vec3 &p : _vertexPositions) {
    xMin = std::min(xMin, p[0]);
    xMax = std::max(xMax, p[0]);
    yMin = std::min(yMin, p[1]);
    yMax = std::max(yMax, p[1]);
  }
  for(unsigned int pIt = 0 ; pIt < _vertexTexCoords.size() ; ++pIt) {
    _vertexTexCoords[pIt] = glm::vec2(
      (_vertexPositions[pIt][0] - xMin)/(xMax-xMin),
      (_vertexPositions[pIt][1] - yMin)/(yMax-yMin));
  }
}

void Mesh::addPlan(float square_half_side)
{
  _vertexPositions.push_back(glm::vec3(-square_half_side,-square_half_side, 0));
  _vertexPositions.push_back(glm::vec3(+square_half_side,-square_half_side, 0));
  _vertexPositions.push_back(glm::vec3(+square_half_side,+square_half_side, 0));
  _vertexPositions.push_back(glm::vec3(-square_half_side,+square_half_side, 0));

  _vertexTexCoords.push_back(glm::vec2(0.0, 0.0));
  _vertexTexCoords.push_back(glm::vec2(1.0, 0.0));
  _vertexTexCoords.push_back(glm::vec2(1.0, 1.0));
  _vertexTexCoords.push_back(glm::vec2(0.0, 1.0));

  _vertexNormals.push_back(glm::vec3(0,0, 1));
  _vertexNormals.push_back(glm::vec3(0,0, 1));
  _vertexNormals.push_back(glm::vec3(0,0, 1));
  _vertexNormals.push_back(glm::vec3(0,0, 1));

  _triangleIndices.push_back(
    glm::uvec3(_vertexPositions.size()-4, _vertexPositions.size()-3, _vertexPositions.size()-2));
  _triangleIndices.push_back(
    glm::uvec3(_vertexPositions.size()-4, _vertexPositions.size()-2, _vertexPositions.size()-1));

  std::cout << "Vertices count: " << _vertexPositions.size() << std::endl;
  std::cout << "Triangles count: " << _triangleIndices.size() << std::endl;
  
  /* Add curvature and principal direction data (example values)
  _vertexCurvatures.push_back(glm::vec2(0.1f, 0.2f));
  _vertexCurvatures.push_back(glm::vec2(0.1f, 0.2f));
  _vertexCurvatures.push_back(glm::vec2(0.1f, 0.2f));
  _vertexCurvatures.push_back(glm::vec2(0.1f, 0.2f));
  
  _principalDir1.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
  _principalDir1.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
  _principalDir1.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
  _principalDir1.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
  
  _principalDir2.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
  _principalDir2.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
  _principalDir2.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
  _principalDir2.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); */
}

#ifdef SUPPORT_OPENGL_45
void Mesh::init()
{
  glCreateBuffers(1, &_posVbo); // Generate a GPU buffer to store the positions of the vertices
  size_t vertexBufferSize = sizeof(glm::vec3)*_vertexPositions.size(); // Gather the size of the buffer from the CPU-side vector
  glNamedBufferStorage(_posVbo, vertexBufferSize, _vertexPositions.data(), GL_DYNAMIC_STORAGE_BIT); // Create a data store on the GPU

  glCreateBuffers(1, &_normalVbo); // Same for normal
  glNamedBufferStorage(_normalVbo, vertexBufferSize, _vertexNormals.data(), GL_DYNAMIC_STORAGE_BIT);

  glCreateBuffers(1, &_texCoordVbo); // Same for texture coordinates
  size_t texCoordBufferSize = sizeof(glm::vec2)*_vertexTexCoords.size();
  glNamedBufferStorage(_texCoordVbo, texCoordBufferSize, _vertexTexCoords.data(), GL_DYNAMIC_STORAGE_BIT);

  glCreateBuffers(1, &_ibo); // Same for the index buffer, that stores the list of indices of the triangles forming the mesh
  size_t indexBufferSize = sizeof(glm::uvec3)*_triangleIndices.size();
  glNamedBufferStorage(_ibo, indexBufferSize, _triangleIndices.data(), GL_DYNAMIC_STORAGE_BIT);

  glCreateVertexArrays(1, &_vao); // Create a single handle that joins together attributes (vertex positions, normals) and connectivity (triangles indices)
  glBindVertexArray(_vao);

  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, _posVbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), 0);

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, _normalVbo);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), 0);

  glEnableVertexAttribArray(2);
  glBindBuffer(GL_ARRAY_BUFFER, _texCoordVbo);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2*sizeof(GLfloat), 0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ibo);
  glBindVertexArray(0); // Desactive the VAO just created. Will be activated at rendering time.
}
#else

void checkOpenGLError(const std::string& location) {
  GLenum error = glGetError();
  if (error != GL_NO_ERROR) {
      std::cerr << "OpenGL error at " << location << ": " << error << std::endl;
  }
}

void Mesh::init()
{
  // Generate a GPU buffer to store the positions of the vertices
  size_t vertexBufferSize = sizeof(glm::vec3)*_vertexPositions.size();
  glGenBuffers(1, &_posVbo);
  glBindBuffer(GL_ARRAY_BUFFER, _posVbo);
  glBufferData(GL_ARRAY_BUFFER, vertexBufferSize, _vertexPositions.data(), GL_DYNAMIC_READ);

  // Same for normal
  glGenBuffers(1, &_normalVbo);
  glBindBuffer(GL_ARRAY_BUFFER, _normalVbo);
  glBufferData(GL_ARRAY_BUFFER, vertexBufferSize, _vertexNormals.data(), GL_DYNAMIC_READ);

  // Same for texture coordinates
  size_t texCoordBufferSize = sizeof(glm::vec2)*_vertexTexCoords.size();
  glGenBuffers(1, &_texCoordVbo);
  glBindBuffer(GL_ARRAY_BUFFER, _texCoordVbo);
  glBufferData(GL_ARRAY_BUFFER, texCoordBufferSize, _vertexTexCoords.data(), GL_DYNAMIC_READ);

  // Same for the index buffer that stores the list of indices of the triangles forming the mesh
  size_t indexBufferSize = sizeof(glm::uvec3)*_triangleIndices.size();
  glGenBuffers(1, &_ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ibo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSize, _triangleIndices.data(), GL_DYNAMIC_READ);

  /*-------------------------------------------------
  glGenBuffers(1, &_curvatureVbo);
  glBindBuffer(GL_ARRAY_BUFFER, _curvatureVbo);
  glBufferData(GL_ARRAY_BUFFER, _vertexCurvatures.size() * sizeof(glm::vec2), _vertexCurvatures.data(), GL_STATIC_DRAW);
  
  // Upload principal direction 1 (κ1 direction)
  glGenBuffers(1, &_pDir1Vbo);
  glBindBuffer(GL_ARRAY_BUFFER, _pDir1Vbo);
  glBufferData(GL_ARRAY_BUFFER, _principalDir1.size() * sizeof(glm::vec3), _principalDir1.data(), GL_STATIC_DRAW);
  
  // Upload principal direction 2 (κ2 direction)
  glGenBuffers(1, &_pDir2Vbo);
  glBindBuffer(GL_ARRAY_BUFFER, _pDir2Vbo);
  glBufferData(GL_ARRAY_BUFFER, _principalDir2.size() * sizeof(glm::vec3), _principalDir2.data(), GL_STATIC_DRAW);*/

  //-------------------------------------------------------------------

  // Create a single handle that joins together attributes (vertex positions, normals) and connectivity (triangles indices)
  glGenVertexArrays(1, &_vao);
  glBindVertexArray(_vao);
  checkOpenGLError("VAO generation");

  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, _posVbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), 0);
  checkOpenGLError("VBO generation");

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, _normalVbo);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), 0);
  checkOpenGLError("Noemal generation");

  glEnableVertexAttribArray(2);
  glBindBuffer(GL_ARRAY_BUFFER, _texCoordVbo);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2*sizeof(GLfloat), 0);
  checkOpenGLError("Text generation");

  /*------------------------------------------------------------------

  glEnableVertexAttribArray(3);
  glBindBuffer(GL_ARRAY_BUFFER, _curvatureVbo);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, (void*)0); 

  glEnableVertexAttribArray(4);
  glBindBuffer(GL_ARRAY_BUFFER, _pDir1Vbo);
  glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

  glEnableVertexAttribArray(5);
  glBindBuffer(GL_ARRAY_BUFFER, _pDir2Vbo);
  glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 0, (void*)0); */

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ibo);

  glBindVertexArray(0); // Desactive the VAO just created. Will be activated at rendering time.
}
#endif

void Mesh::render()
{
    if (_triangleIndices.empty()) {
        std::cerr << "[Error] No triangles to render!" << std::endl;
        return;
    }

    std::cout << " > Rendering mesh with " << _triangleIndices.size() << " triangles" << std::endl;

    glBindVertexArray(_vao);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(_triangleIndices.size() * 3), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Mesh::clear()
{
  _vertexPositions.clear();
  _vertexNormals.clear();
  _vertexTexCoords.clear();
  _triangleIndices.clear();
  if(_vao) {
    glDeleteVertexArrays(1, &_vao);
    _vao = 0;
  }
  if(_posVbo) {
    glDeleteBuffers(1, &_posVbo);
    _posVbo = 0;
  }
  if(_normalVbo) {
    glDeleteBuffers(1, &_normalVbo);
    _normalVbo = 0;
  }
  if(_texCoordVbo) {
    glDeleteBuffers(1, &_texCoordVbo);
    _texCoordVbo = 0;
  }
  if(_ibo) {
    glDeleteBuffers(1, &_ibo);
    _ibo = 0;
  }

  /*if(_curvatureVbo) {
    glDeleteBuffers(1, &_curvatureVbo);
    _curvatureVbo = 0;
  }
  if(_pDir1Vbo) {
    glDeleteBuffers(1, &_pDir1Vbo);
    _pDir1Vbo = 0;
  }
  if(_pDir2Vbo) {
    glDeleteBuffers(1, &_pDir2Vbo);
    _pDir2Vbo = 0;
  }*/
}

// Loads an OFF mesh file. See https://en.wikipedia.org/wiki/OFF_(file_format)
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr) {
  std::cout << " > Start loading mesh <" << filename << ">" << std::endl;
  meshPtr->clear();
  std::ifstream in(filename.c_str());
  if (!in) {
      throw std::ios_base::failure("[Mesh Loader][loadOFF] Cannot open " + filename);
  }

  std::string offString;
  unsigned int sizeV, sizeT, tmp;
  in >> offString >> sizeV >> sizeT >> tmp;

  if (offString != "OFF") {
      throw std::runtime_error("[Mesh Loader][loadOFF] Invalid OFF file format");
  }

  std::cout << " > OFF file format confirmed" << std::endl;
  std::cout << " > Number of vertices: " << sizeV << std::endl;
  std::cout << " > Number of triangles: " << sizeT << std::endl;

  auto &P = meshPtr->vertexPositions();
  auto &T = meshPtr->triangleIndices();
  P.resize(sizeV);
  T.resize(sizeT);
  size_t tracker = (sizeV + sizeT) / 20;

  std::cout << " > [" << std::flush;
  for (unsigned int i = 0; i < sizeV; ++i) {
      if (i % tracker == 0) {
          std::cout << "-" << std::flush;
      }
      in >> P[i][0] >> P[i][1] >> P[i][2];
      std::cout << "Vertex " << i << ": (" << P[i][0] << ", " << P[i][1] << ", " << P[i][2] << ")" << std::endl;
  }

  int s;
  for (unsigned int i = 0; i < sizeT; ++i) {
      if ((sizeV + i) % tracker == 0) {
          std::cout << "-" << std::flush;
      }
      in >> s;
      if (s != 3) {
          throw std::runtime_error("[Mesh Loader][loadOFF] Non-triangular face encountered");
      }
      for (unsigned int j = 0; j < 3; ++j) {
          in >> T[i][j];
          std::cout << "Triangle " << i << ", Vertex " << j << ": " << T[i][j] << std::endl;
      }
  }
  std::cout << "]" << std::endl;
  in.close();

  meshPtr->vertexNormals().resize(P.size(), glm::vec3(0.f, 0.f, 1.f));
  meshPtr->vertexTexCoords().resize(P.size(), glm::vec2(0.f, 0.f));
  meshPtr->recomputePerVertexNormals();
  meshPtr->recomputePerVertexTextureCoordinates();
  meshPtr->computeNeighboringVertices();

  std::cout << " > Mesh <" << filename << "> loaded" << std::endl;
}
