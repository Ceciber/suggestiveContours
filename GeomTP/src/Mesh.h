#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <vector>
#include <memory>
#include <string>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <map>
#include <set>

#include <cmath>

class Mesh {
public:
  virtual ~Mesh();

  const std::vector<glm::vec3> &vertexPositions() const { return _vertexPositions; }
  std::vector<glm::vec3> &vertexPositions() { return _vertexPositions; }

  const std::vector<glm::vec3> &vertexNormals() const { return _vertexNormals; }
  std::vector<glm::vec3> &vertexNormals() { return _vertexNormals; }

  const std::vector<glm::vec2> &vertexTexCoords() const { return _vertexTexCoords; }
  std::vector<glm::vec2> &vertexTexCoords() { return _vertexTexCoords; }

  const std::vector<glm::uvec3> &triangleIndices() const { return _triangleIndices; }
  std::vector<glm::uvec3> &triangleIndices() { return _triangleIndices; }

  /// Compute the parameters of a sphere which bounds the mesh
  void computeBoundingSphere(glm::vec3 &center, float &radius) const;

  void recomputePerVertexNormals(bool angleBased = false);
  void recomputePerVertexTextureCoordinates();

  void init();
  void initOldGL();
  void render();
  void clear();

  void addPlan(float square_half_side = 1.0f);

    void subdivideLoop() {
    std::vector<glm::vec3> newVertices( _vertexPositions.size() , glm::vec3(0,0,0) );
    std::vector<glm::uvec3> newTriangles;

    struct Edge {
      unsigned int a , b;
      Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
      bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
      bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
    };

    std::map< Edge , unsigned int > newVertexOnEdge; // this will be useful to find out whether we already inserted an odd vertex or not
    std::map< Edge , std::set< unsigned int > > trianglesOnEdge; // this will be useful to find out if an edge is boundary or not
    std::vector< std::set< unsigned int > > neighboringVertices( _vertexPositions.size() ); // this will be used to store the adjacent vertices, i.e., neighboringVertices[i] will be the list of vertices that are adjacent to vertex i.
    std::vector< bool > evenVertexIsBoundary( _vertexPositions.size() , false );

    for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
      unsigned int a = _triangleIndices[tIt][0];
      unsigned int b = _triangleIndices[tIt][1];
      unsigned int c = _triangleIndices[tIt][2];

      Edge ab(a, b);
      Edge bc(b, c);
      Edge ac(a, c);

      trianglesOnEdge[ab].insert(tIt);
      trianglesOnEdge[bc].insert(tIt);
      trianglesOnEdge[ac].insert(tIt);

      neighboringVertices[ a ].insert( b );
      neighboringVertices[ a ].insert( c );
      neighboringVertices[ b ].insert( a );
      neighboringVertices[ b ].insert( c );
      neighboringVertices[ c ].insert( a );
      neighboringVertices[ c ].insert( b );
    }

    std::vector< unsigned int > evenVertexValence( _vertexPositions.size() , 0 );
    for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
      evenVertexValence[ v ] = neighboringVertices[ v ].size();
    }

    for (const auto& [edge, triangles] : trianglesOnEdge) { 
        if (triangles.size() == 1) { 
            evenVertexIsBoundary[edge.a] = true;
            evenVertexIsBoundary[edge.b] = true;
        }
    }
    
    for(unsigned int v = 0 ; v < _vertexPositions.size() ; ++v) {
      float alpha_n;
      glm::vec3 sum(0.0f, 0.0f, 0.0f); 

      if(evenVertexIsBoundary[v]){
      for (unsigned int neighbor : neighboringVertices[v]) {
          if(evenVertexIsBoundary[neighbor]){
            sum += _vertexPositions[neighbor];
         }
      }

      newVertices[v] = (6.0f / 8.0f) *_vertexPositions[v] + (1.0f / 8.0f )* sum;

      }
      else{
        int valence = evenVertexValence[v];
        if(valence == 3){
          alpha_n = 3.0f / 16.0f;
        }
        else{
          alpha_n = 3.0f / (8.0f*valence);
        }
        for (unsigned int neighbor : neighboringVertices[v]) {
            sum += _vertexPositions[neighbor];
        }
        newVertices[v] = (1-valence*alpha_n)*_vertexPositions[v] + alpha_n*sum;
      }
    }

    for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
      unsigned int a = _triangleIndices[tIt][0];
      unsigned int b = _triangleIndices[tIt][1];
      unsigned int c = _triangleIndices[tIt][2];


      Edge Eab(a,b);
      unsigned int oddVertexOnEdgeEab = 0;
      if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
        newVertices.push_back( glm::vec3(0,0,0) );
        oddVertexOnEdgeEab = newVertices.size() - 1;
        newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
      }
      else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }

      if (trianglesOnEdge[Eab].size() == 1) {
          newVertices[oddVertexOnEdgeEab] = (_vertexPositions[a] + _vertexPositions[b]) / 2.0f;
      }
      else {
          glm::vec3 sum_ab = _vertexPositions[a] + _vertexPositions[b];
          glm::vec3 sum_jk(0.0f, 0.0f, 0.0f);

          for (unsigned int triangleIndex : trianglesOnEdge[Eab]) {
              const glm::uvec3 &triangle = _triangleIndices[triangleIndex];
              for (unsigned int vertex : {triangle[0], triangle[1], triangle[2]}) {
                  if (vertex != a && vertex != b) {
                      sum_jk += _vertexPositions[vertex];
                      break;
                  }
              }
          }
          newVertices[oddVertexOnEdgeEab] = (3.0f / 8.0f) * sum_ab + (1.0f / 8.0f) * sum_jk;
      }
      Edge Ebc(b,c);
      unsigned int oddVertexOnEdgeEbc = 0;
      if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
        newVertices.push_back( glm::vec3(0,0,0) );
        oddVertexOnEdgeEbc = newVertices.size() - 1;
        newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
      }
      else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }
      if (trianglesOnEdge[Ebc].size() == 1) {
          newVertices[oddVertexOnEdgeEbc] = (_vertexPositions[b] + _vertexPositions[c]) / 2.0f;
      }
      else {
          glm::vec3 sum_bc = _vertexPositions[b] + _vertexPositions[c];
          glm::vec3 sum_pq(0.0f, 0.0f, 0.0f);

          for (unsigned int triangleIndex : trianglesOnEdge[Ebc]) {
              const glm::uvec3 &triangle = _triangleIndices[triangleIndex];
              for (unsigned int vertex : {triangle[0], triangle[1], triangle[2]}) {
                  if (vertex != b && vertex != c) {
                      sum_pq += _vertexPositions[vertex];
                      break;
                  }
              }
          }
          newVertices[oddVertexOnEdgeEbc] = (3.0f / 8.0f) * sum_bc + (1.0f / 8.0f) * sum_pq;
      }


      Edge Eca(c,a);
      unsigned int oddVertexOnEdgeEca = 0;
      if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
        newVertices.push_back( glm::vec3(0,0,0) );
        oddVertexOnEdgeEca = newVertices.size() - 1;
        newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
      }
      else { oddVertexOnEdgeEca = newVertexOnEdge[Eca]; }
      if (trianglesOnEdge[Eca].size() == 1) {
          newVertices[oddVertexOnEdgeEca] = (_vertexPositions[c] + _vertexPositions[a]) / 2.0f;
      }
      else {
          glm::vec3 sum_ac = _vertexPositions[a] + _vertexPositions[c];
          glm::vec3 sum_de(0.0f, 0.0f, 0.0f);

          for (unsigned int triangleIndex : trianglesOnEdge[Eca]) {
              const glm::uvec3 &triangle = _triangleIndices[triangleIndex];
              for (unsigned int vertex : {triangle[0], triangle[1], triangle[2]}) {
                  if (vertex != a && vertex != c) {
                      sum_de += _vertexPositions[vertex];
                      break;
                  }
              }
          }
          newVertices[oddVertexOnEdgeEca] = (3.0f / 8.0f) * sum_ac + (1.0f / 8.0f) * sum_de;
      }

      newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
    }

    _triangleIndices = newTriangles;
    _vertexPositions = newVertices;
    recomputePerVertexNormals( );
    recomputePerVertexTextureCoordinates( );
  }


  void Mesh::computeVertexCurvatures() {
    // Clear previous results
    _vertexCurvatures.resize(_vertexPositions.size(), glm::vec2(0.0f));
    _principalDir1.resize(_vertexPositions.size(), glm::vec3(0.0f));
    _principalDir2.resize(_vertexPositions.size(), glm::vec3(0.0f));

    // Step 1: Initialize containers for curvature tensors
    std::vector<glm::mat2x2> curvatureTensors(_vertexPositions.size(), glm::mat2x2(0.0f));
    std::vector<float> vertexWeights(_vertexPositions.size(), 0.0f); // For weighted averaging

    // Step 2: Iterate over all triangles
    for (const auto& tri : _triangleIndices) {
        // Get the positions and normals of the triangle vertices
        glm::vec3 v0 = _vertexPositions[tri.x];
        glm::vec3 v1 = _vertexPositions[tri.y];
        glm::vec3 v2 = _vertexPositions[tri.z];

        glm::vec3 n0 = _vertexNormals[tri.x];
        glm::vec3 n1 = _vertexNormals[tri.y];
        glm::vec3 n2 = _vertexNormals[tri.z];

        // Edges of the triangle
        glm::vec3 e0 = v1 - v0;
        glm::vec3 e1 = v2 - v1;
        glm::vec3 e2 = v0 - v2;

        // Differences in normals along edges
        glm::vec3 dn0 = n1 - n0;
        glm::vec3 dn1 = n2 - n1;
        glm::vec3 dn2 = n0 - n2;

        // Approximate curvature tensor for each vertex of the triangle
        auto computeTensor = [](const glm::vec3& e, const glm::vec3& dn) -> glm::mat2x2 {
            float lenSq = glm::dot(e, e);
            if (lenSq == 0.0f) return glm::mat2x2(0.0f); // Avoid division by zero
            glm::vec2 projE = glm::vec2(glm::dot(e, glm::vec3(1, 0, 0)), glm::dot(e, glm::vec3(0, 1, 0)));
            glm::vec2 projDn = glm::vec2(glm::dot(dn, glm::vec3(1, 0, 0)), glm::dot(dn, glm::vec3(0, 1, 0)));
            return glm::outerProduct(projDn / lenSq, projE);
        };

        glm::mat2x2 t0 = computeTensor(e0, dn0);
        glm::mat2x2 t1 = computeTensor(e1, dn1);
        glm::mat2x2 t2 = computeTensor(e2, dn2);

        // Step 3: Aggregate tensors for each vertex
        curvatureTensors[tri.x] += t0;
        curvatureTensors[tri.y] += t1;
        curvatureTensors[tri.z] += t2;

        // Use triangle area as weight
        float area = 0.5f * glm::length(glm::cross(e0, e2));
        vertexWeights[tri.x] += area;
        vertexWeights[tri.y] += area;
        vertexWeights[tri.z] += area;
    }

    // Step 4: Normalize tensors and compute eigenvalues/vectors
    for (size_t i = 0; i < _vertexPositions.size(); ++i) {
        if (vertexWeights[i] > 0.0f) {
            curvatureTensors[i] /= vertexWeights[i]; // Average tensor
        }

        // Perform eigen decomposition to get principal curvatures and directions
        glm::mat2x2& tensor = curvatureTensors[i];
        glm::vec2 eigenvalues;
        glm::mat2 eigenvectors;

        computeEigenDecomposition(tensor, eigenvalues, eigenvectors);

        _vertexCurvatures[i] = eigenvalues; // Principal curvatures (κ1, κ2)

        // Map principal directions back to 3D (assuming eigenvectors are in tangent plane)
        glm::vec3 tangentX = glm::normalize(glm::cross(_vertexNormals[i], glm::vec3(1, 0, 0)));
        if (glm::length(tangentX) < 1e-6f) {
            tangentX = glm::normalize(glm::cross(_vertexNormals[i], glm::vec3(0, 1, 0)));
        }
        glm::vec3 tangentY = glm::normalize(glm::cross(_vertexNormals[i], tangentX));

        _principalDir1[i] = tangentX * eigenvectors[0][0] + tangentY * eigenvectors[1][0];
        _principalDir2[i] = tangentX * eigenvectors[0][1] + tangentY * eigenvectors[1][1];
    }
  }

private:
  std::vector<glm::vec3> _vertexPositions;
  std::vector<glm::vec3> _vertexNormals;
  std::vector<glm::vec2> _vertexTexCoords;
  std::vector<glm::uvec3> _triangleIndices;

  // for suggestive contouring
  std::vector<glm::vec2> _vertexCurvatures; // (κ1, κ2) at each vertex
  std::vector<glm::vec3> _principalDir1;    // Direction of κ1 at each vertex
  std::vector<glm::vec3> _principalDir2;    // Direction of κ2 at each vertex


  GLuint _vao = 0;
  GLuint _posVbo = 0;
  GLuint _normalVbo = 0;
  GLuint _texCoordVbo = 0;
  GLuint _ibo = 0;

  GLuint _curvatureVbo = 0;
  GLuint _pDir1Vbo = 0;
  GLuint _pDir2Vbo = 0;

  void computeEigenDecomposition(const glm::mat2x2 &matrix, glm::vec2 &eigenvalues, glm::mat2x2 &eigenvectors) {
    // Ensure the input matrix is symmetric
    float a = matrix[0][0];
    float b = matrix[0][1];
    float c = matrix[1][0];
    float d = matrix[1][1];

    // Compute eigenvalues using the quadratic formula
    float trace = a + d; // Trace of the matrix
    float determinant = a * d - b * c; // Determinant of the matrix

    float discriminant = std::sqrt(trace * trace / 4.0f - determinant);

    eigenvalues[0] = trace / 2.0f + discriminant; // Larger eigenvalue
    eigenvalues[1] = trace / 2.0f - discriminant; // Smaller eigenvalue

    // Compute eigenvectors for the eigenvalues
    for (int i = 0; i < 2; ++i) {
        float lambda = eigenvalues[i];
        glm::vec2 eigvec;

        // Solve (A - λI)v = 0
        if (std::abs(b) > 1e-6) { // Handle general case
            eigvec = glm::normalize(glm::vec2(b, lambda - a));
        } else if (std::abs(a - lambda) > 1e-6) { // Handle diagonal dominance case
            eigvec = glm::normalize(glm::vec2(lambda - d, c));
        } else { // Handle near-degenerate case
            eigvec = glm::vec2(1.0f, 0.0f); // Arbitrary choice for numerical stability
        }

        // Store the eigenvector in the output matrix
        eigenvectors[0][i] = eigvec[0];
        eigenvectors[1][i] = eigvec[1];
    }
  }

};


// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);

#endif  // MESH_H
