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

#include <igl/principal_curvature.h>
#include <igl/grad.h>
#include <Eigen/Geometry>

#include <glm/gtx/string_cast.hpp>

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

  std::vector<std::set<unsigned int>> neighboringVertices; //

  void setCameraPosition(const glm::vec3 &position) {cameraPosition = position;}

  void setLineShaderProgramID(GLuint programID) {
    lineShaderProgramID = programID;
  }

  // New getters (if needed)
  const std::vector<glm::vec3>& contourVertices() const { return _contourVertices; }
  const std::vector<glm::uvec2>& contourIndices() const { return _contourIndices; }

  /// Compute the parameters of a sphere which bounds the mesh
  void computeBoundingSphere(glm::vec3 &center, float &radius) const;

  void recomputePerVertexNormals(bool angleBased = false);
  void recomputePerVertexTextureCoordinates( );
  
  void Mesh::computeNeighboringVertices() 
  {
    neighboringVertices.clear();
    neighboringVertices.resize(_vertexPositions.size());

    // Iterate over all triangles to build the adjacency list
    for (const auto &triangle : _triangleIndices) {
        unsigned int a = triangle[0];
        unsigned int b = triangle[1];
        unsigned int c = triangle[2];

        // Add neighbors for each vertex in the triangle
        neighboringVertices[a].insert(b);
        neighboringVertices[a].insert(c);
        neighboringVertices[b].insert(a);
        neighboringVertices[b].insert(c);
        neighboringVertices[c].insert(a);
        neighboringVertices[c].insert(b);
    }
  }

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
      // If the new vertex is not boundary
      else {
          // We compute the sum of the two vertices at the extremes of the edge
          glm::vec3 sum_ab = _vertexPositions[a] + _vertexPositions[b];

          // We compute the sum of the vertices that are in the triangles shared by the edge but not the extremes
          glm::vec3 sum_jk(0.0f, 0.0f, 0.0f);

          // Iterate over the triangles sharing the edge
          for (unsigned int triangleIndex : trianglesOnEdge[Eab]) {
              // Get the vertices of the triangle
              const glm::uvec3 &triangle = _triangleIndices[triangleIndex];

              // Find the vertex that is not part of the edge (a, b)
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
    computeVertexCurvatures();
  }


  void Mesh::taubinSmooth(float lambda, float mu, int iterations)
  {
    if (neighboringVertices.empty()) {
      computeNeighboringVertices();
    }
    // We’ll do (λ) pass, then (μ) pass, for each iteration
    for(int iter = 0; iter < iterations; ++iter)
    {
        // --- Pass 1: Laplacian step with factor λ ---
        std::vector<glm::vec3> tmpPositions(_vertexPositions.size());
        for(size_t i = 0; i < _vertexPositions.size(); i++)
        {
            glm::vec3 v = _vertexPositions[i];
            glm::vec3 sumNeighbors(0.0f);
            // Sum up neighbor positions
            for(unsigned int nbr : neighboringVertices[i])
            {
                sumNeighbors += _vertexPositions[nbr];
            }
            float degree = static_cast<float>(neighboringVertices[i].size());
            glm::vec3 avgNeighbors = (degree > 0.0f) ? (sumNeighbors / degree) : v;

            // Move along Laplacian
            tmpPositions[i] = v + lambda * (avgNeighbors - v);
        }

        // --- Pass 2: Laplacian step with factor μ ---
        std::vector<glm::vec3> tmpPositions2(_vertexPositions.size());
        for(size_t i = 0; i < _vertexPositions.size(); i++)
        {
            glm::vec3 v = tmpPositions[i];
            glm::vec3 sumNeighbors(0.0f);
            for(unsigned int nbr : neighboringVertices[i])
            {
                sumNeighbors += tmpPositions[nbr];
            }
            float degree = static_cast<float>(neighboringVertices[i].size());
            glm::vec3 avgNeighbors = (degree > 0.0f) ? (sumNeighbors / degree) : v;

            tmpPositions2[i] = v + mu * (avgNeighbors - v);
        }

        // Store final positions after the two passes
        _vertexPositions = tmpPositions2;
    }

    // Optionally recompute normals, curvatures, and upload to GPU
    recomputePerVertexNormals(false);
    recomputePerVertexTextureCoordinates( );
    computeVertexCurvatures();
  }

  // compute curvatures and principal directions for each vertex
  void computeVertexCurvatures() {
    // Convert _vertexPositions and _triangleIndices into Eigen matrices
    Eigen::MatrixXd V(_vertexPositions.size(), 3); // Vertices matrix
    Eigen::MatrixXi F(_triangleIndices.size(), 3); // Faces matrix

    for (size_t i = 0; i < _vertexPositions.size(); ++i) {
      V(i, 0) = _vertexPositions[i].x;
      V(i, 1) = _vertexPositions[i].y;
      V(i, 2) = _vertexPositions[i].z;
    }

    for (size_t i = 0; i < _triangleIndices.size(); ++i) {
      F(i, 0) = _triangleIndices[i].x;
      F(i, 1) = _triangleIndices[i].y;
      F(i, 2) = _triangleIndices[i].z;
    }

    // Matrices to store the principal curvatures and directions
    Eigen::MatrixXd PD1, PD2; // Principal directions
    Eigen::VectorXd PV1, PV2; // Principal curvatures

    // Compute the principal curvatures and directions using libigl
    igl::principal_curvature(V, F, PD1, PD2, PV1, PV2);

    // Resize member variables to store the results
    _vertexCurvatures.resize(_vertexPositions.size());
    _principalDir1.resize(_vertexPositions.size());
    _principalDir2.resize(_vertexPositions.size());

    // Store the results in the member variables
    for (size_t i = 0; i < _vertexPositions.size(); ++i) {
      _vertexCurvatures[i] = glm::vec2(PV1[i], PV2[i]); // Store κ1, κ2
      _principalDir1[i] = glm::vec3(PD1(i, 0), PD1(i, 1), PD1(i, 2)); // Store direction of κ1
      _principalDir2[i] = glm::vec3(PD2(i, 0), PD2(i, 1), PD2(i, 2)); // Store direction of κ2
    }
  }

  void Mesh::computeSuggestiveContours() {
    _contourVertices.clear();
    _contourIndices.clear();

    // Iterate over all triangles
    for (size_t t = 0; t < _triangleIndices.size(); ++t) {
        glm::uvec3 tri = _triangleIndices[t];
        glm::vec3 pos[3], norm[3];
        float kappa_r[3];
        glm::vec3 radial[3];

        // For each vertex of the triangle:
        for (int i = 0; i < 3; ++i) {
            unsigned int idx = tri[i];
            pos[i] = _vertexPositions[idx];
            norm[i] = _vertexNormals[idx];

            // Compute view vector (from vertex to camera)
            glm::vec3 view = cameraPosition - pos[i];

            // Project the view vector onto the tangent plane
            glm::vec3 proj = view - glm::dot(view, norm[i]) * norm[i];
            if (glm::length(proj) > 0.0001f)
                radial[i] = glm::normalize(proj);
            else
                radial[i] = glm::vec3(0.0f);

            // Get principal curvatures and directions at the vertex
            float k1 = _vertexCurvatures[idx].x;
            float k2 = _vertexCurvatures[idx].y;
            glm::vec3 e1 = _principalDir1[idx];
            glm::vec3 e2 = _principalDir2[idx];

            // Compute radial curvature κ_r = k1*(r·e1)² + k2*(r·e2)²
            float d1 = glm::dot(radial[i], e1);
            float d2 = glm::dot(radial[i], e2);
            kappa_r[i] = k1 * d1 * d1 + k2 * d2 * d2;
        }

        // Check edges for zero-crossings: for each edge between vertices i and j
        std::vector<glm::vec3> intersections;
        for (int edge = 0; edge < 3; ++edge) {
            int i0 = edge;
            int i1 = (edge + 1) % 3;
            // If the radial curvature at the two vertices has opposite signs, there’s a zero crossing.
            if (kappa_r[i0] * kappa_r[i1] < 0.0f) {
                // Compute the interpolation factor along the edge.
                float t = fabs(kappa_r[i0]) / (fabs(kappa_r[i0]) + fabs(kappa_r[i1]));
                glm::vec3 p = glm::mix(pos[i0], pos[i1], t);
                intersections.push_back(p);
            }
        }

        // If exactly two edges cross, add the segment connecting the intersections.
        if (intersections.size() == 2) {
            unsigned int baseIndex = _contourVertices.size();
            _contourVertices.push_back(intersections[0]);
            _contourVertices.push_back(intersections[1]);
            _contourIndices.push_back(glm::uvec2(baseIndex, baseIndex + 1));
        }
        // (Optional: if three intersections occur, handle appropriately.)
    }
  }

  void Mesh::uploadContourGeometry() {
    // Generate the VAO/VBO/IBO if they haven't been created yet.
    if (_contourVao == 0) {
        glGenVertexArrays(1, &_contourVao);
        glGenBuffers(1, &_contourVbo);
        glGenBuffers(1, &_contourIbo);
    }
    glBindVertexArray(_contourVao);

    // Upload vertices
    glBindBuffer(GL_ARRAY_BUFFER, _contourVbo);
    glBufferData(GL_ARRAY_BUFFER, _contourVertices.size() * sizeof(glm::vec3),
                 _contourVertices.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0); // Assuming location 0 in your contour vertex shader
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    // Upload indices (line segments)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _contourIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, _contourIndices.size() * sizeof(glm::uvec2),
                 _contourIndices.data(), GL_DYNAMIC_DRAW);

    glBindVertexArray(0);
  }

  void Mesh::renderContours(const glm::mat4 &MVP) {
    // Use the line shader program
    glUseProgram(lineShaderProgramID);

    // Pass the MVP matrix to the shader
    GLint loc = glGetUniformLocation(lineShaderProgramID, "uMVP");
    glUniformMatrix4fv(loc, 1, GL_FALSE, &MVP[0][0]);

    // Bind the contour VAO and draw the lines
    glBindVertexArray(_contourVao);
    glDrawElements(GL_LINES, _contourIndices.size() * 2, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
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

  // NEW: Contour geometry
  std::vector<glm::vec3> _contourVertices;    // Positions of contour points
  std::vector<glm::uvec2> _contourIndices;      // Each uvec2 defines a line segment (indices into _contourVertices)
  
  // NEW: GPU objects for contours
  GLuint _contourVao = 0;
  GLuint _contourVbo = 0;
  GLuint _contourIbo = 0;

  GLuint _vao = 0;
  GLuint _posVbo = 0;
  GLuint _normalVbo = 0;
  GLuint _texCoordVbo = 0;
  GLuint _ibo = 0;

  GLuint _curvatureVbo = 0;
  GLuint _pDir1Vbo = 0;
  GLuint _pDir2Vbo = 0;

  GLuint lineShaderProgramID;

  glm::vec3 cameraPosition; //camera position

};

// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);

#endif  // MESH_H
