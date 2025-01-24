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

  void setCameraPosition(const glm::vec3 &position) {cameraPosition = position;}

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

  //implementation of the object-space algorithm

  // Compute radial curvature κr for each vertex
  void computeVertexRadialCurvature() {
    _vertexRadialCurvatures.resize(_vertexPositions.size());

    std::cout << "Camera Position: (" 
              << cameraPosition.x << ", " 
              << cameraPosition.y << ", " 
              << cameraPosition.z << ")" << std::endl;

    for (size_t i = 0; i < _vertexPositions.size(); ++i) {
      // Compute view direction (camera to vertex)
      glm::vec3 viewDir = glm::normalize(cameraPosition - _vertexPositions[i]);

      // Compute the radial curvature κr = κ1 * (viewDir · PD1)^2 + κ2 * (viewDir · PD2)^2
      float dotPD1 = glm::dot(viewDir, _principalDir1[i]);
      float dotPD2 = glm::dot(viewDir, _principalDir2[i]);
      float kappa1 = _vertexCurvatures[i].x; // κ1
      float kappa2 = _vertexCurvatures[i].y; // κ2

      _vertexRadialCurvatures[i] = kappa1 * dotPD1 * dotPD1 + kappa2 * dotPD2 * dotPD2;

      /*std::cout << "Vertex " << i << ": kr = " << _vertexRadialCurvatures[i] 
              << ", viewDir = (" << viewDir.x << ", " << viewDir.y << ", " << viewDir.z << ")"
              << ", PD1 = (" << _principalDir1[i].x << ", " << _principalDir1[i].y << ", " << _principalDir1[i].z << ")"
              << ", PD2 = (" << _principalDir2[i].x << ", " << _principalDir2[i].y << ", " << _principalDir2[i].z << ")" << std::endl; */
    }
  }

  // Compute gradient of radial curvature ∇κr for each vertex
  void Mesh::computeGradientOfRadialCurvature() {
    // Ensure radial curvatures are computed
    if (_vertexRadialCurvatures.empty()) {
      std::cerr << "Radial curvatures not computed yet." << std::endl;
      return;
    }

    // Convert vertex positions to Eigen matrix
    Eigen::MatrixXd V(_vertexPositions.size(), 3);
    for (size_t i = 0; i < _vertexPositions.size(); ++i) {
      V(i, 0) = _vertexPositions[i].x;
      V(i, 1) = _vertexPositions[i].y;
      V(i, 2) = _vertexPositions[i].z;
    }

    // Convert triangle indices to Eigen matrix
    Eigen::MatrixXi F(_triangleIndices.size(), 3);
    for (size_t i = 0; i < _triangleIndices.size(); ++i) {
      F(i, 0) = _triangleIndices[i].x;
      F(i, 1) = _triangleIndices[i].y;
      F(i, 2) = _triangleIndices[i].z;
    }

    // Convert radial curvatures to Eigen vector
    Eigen::VectorXd radialCurvature(_vertexRadialCurvatures.size());
    for (size_t i = 0; i < _vertexRadialCurvatures.size(); ++i) {
      radialCurvature(i) = _vertexRadialCurvatures[i];
    }

    // Compute the gradient operator
    Eigen::SparseMatrix<double> G;
    igl::grad(V, F, G);

    std::cout << "G rows: " << G.rows() << ", G cols: " << G.cols() << std::endl;
    std::cout << "G non-zeros: " << G.nonZeros() << std::endl;

    // Compute the gradient of radial curvature
    Eigen::MatrixXd gradKappaR = G * radialCurvature;

    std::cout << "gradKappaR rows: " << gradKappaR.rows() << ", cols: " << gradKappaR.cols() << std::endl;

    // Resize _vertexGradKappaR to match the number of rows in gradKappaR
    _vertexGradKappaR.resize(F.rows());

    for (int i = 0; i < F.rows(); ++i) {
        // Each triangle corresponds to three consecutive rows in gradKappaR
        _vertexGradKappaR[i] = glm::vec3(
            gradKappaR(3 * i + 0, 0), // x-component
            gradKappaR(3 * i + 1, 0), // y-component
            gradKappaR(3 * i + 2, 0)  // z-component
        );

        /*std::cout << "Triangle " << i << " - Gradient of Radial Curvature: "
                << "(" << _vertexGradKappaR[i].x << ", "
                << _vertexGradKappaR[i].y << ", "
                << _vertexGradKappaR[i].z << ")" << std::endl; */
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

  std::vector<float> _vertexRadialCurvatures; // Radial curvature κr
  std::vector<glm::vec3> _vertexGradKappaR;  // Gradient of radial curvature

  glm::vec3 cameraPosition; //camera position

  GLuint _vao = 0;
  GLuint _posVbo = 0;
  GLuint _normalVbo = 0;
  GLuint _texCoordVbo = 0;
  GLuint _ibo = 0;

  GLuint _curvatureVbo = 0;
  GLuint _pDir1Vbo = 0;
  GLuint _pDir2Vbo = 0;
  GLuint _radCurvatureVbo = 0;
  GLuint _gradKappaRVbo = 0;

};


// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);

#endif  // MESH_H
