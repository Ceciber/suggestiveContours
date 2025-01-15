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
  void recomputePerVertexTextureCoordinates( );

  void init();
  void initOldGL();
  void render();
  void clear();

  void addPlan(float square_half_side = 1.0f);

  void Mesh::computeVertexCurvatures() {
    const auto &positions = vertexPositions();
    const auto &normals = vertexNormals();
    const auto &triangles = triangleIndices();

    size_t numVertices = positions.size();
    size_t numTriangles = triangles.size();

    // Initialize curvature tensors and weights
    std::vector<glm::mat2x2> curvatureTensors(numVertices, glm::mat2x2(0.0f));
    std::vector<float> vertexWeights(numVertices, 0.0f);

    // Loop over triangles
    for (size_t t = 0; t < numTriangles; ++t) {
        const glm::uvec3 &tri = triangles[t];
        const glm::vec3 &p0 = positions[tri.x];
        const glm::vec3 &p1 = positions[tri.y];
        const glm::vec3 &p2 = positions[tri.z];

        const glm::vec3 &n0 = normals[tri.x];
        const glm::vec3 &n1 = normals[tri.y];
        const glm::vec3 &n2 = normals[tri.z];

        // Edge vectors
        glm::vec3 e0 = p1 - p0;
        glm::vec3 e1 = p2 - p1;
        glm::vec3 e2 = p0 - p2;

        // Normal differences
        glm::vec3 dn0 = n1 - n0;
        glm::vec3 dn1 = n2 - n1;
        glm::vec3 dn2 = n0 - n2;

        // Build the least-squares system for II
        glm::mat2x3 A = {
            { glm::dot(e0, n0), glm::dot(e0, n1), glm::dot(e0, n2) },
            { glm::dot(e1, n0), glm::dot(e1, n1), glm::dot(e1, n2) }
        };
        glm::mat3x2 B = glm::transpose(A);
        glm::mat2x2 II = glm::inverse(glm::transpose(A) * A) * B;

        // Compute the area-weighted contribution to vertices
        float area = 0.5f * glm::length(glm::cross(e0, e1));
        glm::vec3 faceNormal = glm::normalize(glm::cross(e0, e1));

        // Accumulate per-vertex curvature tensors
        for (int i = 0; i < 3; ++i) {
            int vertex = tri[i];
            curvatureTensors[vertex] += II * area;
            vertexWeights[vertex] += area;
        }
    }

    // Normalize curvature tensors at vertices and compute principal directions/curvatures
    _vertexCurvatures.resize(numVertices, glm::vec2(0.0f));
    _principalDir1.resize(numVertices, glm::vec3(0.0f));
    _principalDir2.resize(numVertices, glm::vec3(0.0f));

    for (size_t v = 0; v < numVertices; ++v) {
        if (vertexWeights[v] > 0.0f) {
            curvatureTensors[v] /= vertexWeights[v];
        }

        // Diagonalize the 2x2 curvature tensor
        glm::mat2x2 curvatureTensor = curvatureTensors[v]; // for the matrix to be symmetric
        glm::mat2x2 symMatrix = 0.5f * (curvatureTensor + glm::transpose(curvatureTensor));
        glm::vec2 eigenvalues;
        glm::mat2x2 eigenvectors;

        computeEigenDecomposition(symMatrix, eigenvalues, eigenvectors);

        _vertexCurvatures[v] = eigenvalues;
        _principalDir1[v] = glm::normalize(glm::vec3(eigenvectors[0][0], eigenvectors[1][0], 0.0f)); // Principal direction for κ1
        _principalDir2[v] = glm::normalize(glm::vec3(eigenvectors[0][1], eigenvectors[1][1], 0.0f)); // Principal direction for κ2

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
