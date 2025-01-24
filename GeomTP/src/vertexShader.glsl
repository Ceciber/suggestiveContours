#version 330 core            // minimal GL version support expected from the GPU

layout(location=0) in vec3 vPosition;  // The 1st input attribute is the position (CPU side: glVertexAttrib 0)
layout(location=1) in vec3 vNormal;    // The 2nd input attribute is the normal
layout(location=2) in vec2 vTexCoord;  // The 3rd input attribute is the texture coordinates

// for suggestive contours
layout(location = 3) in vec3 curvature; // Radial curvature
layout(location = 4) in vec3 pDir1;     // Principal direction 1 (κ1 direction)
layout(location = 5) in vec3 pDir2;     // Principal direction 2 (κ2 direction)
layout(location = 6) in float kr;          // Radial curvature
layout(location = 7) in vec3 gradKr;      // Gradient of radial curvature

uniform mat4 modelMat, viewMat, projMat;  // Transformation matrices
uniform mat3 normMat;                     // Normal matrix (to transform normals)

uniform vec3 camPos;  // Camera position

out vec3 fPositionModel;  // Model-space position of the fragment
out vec3 fPosition;       // World-space position of the fragment
out vec3 fNormal;         // Normal in world space
out vec3 fViewVector;     // View vector in world space
out vec2 fTexCoord;       // Texture coordinates

out float radCurvature;
out float dRadCurvature;

void main() {
    // Transform the position to world space and pass it to the fragment shader
    fPositionModel = vPosition;
    fPosition = (modelMat * vec4(vPosition, 1.0)).xyz;
    
    // Transform the normal to world space
    fNormal = normMat * vNormal;
    
    // Calculate the view vector (camera position - fragment position)
    fViewVector = normalize(camPos - fPosition);

    // Pass the texture coordinates to the fragment shader
    fTexCoord = vTexCoord;

    // sugg. contours
    radCurvature = normalize(kr);
    vec3 w = normalize(fViewVector - dot(fViewVector, fNormal) * fNormal); // projection of the view vector onto the tangent plane of the surface
    dRadCurvature = dot(gradKr, w);

    // Compute the final position of the vertex in clip space
    gl_Position = projMat * viewMat * modelMat * vec4(vPosition, 1.0);
}