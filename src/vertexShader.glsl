#version 330 core

layout(location = 0) in vec3 vPosition; // Vertex position
layout(location = 1) in vec3 vNormal;   // Vertex normal (unused in this simplified version)
layout(location = 2) in vec2 vTexCoord; // Texture coordinates (unused in this simplified version)

uniform mat4 modelMat; // Model matrix
uniform mat4 viewMat;  // View matrix
uniform mat4 projMat;  // Projection matrix

void main()
{
    // Transform the vertex position to clip space
    gl_Position = projMat * viewMat * modelMat * vec4(vPosition, 1.0);
}
