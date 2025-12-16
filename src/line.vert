#version 330 core
layout(location = 0) in vec3 inPosition; // Vertex position
uniform mat4 uMVP; // Model-View-Projection matrix

void main()
{
    gl_Position = uMVP * vec4(inPosition, 1.0);
}