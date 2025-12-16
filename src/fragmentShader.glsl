#version 330 core
out vec4 fragColor;

uniform vec3 materialAlbedo;

void main() {
  fragColor = vec4(materialAlbedo, 1.0);
}


