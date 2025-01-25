#version 330 core

in vec3 fPosition;      // World-space position of the fragment
in vec3 fNormal;        // Normal in world space
in vec3 fViewVector;    // View vector in world space
in vec2 fTexCoord;      // Texture coordinates (optional for rendering)
in float radCurvature;
in float dRadCurvature;

out vec4 fragColor;     // Output color of the fragment

void main() {

    // Compute the dot product between the normal and view vector
    float dotProduct = dot(normalize(fNormal), normalize(fViewVector));
 
    // Contour detection: when dot product is near zero, it’s a contour
    if (abs(dotProduct) < 0.1) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0); // Black
    }
    // Visualizing curvature: use radial curvature to create color gradients
    else if (abs(radCurvature) < 0.1 && dRadCurvature > 0.0) {
        fragColor = vec4(1.0, 0.0, 0.0, 1.0);  // Red
    } 
    else {
        fragColor = vec4(0.7, 0.7, 0.7, 0.7);
    }  

}