## Suggestive Contours for 3D Mesh Visualization

This repository contains the source code for an implementation of **suggestive contours**, a technique proposed by De Carlo et al. to enhance the visual perception of 3D mesh geometry in non-photorealistic rendering.

The implementation follows the algorithm proposed by DeCarlo et al., using an **object-space algorithm** to compute both silhouette and suggestive contours. It leverages the **GPU** with shaders for real-time rendering while performing principal curvatures and gradient computations on the **CPU**.

---

### Key Features and Implementation Details

#### 1. Preprocessing: Subdivision and Smoothing

To improve mesh quality and achieve better contouring, a preprocessing step is applied, especially for low-resolution meshes like the simplified Stanford bunny. The project employs subdivision and smoothing techniques to improve mesh quality.

* **Subdivision:** The `subdivideLoop()` function, implemented in a previous lab, is utilized. This is triggered by pressing the **L** key.
* **Smoothing (Taubin Algorithm):** The **Taubin Smoothing algorithm** is applied, involving two consecutive Gaussian smoothing steps with alternating positive ($\lambda$) and negative ($\mu$) scale factors. This process is repeated multiple times to produce a significant smoothing effect.
    * Parameters used are $\lambda=0.33$ and $\mu=-0.34$, repeated for 10 iterations.
    * Triggered by pressing the **R** key.



#### 2. Contour Definitions

The algorithm computes two main types of lines:

| Contour Type | Definition | Condition |
| :--- | :--- | :--- |
| **Normal Contours (Silhouettes)** | Points on a surface where the normal ($n$) is perpendicular to the viewing direction ($v$). | $n\cdot v=0$ |
| **Suggestive Contours** | Locations where contours would appear under slight viewpoint changes. | Radial curvature $\kappa_{r}=0$ AND directional derivative of $\kappa_{r}$ along $w$ is positive: $D_{w}\kappa_{r}>0$ |

#### 3. Core Algorithm Steps (Object-Space)

The object-space algorithm proceeds by finding the solution to $\kappa_{r}=0$ over the entire mesh and then trimming this solution using $D_{w}\kappa_{r}>0$.

1.  **Principal Curvatures and Directions:**
    * Computed on the **CPU**.
    * The implementation leverages the `libigl` library with the `principal_curvature` function, which uses **quadric fitting**.
    * A quadratic surface is fitted to the one-ring neighbors or a local patch of each vertex using least squares.
    * The eigenvalues and eigenvectors of the Weingarten matrix of the fitted surface give the principal curvatures ($\kappa_{1}, \kappa_{2}$) and directions ($e_{1}, e_{2}$).

2.  **Radial Curvature ($\kappa_{r}$):**
    * The view vector ($v$) from the camera to the vertex is projected onto the tangent plane by subtracting its component in the direction of the vertex normal $n$.
    * The projected vector $v_{proj}$ is normalized to obtain the unit vector $w$.
    * $\kappa_{r}$ is computed as: $\kappa_{r}=\kappa_{1}(w\cdot e_{1})^{2}+\kappa_{2}(w\cdot e_{2})^{2}$.
    * This computation was moved directly to the **GPU** (in the shaders) to significantly reduce the performance bottleneck associated with data transmission between the CPU and the shaders.

3.  **Gradient of Radial Curvature ($D_{w}\kappa_{r}$):**
    * The gradient is still computed on the **CPU** and is essential for filtering suggestive contours.

4.  **GPU Rendering (Shaders):**
    * The principal curvatures and directions, and the gradient of the radial curvature, are sent to the shaders.
    * In the shaders, the algorithm checks condition (1) for contours, computes radial curvature per vertex and checks condition (2), and finally normalizes the gradient of the curvature and checks for condition (3).
    * Fragments satisfying the conditions are drawn in black.
    * A small threshold (e.g., 0.2) ensures stable contour detection for $n\cdot v=0$.
    * A small positive threshold $t_{d}$ is applied to condition (3) to avoid spurious zero crossings:
        $$\frac{D_{w}\kappa_{r}}{||w||}>t_{d} \quad \text{}$$



#### 4. Post-processing

A post-processing pass was implemented to enhance the contours and address noise/inconsistencies. The pass uses image-space edge detection techniques.

* The rendered image is processed using a second fragment shader (`postProcessingFragment.glsl`).
* Key objectives include:
    * **Depth-based edge detection:** Analyzing the depth buffer to identify discontinuities corresponding to object boundaries, which improves silhouette definition.
    * **Sobel filter on color gradients:** Applied to the color buffer to highlight intensity changes and capture edges that may have been missed by the depth-based approach.
    * **Adaptive edge refinement:** Combining depth and color-based information to smooth out noise while preserving meaningful contours.

---

### Tested Meshes

The application was tested with:
* The provided **Suzanne monkey**.
* A simplified **Stanford bunny** (with 500 faces).
