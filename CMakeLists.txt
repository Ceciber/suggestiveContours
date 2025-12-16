cmake_minimum_required(VERSION 3.0)

SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED YES)

project(tpSubdiv)

# Export compile commands for use with tools like clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable debug info (optional, remove for release)
set(CMAKE_BUILD_TYPE Debug)

# Add the executable and source files
add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/Mesh.cpp
  src/ShaderProgram.cpp
)

file(COPY src/vertexShader.glsl src/fragmentShader.glsl src/line.vert src/line.frag
     DESTINATION ${CMAKE_BINARY_DIR})

# Add glad
add_subdirectory(dep/glad)
target_link_libraries(${PROJECT_NAME} PRIVATE glad)

# Add GLFW
add_subdirectory(dep/glfw)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

# Add GLM
add_subdirectory(dep/glm)
target_link_libraries(${PROJECT_NAME} PRIVATE glm)

# Add libigl and Eigen include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/libigl/include     # Include libigl
  ${CMAKE_CURRENT_SOURCE_DIR}/libigl/external/eigen  # Include Eigen
)

# Link libigl
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS})

# Copy the output executable back to the source directory
add_custom_command(TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR}
)
