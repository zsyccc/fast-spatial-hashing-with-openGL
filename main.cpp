// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
using namespace glm;

#include "common/shader.hpp"
#include "common/texture.hpp"
#include "common/controls.hpp"
#include "common/objloader.hpp"

#include "voxelizer.h"
#include "tiny_obj_loader.h"
#include <iostream>
#include <functional>
#include <algorithm>

#include <experimental/optional>
#include <chrono>
#include <stdint.h>
#include <thread>

#include <set>
#include <cassert>

using std::cout, std::endl;

int main(int argc, char** argv) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    std::string warn;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                "models/cube.obj");

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        return EXIT_FAILURE;
    }

    size_t voffset = 0;
    size_t noffset = 0;

    float res = 0.025;
    float precision = 0.01;
    int nvertices = attrib.vertices.size();
    int nindices = 0;
    for (size_t s = 0; s < shapes.size(); s++) {
        nindices += shapes[s].mesh.indices.size();
    }

    vx_mesh_t* mesh = vx_mesh_alloc(nvertices, nindices);

    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                mesh->indices[index_offset + v] = idx.vertex_index;
                mesh->normalindices[index_offset + v] = idx.normal_index;
                mesh->vertices[idx.vertex_index].x =
                    attrib.vertices[3 * idx.vertex_index + 0];
                mesh->vertices[idx.vertex_index].y =
                    attrib.vertices[3 * idx.vertex_index + 1];
                mesh->vertices[idx.vertex_index].z =
                    attrib.vertices[3 * idx.vertex_index + 2];
                mesh->normals[idx.normal_index].x =
                    attrib.normals[3 * idx.normal_index + 0];
                mesh->normals[idx.normal_index].y =
                    attrib.normals[3 * idx.normal_index + 1];
                mesh->normals[idx.normal_index].z =
                    attrib.normals[3 * idx.normal_index + 2];
            }
            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }

    std::vector<vx_vertex_t> vertexes;
    std::vector<vx_vec3_t> normals;

    vx_point_cloud_t* result = vx_voxelize_pc(mesh, res, res, res, precision);

    printf("Number of vertices: %ld\n", result->nvertices);
    for (int i = 0; i < result->nvertices; i++) {
        vertexes.push_back(result->vertices[i]);
        normals.push_back(result->normals[i]);
    }

    vx_mesh_free(mesh);
    vx_point_cloud_free(result);

    int nv = vertexes.size();
    for (int i = 0; i < nv; i++) {
        vx_vertex_t& v = vertexes[i];
        const vx_vec3_t& vn = normals[i];
        float zoom = 0.2f;
        v.x += zoom * vn.x;
        v.y += zoom * vn.y;
        v.z += zoom * vn.z;
    }

    // Initialise GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,
                   GL_TRUE);  // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1024, 768, "PSH Demo", NULL, NULL);
    if (window == NULL) {
        fprintf(
            stderr,
            "Failed to open GLFW window. If you have an Intel GPU, they are "
            "not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true;  // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders("SimpleVertexShader.vertexshader",
                                   "SimpleFragmentShader.fragmentshader");
    std::cout << "complied" << std::endl;
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertexes.size() * 3,
                 &vertexes[0], GL_STATIC_DRAW);

    do {
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);
        computeMatricesFromInputs();
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        // ModelMatrix[3][0] = -center.x;
        // ModelMatrix[3][1] = -center.y;
        // ModelMatrix[3][2] = -center.z;
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(0,  // attribute 0. No particular reason for 0,
                                  // but must match the layout in the shader.
                              3,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized?
                              0,         // stride
                              (void*)0   // array buffer offset
        );

        // glPointSize(2.0f);

        // Draw the triangle !
        glDrawArrays(GL_POINTS, 0,
                     vertexes.size());  // 3 indices starting at 0 -> 1 triangle

        glDisableVertexAttribArray(0);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

    }  // Check if the ESC key was pressed or the window was closed
    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0);

    // Cleanup VBO
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}
