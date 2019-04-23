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

#include "fsh/fsh.hpp"

using std::cout, std::endl;

int main(int argc, char** argv) {
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool ret =
        tinyobj::LoadObj(shapes, materials, err, "models/fish_512.obj", NULL);

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        return EXIT_FAILURE;
    }

    size_t voffset = 0;
    size_t noffset = 0;

    std::vector<vx_vertex_t> vertexes;
    std::vector<vx_vec3_t> normals;
    float res = 0.0025;
    float precision = 0.001;

    for (size_t i = 0; i < shapes.size(); i++) {
        vx_mesh_t* mesh;

        mesh = vx_mesh_alloc(shapes[i].mesh.positions.size(),
                             shapes[i].mesh.indices.size());

        for (size_t f = 0; f < shapes[i].mesh.indices.size(); f++) {
            mesh->indices[f] = shapes[i].mesh.indices[f];
        }
        for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
            mesh->vertices[v].x = shapes[i].mesh.positions[3 * v + 0];
            mesh->vertices[v].y = shapes[i].mesh.positions[3 * v + 1];
            mesh->vertices[v].z = shapes[i].mesh.positions[3 * v + 2];
        }

        vx_point_cloud_t* result;
        result = vx_voxelize_pc(mesh, res, res, res, precision);

        for (int i = 0; i < result->nvertices; i++) {
            vertexes.push_back(result->vertices[i]);
            normals.push_back(result->normals[i]);
        }

        vx_point_cloud_free(result);
        vx_mesh_free(mesh);
    }

    printf("Number of vertices: %ld\n", vertexes.size());

    // begin fsh
    using pixel = bool;
    const uint d = 3;
    using PosInt = uint16_t;
    using NorInt = int8_t;
    using HashInt = uint8_t;
    using map = fsh::map<d, pixel, PosInt, NorInt, HashInt>;
    using PosPoint = fsh::point<d, PosInt>;
    using NorPoint = fsh::point<d, NorInt>;
    using IndexInt = uint64_t;
    int scale = 1 / res;
    int normalprec = 100;

    // prepare data
    float minVal = vertexes[0].x;
    for (const auto& it : vertexes) {
        for (const auto& u : it.v) {
            minVal = min(u, minVal);
        }
    }

    std::vector<map::data_t> data;
    std::set<IndexInt> data_b;

    using std::round;
    PosPoint boundings = {0, 0, 0};
    for (size_t i = 0; i < vertexes.size(); i++) {
        const vx_vertex_t& v = vertexes[i];
        const vx_vec3_t& vn = normals[i];
        PosPoint p;
        NorPoint n;
        for (uint i = 0; i < d; i++) {
            PosInt u = std::round((v.v[i] - minVal) * scale);
            boundings[i] = max(boundings[i], u);
            p[i] = u;

            PosInt w = std::round(vn.v[i] * normalprec);
            n[i] = w;
        }
        NorInt g = std::abs(n[0]);
        for (uint i = 1; i < d; i++) {
            g = std::__gcd(g, (NorInt)std::abs(n[i]));
        }
        for (uint i = 0; i < d; i++) {
            n[i] /= g;
        }
        data.push_back(map::data_t{p, n, pixel{true}});
    }

    std::sort(data.begin(), data.end(),
              [](const map::data_t& lhs, const map::data_t& rhs) {
                  return lhs.location < rhs.location;
              });
    auto newend =
        std::unique(data.begin(), data.end(),
                    [](const map::data_t& lhs, const map::data_t& rhs) {
                        return lhs.location == rhs.location;
                    });
    data.erase(newend, data.end());

    PosPoint border = boundings + PosInt(1);
    IndexInt data_max_size = 1;
    for (uint i = 0; i < d; i++) {
        data_max_size *= border[i];
    }
    for (const auto& it : data) {
        data_b.insert(fsh::point_to_index<d>(it.location, border, uint(-1)));
    }

    std::cout << "data size: " << data.size() << std::endl;
    std::cout << "data density: " << float(data.size()) / data_max_size
              << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    map s([&](size_t i) { return data[i]; }, data.size(), boundings);
    auto stop_time = std::chrono::high_resolution_clock::now();

    auto original_data_size =
        data_max_size * (sizeof(pixel) + sizeof(PosPoint));
    std::cout << "original data: " << (original_data_size / (1024 * 1024.0f))
              << " mb" << std::endl;

    std::cout << "class size: " << s.memory_size() / (1024 * 1024.0f) << " mb"
              << std::endl;

    std::cout << "compression factor vs dense: "
              << (float(s.memory_size()) / (data_max_size * sizeof(pixel)))
              << std::endl;

    std::cout << "compression factor vs sparse: "
              << (float(s.memory_size()) /
                  (sizeof(data) +
                   (sizeof(decltype(data)::value_type) - sizeof(NorInt)) *
                       data.size()))
              << std::endl;
    std::cout << "compression factor vs optimal: "
              << (float(s.memory_size()) /
                  (sizeof(data) + (sizeof(decltype(data)::value_type) -
                                   sizeof(PosPoint) - sizeof(NorInt)) *
                                      data.size()))
              << std::endl;

    std::cout << "map creation time: " << std::endl;
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                     stop_time - start_time)
                         .count() /
                     1000.0f
              << " seconds" << std::endl;

#if 1
    std::cout << "exhaustive test" << std::endl;
    for (IndexInt i = 0; i < data_max_size; i++) {
        PosPoint p = fsh::index_to_point<d>(i, border, IndexInt(-1));
        pixel exists = data_b.count(i);
        try {
            s.get(p);
            if (!exists) {
                std::cout << "found non-existing element!" << std::endl;
                std::cout << i << std::endl;
                std::cout << p << std::endl;
            }
        } catch (const std::out_of_range& e) {
            if (exists) {
                std::cout << "didn't find existing element!" << std::endl;
                std::cout << i << std::endl;
                std::cout << p << std::endl;
            }
        }
    }
    std::cout << "finished!" << std::endl;
#endif
    // end fsh

    // using data
#if 1
    vertexes.clear();
    cout << "Reading data" << endl;
    for (IndexInt i = 0; i < data_max_size; i++) {
        PosPoint p = fsh::index_to_point<d>(i, border, IndexInt(-1));
        try {
            s.get(p);
            vx_vertex_t vt;
            for (uint j = 0; j < d; j++) {
                vt.v[j] = 1.0f * p[j] / scale + minVal;
            }
            vertexes.push_back(vt);
        } catch (const std::out_of_range& e) {
        }
    }
    cout << "End reading" << endl;

    vx_vertex_t bound_min = vertexes[0];
    vx_vertex_t bound_max = vertexes[0];
    for (const auto& it : vertexes) {
        for (uint i = 0; i < d; i++) {
            bound_min.v[i] = std::min(bound_min.v[i], it.v[i]);
            bound_max.v[i] = std::max(bound_max.v[i], it.v[i]);
            // cout << it.v[i] << ' ';
        }
        // cout << endl;
    }
    vx_vertex_t center;
    for (uint i = 0; i < d; i++) {
        center.v[i] = (bound_max.v[i] + bound_min.v[i]) / 2;
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
                 vertexes.data(), GL_STATIC_DRAW);

    do {
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);
        computeMatricesFromInputs();
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        ModelMatrix[3][0] = -center.x;
        ModelMatrix[3][1] = -center.y;
        ModelMatrix[3][2] = -center.z;
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
#endif
    return 0;
}
