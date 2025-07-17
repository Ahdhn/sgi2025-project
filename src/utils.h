#pragma once

#include "igl/boundary_loop.h"
#include "igl/igl_inline.h"

#include <Eigen/Core>

/**
 * Returns the coordinates of the vertices of the mesh given their indices.
 *
 * @param inputMeshVertices The vertices of the mesh.
 * @param verticesIdxs The indices of the vertices whose coordinates are to be
 * returned.
 * @return An Eigen::MatrixXd where each row represents the 3D coordinates of a
 * vertex.
 */
template <typename T>
Eigen::MatrixXd getMeshVerticesCoords(const Eigen::MatrixXd& inputMeshVertices,
                                      const T&               verticesIdxs)
{
    Eigen::MatrixXd boundaryVerticesCoords(verticesIdxs.size(), 3);

    for (int i = 0; int idx : verticesIdxs) {
        boundaryVerticesCoords.row(i) = inputMeshVertices.row(idx);
        i++;
    }
    return boundaryVerticesCoords;
}

/**
 * Returns a list of loops that make up the boundary of the mesh.
 * A mesh will have more then one boundary loop if it has holes.
 *
 * @param inputMeshVertices The vertices of the mesh.
 * @param inputMeshFaces The faces of the mesh.
 * @return A vector of vectors, where each inner vector represents a boundary
 * loop and contains the indices of the vertices in that loop.
 */
inline std::vector<std::vector<int>> getMeshBoundaryLoops(
    const Eigen::MatrixXd& inputMeshVertices,
    const Eigen::MatrixXi& inputMeshFaces)
{
    std::vector<std::vector<int>> boundaryVerticesIdxs;
    igl::boundary_loop(inputMeshFaces, boundaryVerticesIdxs);
    return boundaryVerticesIdxs;
}
