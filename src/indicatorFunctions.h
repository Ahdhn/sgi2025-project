#pragma once

#include "igl/boundary_loop.h"
#include "igl/face_areas.h"
#include "igl/igl_inline.h"
#include "igl/read_triangle_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "remesh/src/remesh_botsch.h"

#include <Eigen/Core>
#include <iostream>

namespace locremesh {  /////////////////////////////////////////////////////////

/**
 * Implementation of the triangle quality metric presented in
 * [Abdelkader et al.
 * 2017](https://escholarship.org/content/qt5347s75h/qt5347s75h.pdf?v=lg)
 *
 * @return A vector of doubles, where each value represents the quality of a
 * triangle.
 */
Eigen::VectorXd indFuncTriangleQuality(const Eigen::MatrixXd& meshVertices,
                                       const Eigen::MatrixXi& meshFaces);

};  // namespace locremesh
