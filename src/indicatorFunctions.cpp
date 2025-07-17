#include "indicatorFunctions.h"

namespace locremesh {
Eigen::VectorXd indFuncTriangleQuality(const Eigen::MatrixXd& meshVertices,
                                       const Eigen::MatrixXi& meshFaces)
{
    double sqrt3    = std::sqrt(3);
    int    numFaces = meshFaces.rows();

    Eigen::VectorXd quality = Eigen::VectorXd::Zero(numFaces);
    for (int i = 0; i < numFaces; ++i) {
        Eigen::RowVector3d v0 = meshVertices.row(meshFaces(i, 0));
        Eigen::RowVector3d v1 = meshVertices.row(meshFaces(i, 1));
        Eigen::RowVector3d v2 = meshVertices.row(meshFaces(i, 2));

        double l01 = (v0 - v1).norm();
        double l12 = (v1 - v2).norm();
        double l20 = (v2 - v0).norm();

        double longestEdgeLength = std::max({l01, l12, l20});
        double perimeter         = l01 + l12 + l20;
        double area              = 0.5 * (v1 - v0).cross(v2 - v0).norm();

        quality[i] = (6 * area) / (sqrt3 * perimeter / 2 * longestEdgeLength);
    }

    return quality;
}
}  // namespace locremesh