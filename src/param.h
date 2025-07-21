#include <TinyAD/ScalarFunction.hh>
#include <TinyAD/Utils/Helpers.hh>
#include <TinyAD/Utils/LineSearch.hh>
#include <TinyAD/Utils/NewtonDecrement.hh>
#include <TinyAD/Utils/NewtonDirection.hh>
#include <iostream>
#include "TutteEmbeddingIGL.h"

template <typename PassiveT>
Eigen::Matrix<PassiveT, Eigen::Dynamic, 2> param(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    Eigen::MatrixXd&       previousParam,
    const int              numMaxIterations = 30)
{
    bool shouldCheckForFlip = true;
    if (previousParam.size() == 0) {
        previousParam = tutte_embedding(V, F);
    }

    // Define the local coordinates for ach triangle using the first vector and
    // the normal to make coords.
    std::vector<Eigen::Matrix<double, 2, 3>> flattened_ref_triangles;
    flattened_ref_triangles.reserve(F.rows());
    for (int i = 0; i < F.rows(); ++i) {
        Eigen::Vector3d             a_r    = V.row(F(i, 0));
        Eigen::Vector3d             b_r    = V.row(F(i, 1));
        Eigen::Vector3d             c_r    = V.row(F(i, 2));
        Eigen::Vector3d             e1     = b_r - a_r;
        Eigen::Vector3d             e2     = c_r - a_r;
        Eigen::Vector3d             normal = e1.cross(e2).normalized();
        Eigen::Vector3d             x_axis = e1.normalized();
        Eigen::Vector3d             y_axis = normal.cross(x_axis);
        Eigen::Matrix<double, 2, 3> tri2D;
        tri2D.col(0) = Eigen::Vector2d(0, 0);
        tri2D.col(1) = Eigen::Vector2d(e1.norm(), 0);
        tri2D.col(2) = Eigen::Vector2d(e2.dot(x_axis), e2.dot(y_axis));
        flattened_ref_triangles.push_back(tri2D);
    }

    // This is the function which we will optimize, it hopefully returns the
    // energies for all the faces as a lambda function?
    auto func = TinyAD::scalar_function<2, PassiveT>(TinyAD::range(V.rows()),
                                                     TinyAD::EvalSettings{});
    func.template add_elements<3>(
        TinyAD::range(F.rows()),
        [&](auto& element) -> TINYAD_SCALAR_TYPE(element) {
            using T = TINYAD_SCALAR_TYPE(element);
            Eigen::Index fid =
                element.handle;  // Get reference triangle (in 2D) from
                                 // precomputed flattened geometry
            const Eigen::Matrix<double, 2, 3>& ref_tri =
                flattened_ref_triangles[fid];  // Build reference matrix Mr from
                                               // edges of the flattened
                                               // triangle
            Eigen::Matrix<T, 2, 2> Mr;
            Mr.col(0) = ref_tri.col(1) - ref_tri.col(0);
            Mr.col(1) = ref_tri.col(2) -
                        ref_tri.col(0);  // Get embedded triangle from current
                                         // optimization variables
            Eigen::Vector<T, 2>    a = element.variables(F(fid, 0));
            Eigen::Vector<T, 2>    b = element.variables(F(fid, 1));
            Eigen::Vector<T, 2>    c = element.variables(F(fid, 2));
            Eigen::Matrix<T, 2, 2> M = TinyAD::col_mat(b - a, c - a);
            if (M.determinant() <= 0.0) {
                return INFINITY;
            }
            return ((M * Mr.inverse()).squaredNorm() +
                    (Mr * M.inverse()).squaredNorm()) /
                   (PassiveT)F.rows();
        });
    Eigen::VectorX<PassiveT> x = func.x_from_data(
        [&](Eigen::Index v_idx) { return previousParam.row(v_idx); });
    TinyAD::LinearSolver<PassiveT> solver;

    // Optimization step based off the defined function
    std::vector<double> convergenceHistory;
    for (int i = 0; i < numMaxIterations; ++i) {
        auto [f, g, H_proj]      = func.eval_with_hessian_proj(x);
        Eigen::VectorX<double> d = TinyAD::newton_direction(g, H_proj, solver);
        x                        = TinyAD::line_search(x, d, f, g, func);

        float convergenceRate = TinyAD::newton_decrement(d, g);
        convergenceHistory.push_back(convergenceRate);
        if (convergenceRate < 1e-4) {
            break;
        }
    }
    std::cout << "  Converged in " << convergenceHistory.size() << " iterations"
              << std::endl;

    // Format not weird
    Eigen::Matrix<PassiveT, Eigen::Dynamic, 2> UV(V.rows(), 2);
    for (int i = 0; i < V.rows(); ++i)
        UV.row(i) = x.template segment<2>(2 * i);

    // Check for global UV flip and correct it to ensure consistent orientation.
    // The symmetric Dirichlet energy is reflection-invariant, so the optimizer
    // might converge to a flipped solution. We check the orientation of the
    // resulting UV triangles. Our reference triangles `flattened_ref_triangles`
    // are constructed to have a positive determinant, so we expect the UV
    // triangles to also have positive determinants.
    if (shouldCheckForFlip) {
        for (int i = 0; i < F.rows(); ++i) {
            Eigen::Vector2<PassiveT> p0 = UV.row(F(i, 0));
            Eigen::Vector2<PassiveT> p1 = UV.row(F(i, 1));
            Eigen::Vector2<PassiveT> p2 = UV.row(F(i, 2));
            // Signed area is 0.5 * det([p1-p0, p2-p0]). We only need the sign
            // of the determinant.
            PassiveT det = (p1.x() - p0.x()) * (p2.y() - p0.y()) -
                           (p1.y() - p0.y()) * (p2.x() - p0.x());
            if (det < 0) {
                // If more than half of the triangles are flipped, we flip the
                // entire UV map's V-coordinate.
                std::cout << "  Parametrization was flipped, correcting."
                          << std::endl;
                UV.col(1) *= -1.0;
                break;
            }
        }
    }

    return UV;
}