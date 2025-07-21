#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>

struct Spring {
    int i, j;
    double rest_length;
    double stiffness;
};

class MassSpringSystem {
public:
    Eigen::MatrixXd x, v;           // n x 2
    Eigen::VectorXd m;              // n x 1
    std::vector<Spring> springs;

    std::set<int> fixed;            // fixed point indices
    double dt = 0.01;
    Eigen::Vector2d gravity = {0.0, -9.8};

    void initializeGrid(int rows, int cols);
    void stepImplicitEuler();

    // Energy and gradient for mass–spring system (TinyAD)
    double computeSpringEnergyAndDerivatives(
        const Eigen::VectorXd& x_flat,
        Eigen::VectorXd& grad,
        Eigen::SparseMatrix<double>& hess) const;
};
