#include "mass_spring.h"
#include <TinyAD/Scalar.hh>
#include <Eigen/SparseCholesky>

void MassSpringSystem::initializeGrid(int rows, int cols) {
    int n = rows * cols;
    x.resize(n, 2);
    v = Eigen::MatrixXd::Zero(n, 2);
    m = Eigen::VectorXd::Constant(n, 1.0);

    double spacing = 1.0;
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) {
            int idx = i * cols + j;
            x.row(idx) = Eigen::Vector2d(j * spacing, -i * spacing);
        }

    auto idx = [&](int i, int j) { return i * cols + j; };

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int a = idx(i, j);
            if (j + 1 < cols) {
                int b = idx(i, j+1);
                springs.push_back({a, b, (x.row(a) - x.row(b)).norm(), 10.0});
            }
            if (i + 1 < rows) {
                int b = idx(i+1, j);
                springs.push_back({a, b, (x.row(a) - x.row(b)).norm(), 10.0});
            }
        }
    }

    // Pin the top row
    for (int j = 0; j < cols; ++j)
        fixed.insert(idx(0, j));
}

void MassSpringSystem::stepImplicitEuler() {
    int n = x.rows();
    Eigen::VectorXd x_flat = Eigen::Map<Eigen::VectorXd>(x.data(), 2 * n);

    auto tape = TinyAD::scalar::make_tape<2>(x_flat, [&](int idx) {
        int vi = idx / 2;
        return fixed.find(vi) == fixed.end(); // skip fixed
    });

    auto X = tape.ad_variables();

    tape.add_objective([&](auto& X) {
        double energy = 0.0;

        for (int i = 0; i < n; ++i) {
            auto xi = X.vec(i);
            Eigen::Vector2d xi_prev = x.row(i);
            Eigen::Vector2d vi = (xi - xi_prev) / dt;
            Eigen::Vector2d g = gravity;
            energy += 0.5 * m(i) * (vi - dt * g).squaredNorm();
        }

        for (const auto& s : springs) {
            auto xi = X.vec(s.i);
            auto xj = X.vec(s.j);
            auto d = xi - xj;
            auto l = d.norm();
            auto e = l - s.rest_length;
            energy += 0.5 * s.stiffness * e * e;
        }

        return energy;
    });

    Eigen::VectorXd grad;
    Eigen::SparseMatrix<double> H;
    tape.assemble_gradient_and_hessian(grad, H);

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(H);
    Eigen::VectorXd dx = solver.solve(-grad);

    x_flat += dx;
    Eigen::Map<Eigen::MatrixXd> x_new(x_flat.data(), n, 2);

    Eigen::MatrixXd x_prev = x;
    x = x_new;
    v = (x - x_prev) / dt;
}
