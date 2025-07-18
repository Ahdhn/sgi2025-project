#include <Eigen/SparseCore>
#include <stdexcept>
#include <tuple>
#include <igl/triangulated_grid.h>
#include <vector>
using namespace std;

double Inertia(Eigen::MatrixXd & V, Eigen::MatrixXd & V_tilde, Eigen::VectorXd M){
    //check dimensions
    int num_vertices = V.rows();
    int dimensions = V.cols();

    int num_vertices2 = V_tilde.rows();
    int dimensions2 = V_tilde.cols();

    if (num_vertices != num_vertices2 || dimensions2 != dimensions){
         throw std::invalid_argument("Matrix dimensions do not match");
    }
    
    double sum = 0.0;
    for (int i = 0; i < num_vertices; i++){
        Eigen::RowVectorXd diff = V.row(i) - V_tilde.row(i);
        double squared_dist = diff.dot(diff);
        sum += 0.5 * M(i) * squared_dist;
    }
    return sum;
}

Eigen::MatrixXd grad(Eigen::MatrixXd & V, Eigen::MatrixXd & V_tilde, Eigen::VectorXd M){
    // Check dimensions
    int num_vertices = V.rows();
    int dimensions = V.cols();
    
    if (V.rows() != V_tilde.rows() || V.cols() != V_tilde.cols() || M.size() != num_vertices){
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    
    // Initialize gradient matrix with zeros
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(num_vertices, dimensions);
    
    // Compute gradient: g[i] = m[i] * (x[i] - x_tilde[i])
    for (int i = 0; i < num_vertices; i++){
        g.row(i) = M(i) * (V.row(i) - V_tilde.row(i));
    }
    
    return g;
}

std::tuple<Eigen::VectorXi, Eigen::VectorXi, Eigen::VectorXd> hess(
    Eigen::MatrixXd & V, 
    Eigen::MatrixXd & V_tilde, 
    Eigen::VectorXd M
){
    // Check dimensions
    int num_vertices = V.rows();
    int dimensions = V.cols();
    
    if (V.rows() != V_tilde.rows() || V.cols() != V_tilde.cols() || M.size() != num_vertices){
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    
    int total_size = num_vertices * dimensions;
    
    // Initialize IJV arrays
    Eigen::VectorXi I(total_size);
    Eigen::VectorXi J(total_size);
    Eigen::VectorXd V_vals(total_size);
    
    // Fill IJV arrays - diagonal matrix with masses
    for (int i = 0; i < num_vertices; i++){
        for (int d = 0; d < dimensions; d++){
            int idx = i * dimensions + d;
            I(idx) = idx;           // Row index
            J(idx) = idx;           // Column index (diagonal)
            V_vals(idx) = M(i);     // Value (mass)
        }
    }
    
    return std::make_tuple(I, J, V_vals);
}