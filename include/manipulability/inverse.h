#ifndef MANIPULABILITY_INVERSE_H
#define MANIPULABILITY_INVERSE_H

#include "eigen3/Eigen/Dense"

namespace manipulability {

// 例：ヤコビ行列を計算する関数（実装省略）
Eigen::Matrix<double, 6, 7> calcJacobian(const Eigen::VectorXd &joint_position);

// 例：逆行列を計算する関数（実装省略）
Eigen::Matrix<double, 7, 6> calcJacobianInverse(const Eigen::MatrixXd &jacobian);

}  // namespace manipulability

#endif  // MANIPULABILITY_INVERSE_H
