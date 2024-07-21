#include <iostream>

#include "./eigen-3.4.0/Eigen/Dense"

int main() {
    // 定义一个 2x2 矩阵
    Eigen::Matrix2d mat;
    mat(0, 0) = 3;
    mat(1, 0) = 2.5;
    mat(0, 1) = -1;
    mat(1, 1) = 0.5;

    // 输出矩阵
    std::cout << "Here is the matrix mat:\n" << mat << std::endl;

    // 计算矩阵的逆
    Eigen::Matrix2d mat_inverse = mat.inverse();
    std::cout << "The inverse of mat is:\n" << mat_inverse << std::endl;

    // 矩阵乘法
    Eigen::Matrix2d result = mat * mat_inverse;
    std::cout << "The product of mat and its inverse is:\n"
              << result << std::endl;

    return 0;
}
