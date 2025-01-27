//
// Created by Sofia Iannicelli on 11/13/24.
//

#include "Eigen/Eigen"
#include <iostream>
#include "Rod.h"
#include <type_traits>

#include "MatrixProperty.h"

void test_mat_prop() {
    MatrixProperty<double, 3, 2> example = MatrixProperty<double, 3, 2>::Zero(5);

    Eigen::Matrix<double, 3, 2> ex_fill;
    ex_fill << 1, 2, 3, 4, 5, 6;
    auto ex_block = example.get(0);
    std::cout << "example block" << std::endl << ex_block << std::endl;
    ex_block = ex_fill;
    std::cout << "example block" << std::endl << ex_block << std::endl;
    example.get(0) = ex_block;

    std::cout << example << std::endl;

    // Eigen::Matrix<double, 3, 2*5> values = Eigen::Matrix<double, 3, 2*5>::Zero();
    // values.middleCols(0, 2) = ex_fill;
    // std::cout << values << std::endl;

}

void test_vec_prop() {
    VectorProperty<double, 3> example = VectorProperty<double, 3>::Zero(5);

    Eigen::Vector<double, 3> ex_fill;
    ex_fill << 1, 2, 3;
    Eigen::VectorBlock<Eigen::Vector<double, Eigen::Dynamic>, 3> ex_block = example.get_ref(0);
    std::cout << "example block" << std::endl << ex_block << std::endl;
    ex_block = ex_fill;
    std::cout << "example block" << std::endl << ex_block << std::endl;
    example.get(0) = ex_block;

    std::cout << example << std::endl;
}

std::vector<Rod*> createRods() {
    std::vector<Rod*> rods;

    const int nVerts = 5;

    // VectorProperty<double, 3> restPositions(nVerts);
    Eigen::VectorXd restPositions;
    restPositions.resize(nVerts * 3);
    restPositions.setZero();

    Eigen::Vector3d upVect = {0., 1., 0.};
    for(int i = 0; i < nVerts; ++i) {
        if (i > 0) {
            restPositions[i*3 + 1] = restPositions[(i-1)*3 + 1] + 1;
        }
    }

    std::vector<BoundaryCondition> boundCondit(nVerts, BoundaryCondition::FREE);
    Eigen::VectorXd initialVel = Eigen::Vector<double, nVerts * 3>::Zero();

    Rod *r1 = new Rod(nVerts, restPositions, restPositions, initialVel, boundCondit);

    rods.push_back(r1);

    return rods;
}

int main(int argc, char* argv[])
{
    // test_mat_prop();
    // test_vec_prop();

    std::vector<Rod*> rods = createRods();

    // VectorProperty<double, 3> prop(VectorProperty<double, 3>::Zero(4));
    // std::cout << prop << std::endl;

}