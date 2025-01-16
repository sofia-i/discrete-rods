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

std::vector<Rod> create_rods() {
    std::vector<Rod> rods;

    int nVerts = 5;

    Rod *r1 = new Rod(nVerts);
    std::vector<Eigen::Vector3d> restPositions;
    Eigen::Vector3d upVect = {0., 1., 0.};
    for(int i = 0; i < nVerts; ++i) {
        if(i == 0) {
            restPositions.push_back({0., 0., 0.});
        }
        else {
            restPositions.push_back(restPositions[i-1] + upVect);
        }
    }
}

int main(int argc, char* argv[])
{
    // test_mat_prop();
    test_vec_prop();

    /*
    int nVerts = 5;

    Rod *r1 = new Rod(nVerts);
    std::vector<Eigen::Vector3d> restPositions;
    Eigen::Vector3d upVect = {0., 1., 0.};
    for(int i = 0; i < nVerts; ++i) {
        if(i == 0) {
            restPositions.push_back({0., 0., 0.});
        }
        else {
            restPositions.push_back(restPositions[i-1] + upVect);
        }
    }
     */



}