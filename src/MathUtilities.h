//
// Created by Sofia Iannicelli on 1/14/25.
//

#ifndef DISCRETE_RODS_MATHUTILITIES_H
#define DISCRETE_RODS_MATHUTILITIES_H

#include <cmath>
#include <cfloat>

inline bool double_equals(double inValue, double inCompareTo)
{
    return std::fabs(inValue - inCompareTo) <= DBL_EPSILON;
}

class MathUtilities {
public:
    const static Eigen::Matrix2d J;

    static Eigen::Matrix3d get_rotation(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2) {
        // Based on https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        Eigen::Vector3d v = vector1.cross(vector2);
        Eigen::Matrix3d skew_cross_prod;
        skew_cross_prod << 0, -v[2], v[1],
                        v[2], 0, -v[0],
                        -v[1], v[0], 0;

        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + skew_cross_prod +
                skew_cross_prod * skew_cross_prod * (1. / (1. + vector1.dot(vector2)));

        return rotation;
    }
};

const Eigen::Matrix2d MathUtilities::J = Eigen::Matrix2d {{0, -1}, {1, 0}};

#endif //DISCRETE_RODS_MATHUTILITIES_H
