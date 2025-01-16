//
// Created by Sofia Iannicelli on 1/14/25.
//

#ifndef DISCRETE_RODS_MATRIXPROPERTY_H
#define DISCRETE_RODS_MATRIXPROPERTY_H

#include <Eigen/Eigen>
#include <iostream>
#include <cassert>

// forward declarations
template<typename T, int n_rows, int n_cols> class MatrixProperty;  // pre-declare the template class itself
template<typename T, int n_rows, int n_cols> std::ostream& operator<< (
        std::ostream& os, const MatrixProperty<T, n_rows, n_cols>& mp);

/*
template <typename T, int n_rows, int n_cols>
class MatrixProperty;

template <typename T, int n_rows, int n_cols>
std::ostream& operator<<(std::ostream& os, const MatrixProperty<T, n_rows, n_cols>& mp);
*/

/**
 *
 * @tparam T type for property values (e.g. double, float, int)
 * @tparam n_rows number of rows of each property
 * @tparam n_cols number of columns of each property
 */
template <typename T, int n_rows, int n_cols> class MatrixProperty {
public:
    explicit MatrixProperty(int n_members);
    // explicit MatrixProperty(Eigen::Matrix<T, n_rows, Eigen::Dynamic> values);
    MatrixProperty(int n_members, Eigen::Matrix<T, n_rows, Eigen::Dynamic> values);

    /**
     * Get property for target member
     * @param index index of target member
     * @return
     */
    inline Eigen::Block<Eigen::Matrix<T, n_rows, Eigen::Dynamic>, n_rows, -1, true> get(int index);
    inline typename Eigen::Matrix<T, n_rows, n_cols> get(int index) const;
    Eigen::Matrix<T, n_rows, Eigen::Dynamic> get_values();

    friend std::ostream& operator<< <>(std::ostream& os, const MatrixProperty<T, n_rows, n_cols>& mp);

    static MatrixProperty<T, n_rows, n_cols> Zero(int n_members);

private:
    int n_members; // number of members to hold the property for
    Eigen::Matrix<T, n_rows, Eigen::Dynamic> values;

};

// Default constructor
template<typename T, int n_rows, int n_cols>
MatrixProperty<T, n_rows, n_cols>::MatrixProperty(int n_members) : n_members(n_members),
                                                                   values(n_rows, n_cols * n_members) {}

template<typename T, int n_rows, int n_cols>
MatrixProperty<T, n_rows, n_cols> MatrixProperty<T, n_rows, n_cols>::Zero(int n_members) {
    Eigen::Matrix<T, n_rows, Eigen::Dynamic> values = Eigen::Matrix<T, n_rows, Eigen::Dynamic>::Zero(n_rows, n_members * n_cols);
    return MatrixProperty<T, n_rows, n_cols>(n_members, values);
}

template<typename T, int n_rows, int n_cols>
MatrixProperty<T, n_rows, n_cols>::MatrixProperty(int n_members, Eigen::Matrix<T, n_rows, Eigen::Dynamic> values) :
        n_members(n_members), values(values) {
    assert(values.size() == n_members * n_cols * n_rows &&
           "number of members n_members and length of values passed in do not match.");
}

/*
template<typename T, int n_rows, int n_cols>
MatrixProperty<T, n_rows, n_cols>::MatrixProperty(Eigen::Matrix<T, n_rows, Eigen::Dynamic> values) : values(values) {
    this->n_members = (values.size() / n_rows) / n_cols;
}
 */

template<typename T, int n_rows, int n_cols>
Eigen::Block<Eigen::Matrix<T, n_rows, Eigen::Dynamic>, n_rows, -1, true>
MatrixProperty<T, n_rows, n_cols>::get(int index) {
    return values.middleCols(index*n_cols, n_cols);
}

template<typename T, int n_rows, int n_cols>
Eigen::Matrix<T, n_rows, n_cols> MatrixProperty<T, n_rows, n_cols>::get(int index) const {
    return values.middleCols(index*n_cols, n_cols);
}

template<typename T, int n_rows, int n_cols>
Eigen::Matrix<T, n_rows, Eigen::Dynamic> MatrixProperty<T, n_rows, n_cols>::get_values() {
    return values;
}

template<typename T, int n_rows, int n_cols>
std::ostream& operator<<(std::ostream& os, const MatrixProperty<T, n_rows, n_cols>& mp) {
    os << "Matrix property container with " << mp.n_members << " members" << std::endl;
    os << "Each property has " << n_rows << " rows and " << n_cols << " columns" << std::endl;
    os << "contents: " << std::endl;

    std::vector<std::stringstream> stringstreams;
    stringstreams.reserve(n_rows);
    for(int row = 0; row < n_rows; ++row) {
        stringstreams.push_back(std::stringstream());
    }

    os << "{" << std::endl;
    Eigen::MatrixXd block;
    for(int i = 0; i < mp.n_members; ++i) {
        block = mp.get(i);
        for(auto col : block.colwise()) {
            for(int row = 0; row < n_rows; ++row) {
                stringstreams[row] << "\t" << col[row];
            }
        }
        for(int row = 0; row < n_rows; ++row) {
            stringstreams[row] << "\t";
        }
    }
    for(int row = 0; row < n_rows; ++row) {
        os << stringstreams[row].str() << std::endl;
    }
    os << "}" << std::endl;

    return os;
}


#endif //DISCRETE_RODS_MATRIXPROPERTY_H
