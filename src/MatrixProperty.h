//
// Created by Sofia Iannicelli on 1/14/25.
//

#ifndef DISCRETE_RODS_MATRIXPROPERTY_H
#define DISCRETE_RODS_MATRIXPROPERTY_H

#include <Eigen/Eigen>
#include <iostream>

/**
 *
 * @tparam T type for property values (e.g. double, float, int)
 * @tparam n_rows number of rows of each property
 * @tparam n_cols number of columns of each property
 */
template <typename T, int n_members, int n_rows, int n_cols> class MatrixProperty {
public:
    MatrixProperty();
    explicit MatrixProperty(Eigen::Matrix<T, n_rows, n_cols*n_members> values);

    /**
     * Get property for target member
     * @param index index of target member
     * @return
     */
    inline Eigen::Block<Eigen::Matrix<T, n_rows, n_cols * n_members>, n_rows, -1, true> get(int index) {
        return values.middleCols(index*n_cols, n_cols);;
    }
    inline typename Eigen::Matrix<T, n_rows, n_cols> get(int index) const {
        return values.middleCols(index*n_cols, n_cols);;
    }
    Eigen::Matrix<T, n_rows, n_cols*n_members> get_values();

    friend std::ostream& operator<<(std::ostream& os, const MatrixProperty<T, n_members, n_rows, n_cols>& mp) {
        os << "Matrix property with " << n_members << " members" << std::endl;
        os << "property rows: " << n_rows << " columns: " << n_cols << std::endl;
        os << "contents: " << std::endl;

        std::stringstream row0;
        std::stringstream row1;
        std::stringstream row2;
        os << "{" << std::endl;
        Eigen::MatrixXd block;
        for(int i = 0; i < n_members; ++i) {
            block = mp.get(i);
            for(auto col : block.colwise()) {
                row0 << "\t" << col[0];
                row1 << "\t" << col[1];
                row2 << "\t" << col[2];
            }
            row0 << "\t\t";
            row1 << "\t\t";
            row2 << "\t\t";
        }
        os << row0.str() << std::endl;
        os << row1.str() << std::endl;
        os << row2.str() << std::endl;
        os << "}" << std::endl;

        return os;
    }

    static MatrixProperty<T, n_members, n_rows, n_cols> Zero();

private:
    Eigen::Matrix<T, n_rows, n_cols*n_members> values;

};

template<typename T, int n_members, int n_rows, int n_cols>
MatrixProperty<T, n_members, n_rows, n_cols> MatrixProperty<T, n_members, n_rows, n_cols>::Zero() {
    Eigen::Matrix<T, n_rows, n_cols*n_members> values = Eigen::Matrix<T, n_rows, n_cols*n_members>::Zero();
    return MatrixProperty<T, n_members, n_rows, n_cols>(values);
}

template<typename T, int n_members, int n_rows, int n_cols>
MatrixProperty<T, n_members, n_rows, n_cols>::MatrixProperty() = default;

template<typename T, int n_members, int n_rows, int n_cols>
MatrixProperty<T, n_members, n_rows, n_cols>::MatrixProperty(Eigen::Matrix<T, n_rows, n_cols*n_members> values) {
    this->values = values;
    std::cout << "values constructor. values rows: " << values.rows() << ", values cols: " << values.cols() << std::endl;
}

/*
template<typename T, int n_members, int n_rows, int n_cols>
inline Eigen::Matrix<T, n_rows, n_cols> MatrixProperty<T, n_members, n_rows, n_cols>::get(int index) const {
    // return values.template block<n_rows, n_cols>(index * n_cols, 0);
    return values.middleCols(index*n_cols, n_cols);
}

template<typename T, int n_members, int n_rows, int n_cols>
inline typename Eigen::Matrix<T, n_rows, n_cols> MatrixProperty<T, n_members, n_rows, n_cols>::get(int index) {
    // return values.template block<n_rows, n_cols>(index * n_cols, 0);
    return values.middleCols(index*n_cols, n_cols);
}
 */

template<typename T, int n_members, int n_rows, int n_cols>
Eigen::Matrix<T, n_rows, n_cols*n_members> MatrixProperty<T, n_members, n_rows, n_cols>::get_values() {
    return values;
}


#endif //DISCRETE_RODS_MATRIXPROPERTY_H
