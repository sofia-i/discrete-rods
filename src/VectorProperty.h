//
// Created by Sofia Iannicelli on 1/9/25.
//

#ifndef DISCRETE_RODS_VECTORPROPERTY_H
#define DISCRETE_RODS_VECTORPROPERTY_H

#include <Eigen/Eigen>
#include <cassert>

/**
 *
 * @tparam T type for property values (e.g. double, float, int)
 * @tparam n_dim number of dimensions of the property
 */
template <typename T, int n_members, int n_dim> class VectorProperty {
public:
    VectorProperty();
    explicit VectorProperty(Eigen::Vector<T, n_members * n_dim> values);

    /**
     * Get property for target member
     * @param index index of target member
     * @return property
     */
    inline Eigen::VectorBlock<T, n_dim> get(int index);
    inline Eigen::Vector<T, n_dim> get(int index) const;

    /**
     * Set property at target index
     * @param index index of target member
     * @param member_vals property values for target
     */
    void set(int index, Eigen::Vector<T, n_dim> member_vals);
    /**
     * Set all property values at once. Assume contiguous array
     * @param all_vals member properties arranged in contiguous array
     */
    void set(Eigen::Vector<T, n_members * n_dim> all_vals);

    /**
     * Get VectorProperty of type T with n_members and n_dim, with all values initialized to 0
     * @return
     */
    static VectorProperty<T, n_members, n_dim> Zero();

    friend std::ostream& operator<<(std::ostream& os, const VectorProperty<T, n_members, n_dim>& vp);

private:
    Eigen::Vector<T, n_members * n_dim> values;

public:
    // operators
    Eigen::Matrix<T, n_dim, 1> operator[](int index) {
        return get(index);
    }
    Eigen::Matrix<T, n_dim, 1> operator[](int index) const {
        return get(index);
    }

};

template <typename T, int n_members, int n_dim>
VectorProperty<T, n_members, n_dim>::VectorProperty() = default;

template<typename T, int n_members, int n_dim>
VectorProperty<T, n_members, n_dim> VectorProperty<T, n_members, n_dim>::Zero() {
    Eigen::Vector<T, n_members * n_dim> zero_values = Eigen::Vector<T, n_members*n_dim>::Zero();
    return VectorProperty<T, n_members, n_dim>(zero_values);
}

template<typename T, int n_members, int n_dim>
VectorProperty<T, n_members, n_dim>::VectorProperty(Eigen::Vector<T, n_members * n_dim> values) {
    this->values = values;
}

template<typename T, int n_members, int n_dim>
Eigen::VectorBlock<T, n_dim> VectorProperty<T, n_members, n_dim>::get(int index) {
    return values.segment(index * n_dim, n_dim);
}

template<typename T, int n_members, int n_dim>
Eigen::Vector<T, n_dim> VectorProperty<T, n_members, n_dim>::get(int index) const {
    return values.segment(index * n_dim, n_dim);
}

template <typename T, int n_members, int n_dim>
void VectorProperty<T, n_members, n_dim>::set(int index, Eigen::Vector<T, n_dim> member_vals) {
    this->values.segment(index * n_dim, n_dim) = member_vals;
}

template <typename T, int n_members, int n_dim>
void VectorProperty<T, n_members, n_dim>::set(Eigen::Vector<T, n_members * n_dim> all_vals) {
    assert(all_vals.size() == this->values.size() &&
           "Set values parameter matches target length");
    this->values = all_vals;
}

template <typename T, int n_members, int n_dim>
std::ostream& operator<<(std::ostream& os, const VectorProperty<T, n_members, n_dim>& vp) {
    os << "Vector property with " << n_members << " members" << std::endl;
    os << "Property has " << n_dim << "dimensions" << std::endl;
    os << "contents: " << std::endl;

    // TODO:
}



#endif //DISCRETE_RODS_VECTORPROPERTY_H
