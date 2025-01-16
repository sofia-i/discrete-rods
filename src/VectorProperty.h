//
// Created by Sofia Iannicelli on 1/9/25.
//

#ifndef DISCRETE_RODS_VECTORPROPERTY_H
#define DISCRETE_RODS_VECTORPROPERTY_H

#include <Eigen/Eigen>
#include <cassert>

// forward declarations
template<typename T, int n_dim> class VectorProperty;
template<typename T, int n_dim> std::ostream& operator<< (
        std::ostream& os, const VectorProperty<T, n_dim>& np);

/**
 *
 * @tparam T type for property values (e.g. double, float, int)
 * @tparam n_dim number of dimensions of the property
 */
template <typename T, int n_dim> class VectorProperty {

    typedef Eigen::Vector<T, Eigen::Dynamic> VectorXt;

public:
    explicit VectorProperty(int n_members);
    VectorProperty(int n_members, VectorXt values);

    /**
     * Get property for target member (COPY)
     * @param index index of target member
     * @return property
     */
    inline Eigen::Vector<T, n_dim> get(int index) const;
    inline Eigen::VectorBlock<VectorXt, n_dim> get_ref(int index);

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
    void set(VectorXt all_vals);

    /**
     * Get VectorProperty of type T with n_members and n_dim, with all values initialized to 0
     * @return
     */
    static VectorProperty<T, n_dim> Zero(int n_members);

    friend std::ostream& operator<< <>(std::ostream& os, const VectorProperty<T, n_dim>& vp);

private:
    int n_members; //  number of members to hold property values for
    VectorXt values;

public:
    // operators
    Eigen::Matrix<T, n_dim, 1> operator[](int index) {
        return get_ref(index);
    }
    Eigen::Matrix<T, n_dim, 1> operator[](int index) const {
        return get(index);
    }

};

template <typename T, int n_dim>
VectorProperty<T, n_dim>::VectorProperty(int n_members): n_members(n_members), values(n_members) {}

template<typename T, int n_dim>
VectorProperty<T, n_dim> VectorProperty<T, n_dim>::Zero(const int n_members) {
    VectorXt zero_values = VectorXt::Zero(n_members * n_dim);
    return VectorProperty<T, n_dim>(n_members, zero_values);
}

template<typename T, int n_dim>
VectorProperty<T, n_dim>::VectorProperty(int n_members, VectorXt values) :
        n_members(n_members), values(values) {
    assert(values.size() == n_members * n_dim);
}

/*
template<typename T, int n_dim>
Eigen::VectorBlock<T, n_dim> VectorProperty<T, n_dim>::get(int index) {
    return values.segment(index * n_dim, n_dim);
}
 */

template<typename T, int n_dim>
Eigen::Vector<T, n_dim> VectorProperty<T, n_dim>::get(int index) const {
    return values.template segment<n_dim>(index * n_dim);
}

template<typename T, int n_dim>
Eigen::VectorBlock<Eigen::Vector<T, Eigen::Dynamic>, n_dim> VectorProperty<T, n_dim>::get_ref(int index) {
    return values.template segment<n_dim>(index * n_dim);
}

template <typename T, int n_dim>
void VectorProperty<T, n_dim>::set(int index, Eigen::Vector<T, n_dim> member_vals) {
    this->values.template segment<n_dim>(index * n_dim) = member_vals;
}

template <typename T, int n_dim>
void VectorProperty<T, n_dim>::set(VectorXt all_vals) {
    assert(all_vals.size() == this->values.size() &&
           "Set values parameter matches target length");
    this->values = all_vals;
}

template <typename T, int n_dim>
std::ostream& operator<<(std::ostream& os, const VectorProperty<T, n_dim>& vp) {
    os << "Vector property with " << vp.n_members << " members, ";
    os << "each property has " << n_dim << " dimensions" << std::endl;
    os << "contents: " << std::endl;


    std::vector<std::stringstream> stringstreams;
    stringstreams.reserve(n_dim);
    for(int row = 0; row < n_dim; ++row) {
        stringstreams.push_back(std::stringstream());
    }

    os << "{" << std::endl;
    Eigen::VectorXd block;
    for(int i = 0; i < vp.n_members; ++i) {
        block = vp.get(i);
        for(int row = 0; row < n_dim; ++row) {
            stringstreams[row] << "\t" << block[row] << "\t";
        }
    }
    for(int row = 0; row < n_dim; ++row) {
        os << stringstreams[row].str() << std::endl;
    }
    os << "}" << std::endl;

    return os;
}



#endif //DISCRETE_RODS_VECTORPROPERTY_H
