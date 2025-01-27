//
// Created by Sofia Iannicelli on 11/13/24.
//

#ifndef DISCRETE_RODS_ROD_H
#define DISCRETE_RODS_ROD_H

#include <Eigen/Eigen>
#include "VectorProperty.h"
#include "MatrixProperty.h"

enum BoundaryCondition {
    FREE,
    CLAMPED,
    BODY_COUPLED
};

class Rod
{
public:
    Rod(int vert_count, const Eigen::VectorXd& rest_p,
        const Eigen::VectorXd& initial_p, const Eigen::VectorXd& initial_v,
        std::vector<BoundaryCondition>& vert_conditions);
    ~Rod() = default;

    void computeCenterlineForces();

private:
    void update_rest_edges();
    void update_edges();
    Eigen::Vector3d find_u0();
    Eigen::Vector3d find_rest_u0();
    Eigen::Vector3d find_u0(const VectorProperty<double, 3>& edges);

private:
    explicit Rod(int vert_count);

    /**
     * Sets rest positions and updates rest edges
     * @param rest_p Rest positions ordered contiguously
     */
    void set_rest(const Eigen::VectorXd& rest_p);

    void set_pos(const Eigen::VectorXd& p);
    void set_vel(const Eigen::VectorXd& v);
    void set_theta(const Eigen::VectorXd& th);

    void compute_bishop_frame();
    void compute_bishop_frame(const Eigen::Vector3d& u0, const VectorProperty<double, 3>& edges,
                              VectorProperty<double, 3>& bishop_t, VectorProperty<double, 3>& bishop_u,
                              VectorProperty<double, 3>& bishop_v) const;

    void compute_rest_length();

    void compute_material_frame(const Eigen::VectorXd& theta,
                                const VectorProperty<double, 3>& bishop_frame_u,
                                const VectorProperty<double, 3>& bishop_frame_v,
                                VectorProperty<double, 3>& mat_m1,
                                VectorProperty<double, 3>& mat_m2);

    void quasistatic_material_frame_theta_update(Eigen::VectorXd& theta,
                                                 const VectorProperty<double, 3>& edges,
                                                 const VectorProperty<double, 3>& bishop_u,
                                                 const VectorProperty<double, 3>& bishop_v);

    // void precompute_material_curvatures();
    VectorProperty<double, 2> compute_material_curvatures(const VectorProperty<double, 3>& mat_m1,
                                                          const VectorProperty<double, 3>& mat_m2,
                                                          const VectorProperty<double, 3>& curvature_binorms);
    /**
     * Get curvature binormals per-vertex
     * @return
     */
    VectorProperty<double, 3> get_curvature_binormals(const VectorProperty<double, 3>& edges) const;
    VectorProperty<double, 3> get_curvature_binormals();
    VectorProperty<double, 3> get_rest_curvature_binormals();

    Eigen::VectorXd compute_gradient(const VectorProperty<double, 2>& material_curvatures);
    Eigen::SparseMatrix<double> compute_hessian(const VectorProperty<double, 2>& material_curvatures);

protected:
    inline double get_twist(int index);

private:
    int vert_count;

    Eigen::Vector3d u0;  // "rail" vector  // TODO: rename rail?

    VectorProperty<double, 3> rest_pos;
    VectorProperty<double, 3> rest_edges;

    std::vector<BoundaryCondition> boundary_conditions;  // vertex boundary conditions

    VectorProperty<double, 3> pos;
    VectorProperty<double, 3> vel;
    VectorProperty<double, 3> edges;

    double k_bend;   // bending modulus (beta)  // FIXME: per vertex
    Eigen::Matrix2d bStiff;  // bending stiffness  // FIXME: per vertex

    Eigen::VectorXd theta;  // material frame angle
    Eigen::VectorXd rest_theta;  // material frame angle
    Eigen::VectorXd rest_length;  // vertex-wise rest legnth

    VectorProperty<double, 3> curvature_binorms;  // curvature binormals

    /* TODO: switch to matrix? */
    // the bishop frame is the twist-free reference frame
    VectorProperty<double, 3> bishop_frame_t;
    VectorProperty<double, 3> bishop_frame_u;
    VectorProperty<double, 3> bishop_frame_v;
    // VectorProperty<double, 2> adapted_frames;

    VectorProperty<double, 3> mat_m1;
    VectorProperty<double, 3> mat_m2;

    VectorProperty<double, 2> material_curvatures;
    VectorProperty<double, 2> rest_material_curvatures;

    // bool rest_set;
    // bool initialized;

private:
    Eigen::Matrix<double, 2, 3> get_nabla_mcurv(int indexI, int indexK, int indexJ);
    Eigen::Vector3d get_nabla_curvbinom(int indexI, int indexK);
    Eigen::Vector3d get_nabla_holonomy();

    inline Eigen::Vector2d get_material_curvature(int i, int j);
    inline Eigen::Vector2d get_rest_material_curvature(int i, int j);

};


#endif //DISCRETE_RODS_ROD_H
