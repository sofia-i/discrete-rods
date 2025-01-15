//
// Created by Sofia Iannicelli on 11/13/24.
//

#ifndef DISCRETE_RODS_ROD_H
#define DISCRETE_RODS_ROD_H

#include <Eigen/Eigen>
#include "VectorProperty.h"

enum BoundaryCondition {
    FREE,
    CLAMPED,
    BODY_COUPLED
};

class Rod
{
public:
    explicit Rod(int vert_count);
    ~Rod() = default;

    /**
     * Sets rest positions and updates rest edges
     * @param rest_p Rest positions ordered contiguously
     */
    void set_rest(const Eigen::VectorXd& rest_p);

    void set_pos(const Eigen::VectorXd& p);
    void set_vel(const Eigen::VectorXd& v);
    void set_theta(const Eigen::VectorXd& th);

    void compute_bishop_frame();

    void set_initial_conditions(const Eigen::VectorXd& p, const Eigen::VectorXd& v);
    void set_boundary_conditions(std::vector<BoundaryCondition> vert_conditions);

    void precompute_material_curvatures(const VectorProperty<double, 3>& frame_m1,
                                        const VectorProperty<double, 3>& frame_m2);
    /**
     * Get curvature binormals per-vertex
     * @return
     */
    VectorProperty<double, 3> get_curvature_binormals();

private:
    void update_rest_edges();
    void update_edges();

    // inline

private:
    int vert_count;

    Eigen::Vector3d u0;  // "rail" vector

    VectorProperty<double, 3> rest_pos;
    VectorProperty<double, 3> rest_edges;

    std::vector<BoundaryCondition> boundary_conditions;  // vertex boundary conditions

    VectorProperty<double, 3> pos;
    VectorProperty<double, 3> vel;
    VectorProperty<double, 3> edges;

    Eigen::VectorXd theta;  // material frame angle

    /* TODO: switch to matrix? */
    VectorProperty<double, 3> bishop_frame_t;
    VectorProperty<double, 3> bishop_frame_u;
    VectorProperty<double, 3> bishop_frame_v;
    // VectorProperty<double, 2> adapted_frames;

    VectorProperty<double, 4> material_curvatures;

    bool rest_set;
    bool initialized;


};


#endif //DISCRETE_RODS_ROD_H
