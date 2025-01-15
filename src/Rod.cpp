//
// Created by Sofia Iannicelli on 11/13/24.
//

#include "Rod.h"
#include "MathUtilities.h"

Rod::Rod(int vert_count) : vert_count(vert_count), rest_pos(vert_count),
                           pos(vert_count),vel(vert_count),
                           material_curvatures(vert_count),
                           rest_edges(vert_count-1),
                           edges(vert_count-1),
                           bishop_frame_t(vert_count-1),
                           bishop_frame_u(vert_count-1),
                           bishop_frame_v(vert_count-1),
                           // adapted_frames(vert_count-1),
                           rest_set(false), initialized(false) {
    boundary_conditions.resize(vert_count);
}

void Rod::set_rest(const Eigen::VectorXd& rest_p) {
    // set rest position
    // [BWR+08] Require: xbar_0 ... xbar_n+1
    this->rest_pos.set(rest_p);

    // update rest edges
    update_rest_edges();

    rest_set = true;
}

void Rod::update_rest_edges() {
    for(int i = 0; i < vert_count - 1; ++i) {
        rest_edges.set(i, rest_pos.get(i+1) - rest_pos.get(i));
    }
}

void Rod::update_edges() {
    for(int i = 0; i < vert_count - 1; ++i) {
        edges.set(i, pos.get(i+1) - pos.get(i));
    }
}

void Rod::set_pos(const Eigen::VectorXd& p) {
    this->pos.set(p);
}

void Rod::set_vel(const Eigen::VectorXd& v) {
    this->vel.set(v);
}

void Rod::set_theta(const Eigen::VectorXd& th) {
    this->theta = th;
}

void Rod::compute_bishop_frame() {
    Eigen::Vector3d t, u, v;
    Eigen::Matrix3d rotation;
    for(int i = 0; i < vert_count - 1; ++i) {
        t = edges.get(i).normalized();
        if(i == 0) {
            u = u0;
        }
        // check if t^{i-1} = t^i (the tangent vector is the same as the last)
        else if(double_equals(bishop_frame_t.get(i-1).dot(t), 1.0)) {
            // by convention, there is no rotation in this case
            u = bishop_frame_u.get(i-1);
        }
        else {
            // Find rotation from t^{i-1} to t^i
            rotation = MathUtilities::get_rotation(bishop_frame_t[i-1], t);
            // FIXME: probably need to pre-multiply? row-major?
            u = rotation * bishop_frame_u.get(i-1);
        }

        v = t.cross(u);

        bishop_frame_t.set(i, t);
        bishop_frame_u.set(i, u);
        bishop_frame_v.set(i, v);
    }
}

void Rod::set_initial_conditions(const Eigen::VectorXd& p, const Eigen::VectorXd& v) {
    // [BWR+08] Require: (x_0, xdot_0) ... (x_n+1, xdot__n+1)
    set_pos(p);
    set_vel(v);

    update_edges();

    // FIXME: better way to find u0?
    // Find u0 (a unit vector orthogonal to t0, uniquely defines bishop frame)
    Eigen::Vector3d t0 = edges.get(0);
    Eigen::Vector3d helper;
    do {
        helper = Eigen::Vector3d::Random();
    } while (helper.dot(t0) == 0.0);

    // [BWR+08] Require: u0
    this->u0 = (helper - helper.dot(t0) / t0.dot(t0) * t0).normalized();

    initialized = true;
}

void Rod::set_boundary_conditions(std::vector<BoundaryCondition> vert_conditions) {
    // [BWR+08] Require: Boundary conditions
    this->boundary_conditions = vert_conditions;
}

VectorProperty<double, 3> Rod::get_curvature_binormals() {
    VectorProperty<double, 3> curvature_binormals = VectorProperty<double, 3>(vert_count);
    for(int i = 1; i < this->vert_count - 1; ++i) {
        curvature_binormals[i] = 2 * edges[i-1].cross(edges[i]) / (
                rest_edges[i-1].norm() * rest_edges[i].norm() + rest_edges[i-1].dot(rest_edges[i]));
    }
    return curvature_binormals;
}

void Rod::precompute_material_curvatures(const VectorProperty<double, 3>& frame_m1,
                                         const VectorProperty<double, 3>& frame_m2) {
    // The curvature binormals κb_i are needed to compute material curvature
    VectorProperty<double, 3> curvature_binormals = get_curvature_binormals();
    // compute per-edge
    // TODO: what to do about first edge?
    for (int i = 1; i < vert_count - 1; ++i) {
        material_curvatures[i].segment(0, 2) = Eigen::Vector2d{curvature_binormals[i].dot(frame_m2[i - 1]),
                                                               -(curvature_binormals[i]).dot(frame_m1[i - 1])};
        material_curvatures[i].segment(2, 2) = Eigen::Vector2d{curvature_binormals[i].dot(frame_m2[i]),
                                                               -(curvature_binormals[i]).dot(frame_m1[i])};
    }
}












