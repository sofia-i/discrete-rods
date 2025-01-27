//
// Created by Sofia Iannicelli on 11/13/24.
//

#include "Rod.h"
#include "MathUtilities.h"

Rod::Rod(int vert_count) : vert_count(vert_count),
                           rest_pos(VectorProperty<double, 3>::Zero(vert_count)),
                           pos(VectorProperty<double, 3>::Zero(vert_count)),
                           vel(VectorProperty<double, 3>::Zero(vert_count)),
                           material_curvatures(VectorProperty<double, 2>::Zero(vert_count*2)),
                           rest_material_curvatures(VectorProperty<double, 2>::Zero(vert_count*2)),
                           rest_edges(VectorProperty<double, 3>::Zero(vert_count - 1)),
                           edges(VectorProperty<double, 3>::Zero(vert_count - 1)),
                           curvature_binorms(VectorProperty<double, 3>::Zero(vert_count)),
                           bishop_frame_t(VectorProperty<double, 3>::Zero(vert_count - 1)),
                           bishop_frame_u(VectorProperty<double, 3>::Zero(vert_count - 1)),
                           bishop_frame_v(VectorProperty<double, 3>::Zero(vert_count - 1)),
                           mat_m1(vert_count - 1), mat_m2(vert_count - 1)
                           // adapted_frames(vert_count-1),
                           {
    boundary_conditions.resize(vert_count);
}

Rod::Rod(int vert_count, const Eigen::VectorXd &rest_p, const Eigen::VectorXd &initial_p,
         const Eigen::VectorXd &initial_v, std::vector<BoundaryCondition>& bound_conditions) :
                vert_count(vert_count),
                rest_pos(vert_count, rest_p),
                pos(vert_count, initial_p),
                vel(vert_count, initial_v),
                boundary_conditions(bound_conditions),
                rest_edges(vert_count - 1),
                edges(vert_count - 1),
                material_curvatures(VectorProperty<double, 2>::Zero(vert_count*2)),
                rest_material_curvatures(VectorProperty<double, 2>::Zero(vert_count*2)),
                curvature_binorms(VectorProperty<double, 3>::Zero(vert_count)),
                theta(vert_count - 1),
                bishop_frame_t(VectorProperty<double, 3>::Zero(vert_count - 1)),
                bishop_frame_u(VectorProperty<double, 3>::Zero(vert_count - 1)),
                bishop_frame_v(VectorProperty<double, 3>::Zero(vert_count - 1)),
                mat_m1(vert_count - 1),
                mat_m2(vert_count - 1) {
    // set the rest edges from the rest positions
    update_rest_edges();
    // set the edges from the initial positions
    update_edges();

    // compute rest length
    rest_length.resize(vert_count);
    compute_rest_length();

    // [BWR+08] Require: u0
    this->u0 = find_u0();
    Eigen::Vector3d u0_rest = find_u0();
    // Eigen::Vector3d u0_rest = find_rest_u0();

    // curvature binormals
    this->curvature_binorms = get_curvature_binormals();
    VectorProperty<double, 3> rest_curvature_binorms = get_rest_curvature_binormals();

    // bishop frame
    compute_bishop_frame();
    // rest bishop frame
    VectorProperty<double, 3> rest_bishop_t(vert_count - 1);
    VectorProperty<double, 3> rest_bishop_u(vert_count - 1);
    VectorProperty<double, 3> rest_bishop_v(vert_count - 1);
    compute_bishop_frame(u0_rest, rest_edges, rest_bishop_t, rest_bishop_u, rest_bishop_v);

    // thetas
    theta.setZero();
    rest_theta.setZero();

    // material frame  // TODO: move?
    compute_material_frame(theta, bishop_frame_u, bishop_frame_v, mat_m1, mat_m2);
    VectorProperty<double, 3> rest_m1(vert_count - 1);
    VectorProperty<double, 3> rest_m2(vert_count - 1);
    compute_material_frame(rest_theta, rest_bishop_u, rest_bishop_v, rest_m1, rest_m2);

    // material curvatures  // TODO: move?
    this->material_curvatures = compute_material_curvatures(mat_m1, mat_m2, curvature_binorms);
    this->rest_material_curvatures = compute_material_curvatures(rest_m1, rest_m2, rest_curvature_binorms);

    quasistatic_material_frame_theta_update(rest_theta, rest_edges, rest_bishop_u, rest_bishop_v);
    // FIXME FIXME: what else does this (above) affect??
    quasistatic_material_frame_theta_update(theta, edges, bishop_frame_u, bishop_frame_v);
}

void Rod::compute_rest_length() {
    // denoted lbar_i in the paper
    // TODO: none for the last vert?
    for(int i = 0; i < vert_count; ++i) {
        double length = 0;
        if(i > 0) {
            length += rest_edges[i-1].norm();
        }
        length += rest_edges[i].norm();
        rest_length[i] = length;
    }
}

Eigen::Vector3d Rod::find_u0(const VectorProperty<double, 3>& edges) {
    // Find u0 (a unit vector orthogonal to t0, uniquely defines bishop frame)
    Eigen::Vector3d t0 = edges.get(0);
    Eigen::Vector3d helper;
    do {
        helper = Eigen::Vector3d::Random();
    } while (helper.dot(t0) == 0.0);

    // [BWR+08] Require: u0
    return (helper - helper.dot(t0) / t0.dot(t0) * t0).normalized();
}

Eigen::Vector3d Rod::find_u0() {
    find_u0(edges);
}

Eigen::Vector3d Rod::find_rest_u0() {
    find_u0(rest_edges);
}

void Rod::set_rest(const Eigen::VectorXd& rest_p) {
    // set rest position
    // [BWR+08] Require: xbar_0 ... xbar_n+1
    this->rest_pos.set(rest_p);

    // update rest edges
    update_rest_edges();
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

inline double Rod::get_twist(int index) {
    return theta[index] - theta[index - 1];
}

inline Eigen::Vector2d Rod::get_material_curvature(int i, int j) {
    assert(j >= i - 1 && j <= i);
    return material_curvatures[i*2 + (j-i)];
}

inline Eigen::Vector2d Rod::get_rest_material_curvature(int i, int j) {
    assert(j >= i - 1 && j <= i);
    return rest_material_curvatures[i*2 + (j-i)];
}

/*
 * Requires: vert_count, edges, u0
 */
void Rod::compute_bishop_frame(const Eigen::Vector3d& u0, const VectorProperty<double, 3>& edges,
                               VectorProperty<double, 3>& bishop_t,
                               VectorProperty<double, 3>& bishop_u,
                               VectorProperty<double, 3>& bishop_v) const {
    Eigen::Vector3d t, u, v;
    Eigen::Matrix3d rotation;
    for(int i = 0; i < vert_count - 1; ++i) {
        t = edges.get(i).normalized();
        if(i == 0) {
            u = u0;
        }
        // check if t^{i-1} = t^i (the tangent vector is the same as the last)
        else if(double_equals(bishop_t.get(i-1).dot(t), 1.0)) {
            // by convention, there is no rotation in this case
            u = bishop_u.get(i-1);
        }
        else {
            // Find rotation from t^{i-1} to t^i
            rotation = MathUtilities::get_rotation(bishop_t[i-1], t);
            // FIXME: probably need to pre-multiply? row-major?
            u = rotation * bishop_u.get(i-1);
        }

        v = t.cross(u);

        bishop_t.set(i, t);
        bishop_u.set(i, u);
        bishop_v.set(i, v);
    }
}

void Rod::compute_bishop_frame() {
    compute_bishop_frame(u0, edges, bishop_frame_t, bishop_frame_u, bishop_frame_v);
}

VectorProperty<double, 3> Rod::get_curvature_binormals(const VectorProperty<double, 3>& edges) const {
    VectorProperty<double, 3> curvature_binormals = VectorProperty<double, 3>(vert_count);
    for(int i = 1; i < this->vert_count - 1; ++i) {
        curvature_binormals[i] = 2 * edges[i-1].cross(edges[i]) / (
                rest_edges[i-1].norm() * rest_edges[i].norm() + edges[i-1].dot(edges[i]));
    }
    return curvature_binormals;
}

VectorProperty<double, 3> Rod::get_rest_curvature_binormals() {
    return get_curvature_binormals(rest_edges);
}

VectorProperty<double, 3> Rod::get_curvature_binormals() {
    return get_curvature_binormals(edges);
}

void Rod::compute_material_frame(const Eigen::VectorXd& theta,
                                 const VectorProperty<double, 3>& bishop_frame_u,
                                 const VectorProperty<double, 3>& bishop_frame_v,
                                 VectorProperty<double, 3>& mat_m1,
                                 VectorProperty<double, 3>& mat_m2) {
    // per edge
    for(int i = 0; i < vert_count - 1; ++i) {
        mat_m1[i] = cos(theta[i]) * bishop_frame_u[i] + sin(theta[i]) * bishop_frame_v[i];
        mat_m2[i] = -sin(theta[i]) * bishop_frame_u[i] + cos(theta[i]) * bishop_frame_v[i];
    }
}

VectorProperty<double, 2> Rod::compute_material_curvatures(const VectorProperty<double, 3>& mat_m1,
                                                              const VectorProperty<double, 3>& mat_m2,
                                                              const VectorProperty<double, 3>& curvature_binorms) {
    // The curvature binormals κb_i are needed to compute material curvature
    // VectorProperty<double, 3> curvature_binormals = get_curvature_binormals();
    // compute per-edge
    // TODO: what to do about first edge?
    VectorProperty<double, 2> material_curvatures(vert_count - 1);
    Eigen::Vector2d entry;
    for (int i = 1; i < vert_count - 1; ++i) {
        entry = Eigen::Vector2d(curvature_binorms[i].dot(mat_m2[i-1]), -curvature_binorms[i].dot(mat_m1[i-1]));
        material_curvatures.set(i*2, entry);

        entry = Eigen::Vector2d(curvature_binorms[i].dot(mat_m2[i]), -curvature_binorms[i].dot(mat_m1[i]));
        material_curvatures.set(i*2 + 1, entry);
        // Eigen::Vector2d entry (curvature_binorms[i].dot(mat_m2[i-1] + mat_m2[i]),
        //                       -curvature_binorms[i].dot(mat_m1[i-1] + mat_m1[i]));
        // material_curvatures.set(i, entry);
    }
    return material_curvatures;
}

Eigen::VectorXd Rod::compute_gradient(const VectorProperty<double, 2>& material_curvatures) {

    Eigen::VectorXd gradient;

    double deltaWj;
    double deltaWjp;
    // per-non-bounded edge
    for(int j = 0; j < vert_count - 1; ++j) {
        deltaWj = (1./rest_length[j]) * get_material_curvature(j, j).transpose() * MathUtilities::J * bStiff *
                  (get_material_curvature(j, j) - get_rest_material_curvature(j, j));
        deltaWjp = (1. / rest_length[j+1]) * (get_material_curvature(j+1, j)).transpose() * MathUtilities::J * bStiff *
                   (get_material_curvature(j+1, j) - get_rest_material_curvature(j+1, j));
        double mj = get_twist(j);
        double mjp = get_twist(j+1);
        gradient[j] = deltaWj + deltaWjp + 2 * k_bend * ((mj / rest_length[j]) - (mjp / rest_length[j+1]));
    }

    return gradient;
}

Eigen::SparseMatrix<double> Rod::compute_hessian(const VectorProperty<double, 2>& material_curvatures) {

    Eigen::SparseMatrix<double> hessian;
    typedef Eigen::Triplet<double> ET;
    std::vector<ET> tripletList;
    tripletList.reserve(3 * (vert_count - 1));

    double ddWj;
    double ddWjp;

    // per non-bounded edge
    for(int j = 0; j < vert_count - 1; ++j) {
        double a = 2 * k_bend / rest_length[j];
        double b = 2 * k_bend / rest_length[j+1];
        if(j > 0) {
            tripletList.push_back(ET(j, j-1, -a));
        }
        tripletList.push_back(ET(j, j+1, -b));

        ddWj = (1 / rest_length[j]) * get_material_curvature(j, j).transpose() * MathUtilities::J.transpose() *
               bStiff * MathUtilities::J * get_material_curvature(j, j);
        ddWj -= (1 / rest_length[j]) * get_material_curvature(j, j).transpose() * bStiff *
                (get_material_curvature(j, j) - get_rest_material_curvature(j ,j));

        ddWjp = (1 / rest_length[j+1]) * get_material_curvature(j+1, j).transpose() * MathUtilities::J.transpose() *
                bStiff * MathUtilities::J * get_material_curvature(j+1, j);
        ddWjp -= (1 / rest_length[j+1]) * get_material_curvature(j+1, j).transpose() * bStiff *
                 (get_material_curvature(j+1, j) - get_rest_material_curvature(j+1, j));

        tripletList.push_back(ET(j, j, a + b + ddWj + ddWjp));
    }

    hessian.setFromTriplets(tripletList.begin(), tripletList.end());

    return hessian;
}

void Rod::quasistatic_material_frame_theta_update(Eigen::VectorXd& theta,
                                                  const VectorProperty<double, 3>& edges,
                                                  const VectorProperty<double, 3>& bishop_u,
                                                  const VectorProperty<double, 3>& bishop_v) {
    // [BWR+08] 5.1 Quasistatic Update
    // compute theta to satisfy ∂E(Γ)/∂θj = 0
    // TODO
    // Use Newton's method to minimize the elastic energy with respect to the material frames
    Eigen::VectorXd working_theta = theta;

    // Newton's method: iteratively update θ until ∂E(Γ)/∂θj
    int iterationCount = 0;
    Eigen::VectorXd step;
    double epsilon = 1e-8;

    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> hessian;

    VectorProperty<double, 3> mat_m1(vert_count - 1);
    VectorProperty<double, 3> mat_m2(vert_count - 1);

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg;

    do {
        iterationCount++;

        compute_material_frame(theta, bishop_u, bishop_v, mat_m1, mat_m2);
        VectorProperty<double, 3> curvature_binorms = get_curvature_binormals(edges);
        const VectorProperty<double, 2>& material_curvatures = compute_material_curvatures(mat_m1, mat_m2,
                                                                                           curvature_binorms);

        gradient = compute_gradient(material_curvatures);
        hessian = compute_hessian(material_curvatures);

        cg.compute(hessian);
        step = cg.solve(gradient);

        working_theta += step;
    } while(step.norm() > epsilon);

    theta = working_theta;
}

Eigen::Vector3d Rod::get_nabla_curvbinom(int indexI, int indexK) {
    // ∇_i (kb)_k

}

Eigen::Vector3d Rod::get_nabla_holonomy() {
    
}

Eigen::Matrix<double, 2, 3> Rod::get_nabla_mcurv(int indexI, int indexK, int indexJ) {
    Eigen::Matrix<double, 2, 3> nabla_mcurv;

    Eigen::Matrix<double, 2, 3> helper;
    helper.row(0) = mat_m2[indexJ].transpose();
    helper.row(1) = - mat_m1[indexJ].transpose();

    nabla_mcurv = helper * get_nabla_curvbinom(indexI, indexK) -
            (MathUtilities::J * get_material_curvature(indexK, indexJ) * .transpose());
}

void Rod::computeCenterlineForces() {
    // -dE(Γ)/dx_i = -δE/δx_i + δE/δθ^n δθ^n/δx_i

    // δE/δx_i = sum_{k=1}^n (1/lbar_k) sum_{j=k-1}^k (∇_i ω_k^j)^T Bbar^j (ω_k^j - ωbar_k^j)
    VectorProperty<double, 3> partialEnergy_pos(VectorProperty<double, 3>::Zero(vert_count));
    for(int i = 0; i < this->vert_count; ++i) {
        Eigen::Vector3d term = Eigen::Vector3d::Zero();
        for(int k = 0; k < this->vert_count; ++k) {
            for(int j = k - 1; j <= k; ++j) {
                term += (1./rest_length[k]) * get_nabla_mcurv(k, j)
            }
        }
    }

    Eigen::VectorXd partialEnergy_theta(VectorProperty<double, 3>::Zero(vert_count));
    partialEnergy_theta.resize(vert_count - 1);
    for(int n = 0; n < vert_count; ++n) {
        double entry = (1./rest_length[n]) * material_curvatures[n].transpose() * MathUtilities::J *
                bStiff * (material_curvatures[n] - rest_material_curvatures[n]) +
                (2 * k_bend * get_twist(n)) / rest_length[n];
        partialEnergy_theta[n] = entry;
    }
}




