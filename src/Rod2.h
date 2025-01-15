//
// Created by Sofia Iannicelli on 1/8/25.
//

#ifndef DISCRETE_RODS_ROD2_H
#define DISCRETE_RODS_ROD2_H


#include <vector>
#include "Eigen/Eigen"

class Rod {

public:
    explicit Rod(int nVertices);
    virtual ~Rod();

    // getters
    std::vector<Eigen::Vector3d*> getVertices() const;
    // Eigen::MatrixXd getVertices() const;

    // setters
    void setVertices(std::vector<Eigen::Vector3d*> vertices);
    void setRestPositions(std::vector<Eigen::Vector3d*> restPositions);
    // void setVertices(Eigen::MatrixXd vertices);
    // void setVertices(std::vector<Eigen::Vector3d> vertices);
    // void setRestPositions(Eigen::MatrixXd restPositions);
    // void setRestPositions(std::vector<Eigen::Vector3d> restPositions);

    // functionality
    // void addVertices()
    void computeMaterialFrame();
    void computeRestCenterlineCurvature(Eigen::VectorXd& result);

    void computeCurvatureBinormal(std::vector<Eigen::Vector3d>& result);
    void computeMaterialCurvature(std::vector<std::vector<Eigen::Vector4d>>& result);
    void getRestMatCurvature(std::vector<Eigen::Vector2d>& result);

    // forces

    // Spring Force
    // fs(ks, cs)_i = ks(||ei|| - ||ei_bar||)ei_hat + cs(delta(vi) dot ei_hat) ei_hat
    Eigen::MatrixXd computeSpringForce(double k_spring, double c_damping);

private:
    /* Get column at index */
    Eigen::Vector3d getVect(Eigen::MatrixXd mat, int index);
    void setVect(Eigen::MatrixXd& mat, const Eigen::Vector3d& vect, int index);

private:
    int vertCount;

    // Assume vertices are connected contiguously?
    // TODO: decide storage
    // Eigen::MatrixXd _vertices_;
    std::vector<Eigen::Vector3d*> _vertices_;

    // material frames per edge
    // FIXME: Structure of arrays?
    std::vector<Eigen::Vector3d> tangents;
    std::vector<Eigen::Vector3d> material_frame_m1;
    std::vector<Eigen::Vector3d> material_frame_m2;

    // Eigen::MatrixXd rest_positions;
    // Eigen::MatrixXd velocities;
    std::vector<Eigen::Vector3d*> rest_positions;
    std::vector<Eigen::Vector4d*> rest_mat_curvatures;
    std::vector<Eigen::Vector3d*> velocities;
    // structure of arrays for matrix compute?

    Eigen::VectorXd centerline_curvature_rest;

};


#endif //DISCRETE_RODS_ROD2_H
