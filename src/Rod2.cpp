//
// Created by Sofia Iannicelli on 1/8/25.
//

#include "Rod2.h"

#include <utility>
#include <cmath>

const double EPSILON = 1.e-10;

Rod::Rod(int nVertices) {
    this->vertCount = nVertices;
    this->_vertices_.resize(vertCount);
    this->rest_positions.resize(vertCount);
    this->velocities.resize(vertCount);
    // this->_vertices_.resize(vertCount, 3);
    // this->rest_positions.resize(vertCount, 3);
    // this->velocities.resize(vertCount, 3);
}

std::vector<Eigen::Vector3d *> Rod::getVertices() const {
    return _vertices_;
}

void Rod::setVertices(std::vector<Eigen::Vector3d*> vertices) {
    this->_vertices_ = std::move(vertices);
}

void Rod::setRestPositions(std::vector<Eigen::Vector3d *> restPositions) {
    this->rest_positions = std::move(restPositions);
}

/*
Eigen::MatrixXd Rod::getVertices() const {
    return this->_vertices_;
}

void Rod::setVertices(Eigen::MatrixXd vertices) {
    this->_vertices_ = std::move(vertices); // FIXME: probably shouldn't be move if not pointers
}

void Rod::setVertices(std::vector<Eigen::Vector3d> vertices) {
    for(int i = 0; i < vertices.size(); ++i) {
        setVect(this->_vertices_, vertices[i], i);
    }
}

void Rod::setRestPositions(Eigen::MatrixXd restPositions) {
    this->rest_positions = std::move(restPositions); // FIXME: probably shouldn't be move if not pointers
}

void Rod::setRestPositions(std::vector<Eigen::Vector3d> restPositions) {
    for(int i = 0; i < restPositions.size(); ++i) {
        setVect(this->rest_positions, restPositions[i], i);
    }
}
 */

void Rod::computeMaterialCurvature(std::vector<std::vector<Eigen::Vector2d>>& matCurvatures) {
    // computed for edge-vertex pairs (j-i)
    std::vector<Eigen::Vector3d> curvatureBinorm;
    computeCurvatureBinormal(curvatureBinorm);

    // std::vector<std::vector<Eigen::Vector2d>> matCurvatures;
    Eigen::Vector2d matCurv;
    for(int i = 0; i < this->vertCount; ++i) {
        std::vector<Eigen::Vector2d> vertMatCurvs;
        for(int j = i - 1; j <= i; ++j) {
            matCurv[0] = curvatureBinorm[i].dot(material_frame_m2[j]);
            matCurv[1] = -(curvatureBinorm[i]).dot(material_frame_m1[j]);
            vertMatCurvs.push_back(matCurv);
        }
        matCurvatures.push_back(vertMatCurvs);
    }
}

void Rod::getRestMatCurvature(std::vector<Eigen::Vector2d> &result) {

}

void Rod::computeBendEnergy() {
    std::vector<std::vector<Eigen::Vector2d>> matCurvatures;
    computeMaterialCurvature(matCurvatures);

    Eigen::Vector3d bendEnergy;
    for(int i = 0; i < this->vertCount; ++i) {
        for(int j = i-1; j <=i; ++j) {
            (matCurvatures[i][j] - restMatCurvatures[i][j]).transpose()
        }
        bendEnergy *= 1. / (2*l[i])
    }
}

void Rod::computeRestCenterlineCurvature(Eigen::VectorXd &result) {
    // TODO: ?
    result.resize(vertCount);

    // curvature is associated with vertices
    // integrated curvature based on Discrete Elastic Rods :: Discrete curvature
    // TODO: what's the curvature at the endpoints?
    Eigen::Vector3d edge1;
    Eigen::Vector3d edge2;
    for(int i = 1; i < vertCount - 1; ++i) {
        edge1 = *(this->rest_positions[i]) - *(this->rest_positions[i-1]);
        edge2 = *(this->rest_positions[i+1]) - *(this->rest_positions[i]);

        double turningAngle = EIGEN_PI - acos(edge1.dot(edge2) / (edge1.norm() * edge2.norm())); // TODO - figure out if pi should be here

        double curvature = 2. * tan(turningAngle / 2.);
        result[i] = curvature;
    }
}

void Rod::computeCurvatureBinormal(std::vector<Eigen::Vector3d> &result) {
    result.resize(this->vertCount);
    // TODO: what about first and last?
    Eigen::Vector3d edge1;
    Eigen::Vector3d edge2;
    Eigen::Vector3d rest_edge1;
    Eigen::Vector3d rest_edge2;
    Eigen::Vector3d kbi;
    for(int i = 1; i < this->vertCount - 1; ++i) {
        edge1 = *(this->_vertices_[i]) - *(this->_vertices_[i-1]);
        edge2 = *(this->_vertices_[i+1]) - *(this->_vertices_[i]);
        rest_edge1 = *(this->rest_positions[i]) - *(this->rest_positions[i-1]);
        rest_edge2 = *(this->rest_positions[i+1]) - *(this->rest_positions[i]);

        kbi = 2. * edge1.cross(edge2);
        kbi /= rest_edge1.norm() * rest_edge2.norm() + edge1.dot(edge2);

        result.push_back(kbi);
    }
}

void Rod::computeMaterialFrame() {
    // using parallel transport

    // use u0, a unit vector orthogonal to t0
    // and transport it along to help define bishop frames
    // FIXME: better way to find an orthogonal vector?
    Eigen::Vector3d helperV = Eigen::Vector3d(1., 1., 1.);
    Eigen::Vector3d t0 = *(this->_vertices_[1]) - *(this->_vertices_[0]);
    if(helperV.dot(t0) == 0.0) helperV[0] += 0.5;

    Eigen::Vector3d u0 = helperV - helperV.dot(t0) * t0 / t0.dot(t0);

    // Set first values
    this->tangents[0] = t0.normalized();
    this->material_frame_m1[0] = u0; // TODO: normalize?
    this->material_frame_m2[0] = (u0.cross(t0)).normalized();

    Eigen::Vector3d uTransport = u0;

    Eigen::Vector3d edge, t, m1, m2, bitan;
    Eigen::AngleAxis<double> rotation;
    double theta;

    // iterate across rod
    for(int i = 1; i < this->_vertices_.size() - 1; ++i) {
        // edge ei = x_(i+1) - x_i
        edge = *(this->_vertices_[i+1]) - *(this->_vertices_[i]);
        // Compute tangent (edge direction) (adapted to centerline)
        t = edge / edge.norm();
        this->tangents[i] = t;

        // Find material frame
        // Compute bitangent
        bitan = t.cross(this->tangents[i-1]);  // should be normalized
        // if the length of the bitangent is 0, the tangent vectors are the same, so transport the vector exactly
        if(bitan.norm() < EPSILON) {
            this->material_frame_m1[i] = uTransport;
        }
        else {
            bitan = bitan.normalized();
            theta = acos(t.dot(this->tangents[i-1]));
            // rotate uTransport by theta around bitan
            rotation = Eigen::AngleAxisd(theta, bitan);
            uTransport = rotation * uTransport;
            this->material_frame_m1[i] = uTransport.normalized();
        }
        // Find final material frame perpendicular to tan, matframe1
        this->material_frame_m2[i] = t.cross(uTransport).normalized();
    }
}

Eigen::MatrixXd Rod::computeSpringForce(double k_spring, double c_damping) {
    // std::vector<Eigen::Vector3d> springForces;
    Eigen::MatrixXd springForces(this->vertCount, 3);
    Eigen::Vector3d edge; // TODO: save edges?
    Eigen::Vector3d rest_edge;
    Eigen::Vector3d delta_vel;
    Eigen::Vector3d springForce;
    for(int i = 0; i < this->vertCount - 1; ++i) {
        // edge = getVect(_vertices_, i + 1) - getVect(_vertices_, i);
        // rest_edge = getVect(rest_positions, i + 1) - getVect(rest_positions, i);
        // delta_vel = getVect(velocities, i + 1) - getVect(velocities, i);
        edge = *(this->_vertices_[i+1]) - *(this->_vertices_[i]);
        rest_edge = *(this->rest_positions[i+1]) - *(this->rest_positions[i]);
        delta_vel = *(this->velocities[i+1]) - *(this->velocities[i]);
        // rest_edge = this->rest_positions[i+1] - this->rest_positions[i];
        springForce = k_spring * (edge.norm() - rest_edge.norm()) * edge.normalized();
        springForce += c_damping * delta_vel.dot( edge.normalized()) * edge.normalized();
        setVect(springForces, springForce, i);
        // springForces.push_back(springForce);
    }
    return springForces;
}

Eigen::Vector3d Rod::getVect(Eigen::MatrixXd mat, int index) {
    return mat.col(index);
}

void Rod::setVect(Eigen::MatrixXd &mat, const Eigen::Vector3d& vect, int index) {
    mat.col(index) = vect;
}