#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

struct SteeredTrailerStateIndices {
    static const int x_idx = 0;
    static const int y_idx = 1;
    static const int heading_idx = 2;
    //static const int velocity_idx = 3;
    static const int steer_angle_idx = 3;
    static const int joint_angle_idx = 4;
    static const int num_coordinates = 5;
};

class SteeredTrailerState : public drake::systems::BasicVector<double>
{
    public:
        typedef SteeredTrailerStateIndices K;
        SteeredTrailerState() : drake::systems::BasicVector<double>(K::num_coordinates){
            this->SetFromVector(drake::VectorX<double>::Zero(K::num_coordinates));
        }
        const double& x() const { return this->GetAtIndex(K::x_idx); }
        void set_x(const double& x) { this->SetAtIndex(K::x_idx, x); }
        const double& y() const { return this->GetAtIndex(K::y_idx); }
        void set_y(const double& y) { this->SetAtIndex(K::y_idx, y); }
        const double& heading() const { return this->GetAtIndex(K::heading_idx); }
        void set_heading(const double& heading) { this->SetAtIndex(K::heading_idx, heading); }
        //const double& velocity() const { return this->GetAtIndex(K::velocity_idx); }
        //void set_velocity(const double& velocity) { this->SetAtIndex(K::velocity_idx, velocity); }
        const double& steer_angle() const { return this->GetAtIndex(K::steer_angle_idx); }
        void set_steer_angle(const double& steer_angle) { this->SetAtIndex(K::steer_angle_idx, steer_angle); }
        const double& joint_angle() const { return this->GetAtIndex(K::joint_angle_idx); }
        void set_joint_angle(const double& joint_angle) { this->SetAtIndex(K::joint_angle_idx, joint_angle); }
        SteeredTrailerState* DoClone() const override { return new SteeredTrailerState; }
};
