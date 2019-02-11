#pragma once

#include <iostream>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/pose_vector.h>
#include "drake/automotive/gen/simple_car_state.h"

// LeafSystem that converts SimpleCarState to visualizable state
class SimpleCarStateToPose : public drake::systems::LeafSystem<double>
{
    public:
        SimpleCarStateToPose();
        const drake::systems::InputPort<double>& simple_car_state_input() const;
        const drake::systems::OutputPort<double>& pose_output() const;
    private:
        void convert(const drake::systems::Context<double>& context, drake::systems::rendering::PoseVector<double>* output) const;
        int input_idx;
        int output_idx;
};
