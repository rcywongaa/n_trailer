#pragma once

#include <drake/automotive/simple_car.h>
#include "drake/automotive/gen/simple_car_state.h"
#include <drake/systems/framework/diagram.h>

class Tractor : public drake::systems::Diagram<double>
{
    public:
        Tractor();
        const drake::systems::InputPortDescriptor<double>& driving_command_input() const;
        const drake::systems::OutputPort<double>& state_output() const;
        const drake::systems::OutputPort<double>& state_d_output() const;
    private:
        drake::automotive::SimpleCar<double>* simple_tractor;
        int driving_command_idx;
        int state_idx;
        int state_d_idx;
};
