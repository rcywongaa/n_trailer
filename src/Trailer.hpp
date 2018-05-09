#pragma once

#include "drake/automotive/gen/simple_car_state.h"
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/state.h>

class Trailer : public drake::systems::LeafSystem<double>
{
    public:
        /// @param[in] length the length between preceding and current wheel axles
        Trailer(double length, const drake::automotive::SimpleCarState<double>& initial_position);
        void DoCalcTimeDerivatives(const drake::systems::Context<double>& context, drake::systems::ContinuousState<double>* derivatives) const override;
        void SetDefaultState(const drake::systems::Context<double>&, drake::systems::State<double>* state) const override;
        const drake::systems::InputPortDescriptor<double>& preceding_state_input() const;
        const drake::systems::InputPortDescriptor<double>& preceding_velocity_input() const;
        const drake::systems::OutputPort<double>& state_output() const;
        const drake::systems::OutputPort<double>& velocity_output() const;
    private:
        const drake::automotive::SimpleCarState<double>& get_state(const drake::systems::Context<double>& context) const;
        void set_output_state(const drake::systems::Context<double>& context, drake::automotive::SimpleCarState<double>* state) const;
        void calc_velocity(const drake::systems::Context<double>& context, drake::automotive::SimpleCarState<double>* state) const;

        double length;
        drake::automotive::SimpleCarState<double> initial_state;
        int preceding_state_idx;
        int preceding_velocity_idx;
        int state_idx;
        int velocity_idx;
};
