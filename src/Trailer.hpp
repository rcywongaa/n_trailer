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
        const drake::systems::InputPortDescriptor<double>& preceding_state_d_input() const;
        const drake::systems::OutputPort<double>& state_output() const;
        const drake::systems::OutputPort<double>& state_d_output() const;
    private:
        const drake::automotive::SimpleCarState<double>& get_state(const drake::systems::Context<double>& context) const;
        const drake::automotive::SimpleCarState<double>* const get_preceding_state(const drake::systems::Context<double>& context) const;
        double get_linear_velocity(const drake::systems::Context<double>& context) const;
        double get_joint_angle(const drake::systems::Context<double>& context) const;
        void get_output_state(const drake::systems::Context<double>& context, drake::automotive::SimpleCarState<double>* state) const;
        void get_output_state_d(const drake::systems::Context<double>& context, drake::automotive::SimpleCarState<double>* state) const;

        double length;
        drake::automotive::SimpleCarState<double> initial_state;
        int preceding_state_idx;
        int preceding_state_d_idx;
        int state_idx;
        int state_d_idx;
};
