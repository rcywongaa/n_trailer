#pragma once
#include <drake/systems/framework/leaf_system.h>
#include "drake/automotive/gen/driving_command.h"
#include "drake/common/drake_optional.h"

class SteeredTrailerController : public drake::systems::LeafSystem<double>
{
    public:
        SteeredTrailerController();
        const drake::systems::InputPortDescriptor<double>& preceding_state_input() const;
        //const drake::systems::InputPortDescriptor<double>& full_state_input() const;
        const drake::systems::InputPortDescriptor<double>& steer_angle_input() const;
        const drake::systems::InputPortDescriptor<double>& joint_angle_input() const;
        const drake::systems::OutputPort<double>& link_extension_velocity_output() const;
        const drake::systems::OutputPort<double>& steer_angle_output() const;
    protected:
        drake::optional<bool> DoHasDirectFeedthrough(int input_port, int output_port) const override;
    private:
        void calc_link_extension_velocity(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const;
        void calc_steer_angle(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const;
        double get_desired_velocity() const;
        double get_desired_steer_angle() const;

        int preceding_state_input_idx;
        //int full_state_input_idx;
        int steer_angle_input_idx;
        int joint_angle_input_idx;
        int link_extension_velocity_output_idx;
        int steer_angle_output_idx;
};

