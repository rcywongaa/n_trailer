#pragma once

#include "VisualState.hpp"

#include "drake/automotive/gen/simple_car_state.h"

// LeafSystem that converts SimpleCarState to VisualState
class VisualStateConverter : public drake::systems::LeafSystem<double>
{
    public:
        VisualStateConverter() :
            input_idx(this->DeclareVectorInputPort(drake::automotive::SimpleCarState<double>()).get_index()),
            output_idx(this->DeclareVectorOutputPort(VisualState<double>(), &VisualStateConverter::convert).get_index())
        {
            ;
        }
        const drake::systems::InputPortDescriptor<double>& simple_car_state_input() const
        {
            return drake::systems::System<double>::get_input_port(input_idx);
        }
        const drake::systems::OutputPort<double>& visual_state_output() const
        {
            return drake::systems::System<double>::get_output_port(output_idx);
        }
    private:
        void convert(const drake::systems::Context<double>& context, VisualState<double>* output) const
        {
            const drake::automotive::SimpleCarState<double>* const state = this->EvalVectorInput<drake::automotive::SimpleCarState>(context, input_idx);
            DRAKE_ASSERT(state != nullptr);
            output->set_x(state->x());
            output->set_y(state->y());
            output->set_heading(state->heading());
        }
        int input_idx;
        int output_idx;
};
