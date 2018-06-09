#include "TractorController.hpp"
#include "drake/automotive/gen/simple_car_state.h"

using namespace drake::systems;
using namespace drake::automotive;

TractorController::TractorController() :
    state_input_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    driving_command_output_idx(this->DeclareVectorOutputPort(DrivingCommand<double>(), &TractorController::create_driving_command).get_index())
{
    ;
}

const InputPortDescriptor<double>& TractorController::state_input() const
{
    return System<double>::get_input_port(state_input_idx);
}

const OutputPort<double>& TractorController::driving_command_output() const
{
    return System<double>::get_output_port(driving_command_output_idx);
}

void TractorController::create_driving_command(const Context<double>& context, DrivingCommand<double>* output) const
{
    const SimpleCarState<double>* const state = this->EvalVectorInput<SimpleCarState>(context, state_input_idx);
    DRAKE_ASSERT(state != nullptr);

    output->set_steering_angle(0.5);
    output->set_acceleration(0.5);
}
