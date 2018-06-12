#include "SteeredTrailerController.hpp"
#include "drake/automotive/gen/simple_car_state.h"

using namespace drake::systems;
using namespace drake::automotive;

SteeredTrailerController::SteeredTrailerController() :
    tractor_state_input_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    link_velocity_output_idx(this->DeclareVectorOutputPort(BasicVector<double>(1), &SteeredTrailerController::calc_link_velocity).get_index()),
    steer_angle_output_idx(this->DeclareVectorOutputPort(BasicVector<double>(1), &SteeredTrailerController::calc_steer_angle).get_index())
{
    ;
}               

const InputPortDescriptor<double>& SteeredTrailerController::tractor_state_input() const
{
    return System<double>::get_input_port(tractor_state_input_idx);
}

const OutputPort<double>& SteeredTrailerController::link_velocity_output() const
{
    return System<double>::get_output_port(link_velocity_output_idx);
}

const OutputPort<double>& SteeredTrailerController::steer_angle_output() const
{
    return System<double>::get_output_port(steer_angle_output_idx);
}

void SteeredTrailerController::calc_link_velocity(const Context<double>& context, BasicVector<double>* output) const
{
    const SimpleCarState<double>* const state = this->EvalVectorInput<SimpleCarState>(context, tractor_state_input_idx);
    DRAKE_ASSERT(state != nullptr);

    output->get_mutable_value()(0) = state->velocity();
}

void SteeredTrailerController::calc_steer_angle(const Context<double>& context, BasicVector<double>* output) const
{
    const SimpleCarState<double>* const state = this->EvalVectorInput<SimpleCarState>(context, tractor_state_input_idx);
    DRAKE_ASSERT(state != nullptr);

    output->get_mutable_value()(0) = state->heading();
}
