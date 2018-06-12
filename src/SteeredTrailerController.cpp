#include <iostream>
#include "SteeredTrailerController.hpp"
#include "drake/automotive/gen/simple_car_state.h"

#include "SteeredTrailerState.hpp"

using namespace drake::systems;
using namespace drake::automotive;

SteeredTrailerController::SteeredTrailerController() :
    full_state_input_idx(this->DeclareVectorInputPort(SteeredTrailerState()).get_index()),
    preceding_state_input_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    link_extension_velocity_output_idx(this->DeclareVectorOutputPort(BasicVector<double>(1), &SteeredTrailerController::calc_link_extension_velocity).get_index()),
    steer_angle_output_idx(this->DeclareVectorOutputPort(BasicVector<double>(1), &SteeredTrailerController::calc_steer_angle).get_index())
{
    std::cout << "Controller full_state_input_idx = " << std::to_string(full_state_input_idx) << std::endl;
    std::cout << "Controller preceding_state_input_idx = " << std::to_string(preceding_state_input_idx) << std::endl;
    std::cout << "Controller link_extension_velocity_output_idx = " << std::to_string(link_extension_velocity_output_idx) << std::endl;
    std::cout << "Controller steer_angle_output_idx = " << std::to_string(steer_angle_output_idx) << std::endl;
}

const InputPortDescriptor<double>& SteeredTrailerController::full_state_input() const
{
    return System<double>::get_input_port(full_state_input_idx);
}

const InputPortDescriptor<double>& SteeredTrailerController::preceding_state_input() const
{
    return System<double>::get_input_port(preceding_state_input_idx);
}

const OutputPort<double>& SteeredTrailerController::link_extension_velocity_output() const
{
    return System<double>::get_output_port(link_extension_velocity_output_idx);
}

const OutputPort<double>& SteeredTrailerController::steer_angle_output() const
{
    return System<double>::get_output_port(steer_angle_output_idx);
}

void SteeredTrailerController::calc_link_extension_velocity(const Context<double>& context, BasicVector<double>* output) const
{
    const SteeredTrailerState* const state = dynamic_cast<const SteeredTrailerState* const>(this->EvalVectorInput(context, full_state_input_idx));
    DRAKE_ASSERT(state != nullptr);
    const SimpleCarState<double>* const preceding_state = this->EvalVectorInput<SimpleCarState>(context, preceding_state_input_idx);
    DRAKE_ASSERT(preceding_state != nullptr);
    double desired_velocity = get_desired_velocity();
    double desired_front_wheel_velocity = desired_velocity / std::cos(state->heading());
    double desired_link_velocity = desired_front_wheel_velocity / std::cos(state->steer_angle() - state->joint_angle());
    output->get_mutable_value()(0) = preceding_state->velocity() - desired_link_velocity;
}

void SteeredTrailerController::calc_steer_angle(const Context<double>& context, BasicVector<double>* output) const
{
    output->get_mutable_value()(0) = get_desired_steer_angle();
}

double SteeredTrailerController::get_desired_velocity() const
{
    return 1.0;
}

double SteeredTrailerController::get_desired_steer_angle() const
{
    return 0.5;
}
