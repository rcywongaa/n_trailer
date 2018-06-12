#include "SteeredTrailer.hpp"

#include <cmath>
#include <iostream>

using namespace drake::systems;
using namespace drake::automotive;

SteeredTrailer::SteeredTrailer(double initial_link_length, double trailer_length, const SimpleCarState<double>& initial_state) :
    preceding_state_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    preceding_state_d_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    link_extension_velocity_idx(this->DeclareVectorInputPort(BasicVector<double>(1)).get_index()),
    steer_angle_idx(this->DeclareVectorInputPort(BasicVector<double>(1)).get_index()),
    state_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &SteeredTrailer::get_output_state).get_index()),
    state_d_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &SteeredTrailer::get_output_state_d).get_index()),
    full_state_idx(this->DeclareVectorOutputPort(SteeredTrailerState(), &SteeredTrailer::get_output_full_state).get_index()),
    trailer_length(trailer_length)
{
    std::cout << "Trailer preceding_state_idx = " << std::to_string(preceding_state_idx) << std::endl;
    std::cout << "Trailer preceding_state_d_idx = " << std::to_string(preceding_state_d_idx) << std::endl;
    std::cout << "Trailer link_extension_velocity_idx = " << std::to_string(link_extension_velocity_idx) << std::endl;
    std::cout << "Trailer steer_angle_idx = " << std::to_string(steer_angle_idx) << std::endl;
    std::cout << "Trailer state_idx = " << std::to_string(state_idx) << std::endl;
    std::cout << "Trailer state_d_idx = " << std::to_string(state_d_idx) << std::endl;
    std::cout << "Trailer full_state_idx = " << std::to_string(full_state_idx) << std::endl;
    this->DeclareContinuousState(SimpleCarState<double>());
    this->initial_state.set_x(initial_state.x());
    this->initial_state.set_y(initial_state.y());
    this->initial_state.set_heading(initial_state.heading());
    this->initial_state.set_velocity(initial_state.velocity());
}

void SteeredTrailer::SetDefaultState(const Context<double>&, State<double>* state) const
{
    SimpleCarState<double>& mutable_state = dynamic_cast<SimpleCarState<double>&>(state->get_mutable_continuous_state().get_mutable_vector());
    mutable_state.set_x(initial_state.x());
    mutable_state.set_y(initial_state.y());
    mutable_state.set_heading(initial_state.heading());
    mutable_state.set_velocity(initial_state.velocity());
    std::cout << get_name() << " : SetDefaultState(" << mutable_state << ")" << std::endl;
}

void SteeredTrailer::DoCalcTimeDerivatives(const Context<double>& context, ContinuousState<double>* derivatives) const
{
    SimpleCarState<double>& state_d = dynamic_cast<SimpleCarState<double>&>(derivatives->get_mutable_vector());
    get_output_state_d(context, &state_d);
}

const InputPortDescriptor<double>& SteeredTrailer::preceding_state_input() const
{
    return System<double>::get_input_port(preceding_state_idx);
}

const InputPortDescriptor<double>& SteeredTrailer::preceding_state_d_input() const
{
    return System<double>::get_input_port(preceding_state_d_idx);
}

const InputPortDescriptor<double>& SteeredTrailer::link_extension_velocity_input() const
{
    return System<double>::get_input_port(link_extension_velocity_idx);
}

const InputPortDescriptor<double>& SteeredTrailer::steer_angle_input() const
{
    return System<double>::get_input_port(steer_angle_idx);
}

const OutputPort<double>& SteeredTrailer::state_output() const
{
    return System<double>::get_output_port(state_idx);
}

const OutputPort<double>& SteeredTrailer::state_d_output() const
{
    return System<double>::get_output_port(state_d_idx);
}

const OutputPort<double>& SteeredTrailer::full_state_output() const
{
    return System<double>::get_output_port(full_state_idx);
}

const SimpleCarState<double>& SteeredTrailer::get_state(const drake::systems::Context<double>& context) const
{
    return dynamic_cast<const SimpleCarState<double>&>(context.get_continuous_state().get_vector());
}

const SimpleCarState<double>* const SteeredTrailer::get_preceding_state(const drake::systems::Context<double>& context) const
{
    return this->EvalVectorInput<SimpleCarState>(context, this->preceding_state_idx);
}

const SimpleCarState<double>* const SteeredTrailer::get_preceding_state_d(const drake::systems::Context<double>& context) const
{
    return this->EvalVectorInput<SimpleCarState>(context, this->preceding_state_d_idx);
}

const double SteeredTrailer::get_link_velocity(const drake::systems::Context<double>& context) const
{
    double link_extension_velocity = this->EvalVectorInput(context, link_extension_velocity_idx)->GetAtIndex(0);
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    return preceding_state->velocity() - link_extension_velocity;
}

const double SteeredTrailer::get_steer_angle(const drake::systems::Context<double>& context) const
{
    return this->EvalVectorInput(context, this->steer_angle_idx)->GetAtIndex(0);
}

const double SteeredTrailer::get_front_wheel_velocity(const Context<double>& context) const
{
    const double link_velocity = get_link_velocity(context);
    const double steer_angle = get_steer_angle(context);
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    double joint_angle = get_joint_angle(context);
    double front_wheel_velocity = link_velocity * std::cos(steer_angle - joint_angle);
    return front_wheel_velocity;
}

double SteeredTrailer::get_linear_velocity(const Context<double>& context) const
{
    double front_wheel_velocity = get_front_wheel_velocity(context);
    double steer_angle = get_steer_angle(context);
    return front_wheel_velocity * std::cos(steer_angle);
}

double SteeredTrailer::get_angular_velocity(const Context<double>& context) const
{
    const double link_velocity = get_link_velocity(context);
    const double steer_angle = get_steer_angle(context);
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    double joint_angle = get_joint_angle(context);
    double front_wheel_velocity = get_front_wheel_velocity(context);
    return front_wheel_velocity * std::sin(steer_angle) / this->trailer_length;
}

double SteeredTrailer::get_joint_angle(const Context<double>& context) const
{
    const SimpleCarState<double>& state = get_state(context);
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    double front_wheel_x = state.x() + this->trailer_length*std::cos(state.heading());
    double front_wheel_y = state.y() + this->trailer_length*std::sin(state.heading());
    return std::atan2(preceding_state->y() - front_wheel_y, preceding_state->x() - front_wheel_x);
}

void SteeredTrailer::get_output_state(const Context<double>& context, SimpleCarState<double>* state) const
{
    const SimpleCarState<double>& current_state = get_state(context);
    state->set_x(current_state.x());
    state->set_y(current_state.y());
    state->set_heading(current_state.heading());
    state->set_velocity(get_linear_velocity(context));
}

void SteeredTrailer::get_output_state_d(const Context<double>& context, SimpleCarState<double>* state_d) const
{
    const SimpleCarState<double>& state = get_state(context);
    double joint_angle = get_joint_angle(context);
    double v = get_linear_velocity(context);
    state_d->set_x(v * std::cos(state.heading()));
    state_d->set_y(v * std::sin(state.heading()));
    double angular_velocity = get_angular_velocity(context);
    state_d->set_heading(angular_velocity);
    state_d->set_velocity(0.0);
}

void SteeredTrailer::get_output_full_state(const Context<double>& context, SteeredTrailerState* full_state) const
{
    const SimpleCarState<double>& simple_state = get_state(context);
    full_state->set_x(simple_state.x());
    full_state->set_y(simple_state.y());
    full_state->set_heading(simple_state.heading());
    //full_state->set_velocity(simple_state.velocity());
    full_state->set_steer_angle(get_steer_angle(context));
    full_state->set_joint_angle(get_joint_angle(context));
}
