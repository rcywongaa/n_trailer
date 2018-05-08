#include "Trailer.hpp"

#include <cmath>

#include "VisualStateConverter.hpp"

using namespace drake::systems;
using namespace drake::automotive;

Trailer::Trailer(double length) :
    preceding_state_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    preceding_velocity_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    state_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &Trailer::set_output_state).get_index()),
    velocity_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &Trailer::calc_velocity).get_index())
{
    this->DeclareContinuousState(SimpleCarState<double>());
    this->length = length;
}

void Trailer::DoCalcTimeDerivatives(const Context<double>& context, ContinuousState<double>* derivatives) const
{
    SimpleCarState<double>& velocity = dynamic_cast<SimpleCarState<double>&>(derivatives->get_mutable_vector());
    calc_velocity(context, &velocity);
}

const InputPortDescriptor<double>& Trailer::preceding_state_input() const
{
    return System<double>::get_input_port(preceding_state_idx);
}

const InputPortDescriptor<double>& Trailer::preceding_velocity_input() const
{
    return System<double>::get_input_port(preceding_velocity_idx);
}

const OutputPort<double>& Trailer::state_output() const
{
    return System<double>::get_output_port(state_idx);
}

const OutputPort<double>& Trailer::velocity_output() const
{
    return System<double>::get_output_port(velocity_idx);
}

const SimpleCarState<double>& Trailer::get_state(const drake::systems::Context<double>& context) const
{
    return dynamic_cast<const SimpleCarState<double>&>(context.get_continuous_state().get_vector());
}

void Trailer::set_output_state(const Context<double>& context, SimpleCarState<double>* state) const
{
    const SimpleCarState<double>& current_state = get_state(context);
    state->set_x(current_state.x());
    state->set_y(current_state.y());
    state->set_heading(current_state.heading());
    state->set_velocity(current_state.velocity());
}

void Trailer::calc_velocity(const Context<double>& context, SimpleCarState<double>* velocity) const
{
    const SimpleCarState<double>& state = get_state(context);
    const SimpleCarState<double>* const preceding_state = this->EvalVectorInput<SimpleCarState>(context, this->preceding_state_idx);
    const SimpleCarState<double>* const preceding_velocity = this->EvalVectorInput<SimpleCarState>(context, this->preceding_velocity_idx);
    double joint_angle = state.heading() - preceding_state->heading();
    velocity->set_velocity(preceding_velocity->velocity() * std::cos(joint_angle));
    velocity->set_x(velocity->velocity() * std::cos(state.heading()));
    velocity->set_y(velocity->velocity() * std::sin(state.heading()));
    velocity->set_heading(velocity->velocity() * std::sin(joint_angle) / length);
}
