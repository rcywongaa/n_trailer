#include "Trailer.hpp"

#include <cmath>
#include <iostream>

using namespace drake::systems;
using namespace drake::automotive;

Trailer::Trailer(double length, const SimpleCarState<double>& initial_state) :
    preceding_state_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    preceding_state_d_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    state_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &Trailer::get_output_state).get_index()),
    state_d_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &Trailer::get_output_state_d).get_index())
{
    this->DeclareContinuousState(SimpleCarState<double>());
    this->initial_state.set_x(initial_state.x());
    this->initial_state.set_y(initial_state.y());
    this->initial_state.set_heading(initial_state.heading());
    this->initial_state.set_velocity(initial_state.velocity());
    this->length = length;
}

void Trailer::SetDefaultState(const Context<double>&, State<double>* state) const
{
    SimpleCarState<double>& mutable_state = dynamic_cast<SimpleCarState<double>&>(state->get_mutable_continuous_state().get_mutable_vector());
    mutable_state.set_x(initial_state.x());
    mutable_state.set_y(initial_state.y());
    mutable_state.set_heading(initial_state.heading());
    mutable_state.set_velocity(initial_state.velocity());
    std::cout << get_name() << " : SetDefaultState(" << mutable_state << ")" << std::endl;
}

void Trailer::DoCalcTimeDerivatives(const Context<double>& context, ContinuousState<double>* derivatives) const
{
    SimpleCarState<double>& state_d = dynamic_cast<SimpleCarState<double>&>(derivatives->get_mutable_vector());
    get_output_state_d(context, &state_d);
}

const InputPortDescriptor<double>& Trailer::preceding_state_input() const
{
    return System<double>::get_input_port(preceding_state_idx);
}

const InputPortDescriptor<double>& Trailer::preceding_state_d_input() const
{
    return System<double>::get_input_port(preceding_state_d_idx);
}

const OutputPort<double>& Trailer::state_output() const
{
    return System<double>::get_output_port(state_idx);
}

const OutputPort<double>& Trailer::state_d_output() const
{
    return System<double>::get_output_port(state_d_idx);
}

const SimpleCarState<double>& Trailer::get_state(const drake::systems::Context<double>& context) const
{
    return dynamic_cast<const SimpleCarState<double>&>(context.get_continuous_state().get_vector());
}

const SimpleCarState<double>* const Trailer::get_preceding_state(const drake::systems::Context<double>& context) const
{
    return this->EvalVectorInput<SimpleCarState>(context, this->preceding_state_idx);
}

double Trailer::get_linear_velocity(const Context<double>& context) const
{
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    double joint_angle = get_joint_angle(context);
    return preceding_state->velocity() * std::cos(joint_angle);
}

double Trailer::get_joint_angle(const Context<double>& context) const
{
    const SimpleCarState<double>& state = get_state(context);
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    return state.heading() - preceding_state->heading();
}

void Trailer::get_output_state(const Context<double>& context, SimpleCarState<double>* state) const
{
    const SimpleCarState<double>& current_state = get_state(context);
    const SimpleCarState<double>* const preceding_state = this->EvalVectorInput<SimpleCarState>(context, this->preceding_state_idx);
    state->set_x(current_state.x());
    state->set_y(current_state.y());
    state->set_heading(current_state.heading());
    state->set_velocity(get_linear_velocity(context));
}

void Trailer::get_output_state_d(const Context<double>& context, SimpleCarState<double>* state_d) const
{
    const SimpleCarState<double>& state = get_state(context);
    const SimpleCarState<double>* const preceding_state = get_preceding_state(context);
    //const SimpleCarState<double>* const preceding_state_d = this->EvalVectorInput<SimpleCarState>(context, this->preceding_state_d_idx);
    double joint_angle = get_joint_angle(context);
    double v = get_linear_velocity(context);
    state_d->set_x(v * std::cos(state.heading()));
    state_d->set_y(v * std::sin(state.heading()));
    state_d->set_heading(-preceding_state->velocity() * std::sin(joint_angle) / length);
    state_d->set_velocity(0.0);
}
