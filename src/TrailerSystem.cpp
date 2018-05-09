#include "TrailerSystem.hpp"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>

#include "SimpleCarStateToPose.hpp"

using namespace drake::systems;
using namespace drake::automotive;

TrailerSystem::TrailerSystem(int num_trailers, double trailer_length) :
    trailers(num_trailers)
{
    this->num_trailers = num_trailers;

    DiagramBuilder<double> builder;

    const OutputPort<double>* preceding_state_output;
    const OutputPort<double>* preceding_velocity_output;
    for (int i = 0; i < num_trailers+1; i++)
    {
        if (i == 0)
        {
            tractor = builder.AddSystem<Tractor>();
            builder.ExportInput(tractor->driving_command_input());
            preceding_state_output = &(tractor->state_output());
            preceding_velocity_output = &(tractor->velocity_output());
        }
        else
        {
            SimpleCarState<double> initial_position;
            initial_position.set_x(-trailer_length * i);
            Trailer* trailer = builder.AddSystem<Trailer>(trailer_length, initial_position);
            builder.Connect(*preceding_state_output, trailer->preceding_state_input());
            builder.Connect(*preceding_velocity_output, trailer->preceding_velocity_input());
            trailers.push_back(trailer);
            preceding_state_output = &(trailer->state_output());
            preceding_velocity_output = &(trailer->velocity_output());
        }

        SimpleCarStateToPose* converter = builder.AddSystem<SimpleCarStateToPose>();
        builder.Connect(*preceding_state_output, converter->simple_car_state_input());
        int pose_output_idx = builder.ExportOutput(converter->pose_output());
        pose_output_indices.push_back(pose_output_idx);
    }
    //Rear state output
    state_output_idx = builder.ExportOutput(*preceding_state_output);
    //Rear velocity output
    velocity_output_idx = builder.ExportOutput(*preceding_velocity_output);
    // Visual output
    builder.BuildInto(this);
}

const InputPortDescriptor<double>& TrailerSystem::driving_command_input() const
{
    return this->get_input_port(0);
}

const OutputPort<double>& TrailerSystem::state_output() const
{
    return this->get_output_port(state_output_idx);
}

const OutputPort<double>& TrailerSystem::velocity_output() const
{
    return this->get_output_port(velocity_output_idx);
}

const OutputPort<double>& TrailerSystem::pose_output(int idx) const
{
    return this->get_output_port(pose_output_indices[idx]);
}
