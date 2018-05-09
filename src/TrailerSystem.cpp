#include "TrailerSystem.hpp"

#include <string>

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
    const OutputPort<double>* preceding_state_d_output;
    std::string name;
    for (int i = 0; i < num_trailers+1; i++)
    {
        if (i == 0)
        {
            tractor = builder.AddSystem<Tractor>();
            tractor->set_name("tractor");
            builder.ExportInput(tractor->driving_command_input());
            preceding_state_output = &(tractor->state_output());
            preceding_state_d_output = &(tractor->state_d_output());
        }
        else
        {
            SimpleCarState<double> initial_position;
            initial_position.set_x(-trailer_length * i);
            Trailer* trailer = builder.AddSystem<Trailer>(trailer_length, initial_position);
            trailer->set_name("trailer" + std::to_string(i));
            builder.Connect(*preceding_state_output, trailer->preceding_state_input());
            builder.Connect(*preceding_state_d_output, trailer->preceding_state_d_input());
            trailers.push_back(trailer);
            preceding_state_output = &(trailer->state_output());
            preceding_state_d_output = &(trailer->state_d_output());
        }

        SimpleCarStateToPose* converter = builder.AddSystem<SimpleCarStateToPose>();
        builder.Connect(*preceding_state_output, converter->simple_car_state_input());
        int pose_output_idx = builder.ExportOutput(converter->pose_output());
        pose_output_indices.push_back(pose_output_idx);
    }
    //Rear state output
    state_output_idx = builder.ExportOutput(*preceding_state_output);
    //Rear state_d output
    state_d_output_idx = builder.ExportOutput(*preceding_state_d_output);
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

const OutputPort<double>& TrailerSystem::state_d_output() const
{
    return this->get_output_port(state_d_output_idx);
}

const OutputPort<double>& TrailerSystem::pose_output(int idx) const
{
    return this->get_output_port(pose_output_indices[idx]);
}
