#include "TrailerSystem.hpp"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>

#include "VisualStateConverter.hpp"
#include "VisualState.hpp"

using namespace drake::systems;

TrailerSystem::TrailerSystem(int num_trailers, double trailer_length) :
    trailers(num_trailers)
{
    this->num_trailers = num_trailers;

    DiagramBuilder<double> builder;

    const int VISUAL_STATE_SIZE = VisualState<double>().size();
    auto mux = builder.AddSystem<Multiplexer<double>>(VISUAL_STATE_SIZE * (num_trailers+1));

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
            Trailer* trailer = builder.AddSystem<Trailer>(trailer_length);
            builder.Connect(*preceding_state_output, trailer->preceding_state_input());
            builder.Connect(*preceding_velocity_output, trailer->preceding_velocity_input());
            trailers.push_back(trailer);
            preceding_state_output = &(trailer->state_output());
            preceding_velocity_output = &(trailer->velocity_output());
        }

        VisualStateConverter* converter = builder.AddSystem<VisualStateConverter>();
        builder.Connect(*preceding_state_output, converter->simple_car_state_input());

        auto demux = builder.AddSystem<Demultiplexer<double>>(VISUAL_STATE_SIZE);
        builder.Connect(converter->visual_state_output(), demux->get_input_port(0));
        for (int j = 0; j < VISUAL_STATE_SIZE; j++)
        {
            builder.Connect(demux->get_output_port(j), mux->get_input_port(VISUAL_STATE_SIZE*i + j));
        }
    }
    //Rear state output
    builder.ExportOutput(*preceding_state_output);
    //Rear velocity output
    builder.ExportOutput(*preceding_velocity_output);
    // Visual output
    builder.ExportOutput(mux->get_output_port(0));
    builder.BuildInto(this);
}

const InputPortDescriptor<double>& TrailerSystem::driving_command_input() const
{
    return this->get_input_port(0);
}

const OutputPort<double>& TrailerSystem::state_output() const
{
    return this->get_output_port(0);
}

const OutputPort<double>& TrailerSystem::velocity_output() const
{
    return this->get_output_port(1);
}

const OutputPort<double>& TrailerSystem::visual_output() const
{
    return this->get_output_port(2);
}
