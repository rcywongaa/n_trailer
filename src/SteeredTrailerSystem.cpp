#include "SteeredTrailerSystem.hpp"

#include <string>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>

#include "SimpleCarStateToPose.hpp"

using namespace drake::systems;
using namespace drake::automotive;

SteeredTrailerSystem::SteeredTrailerSystem(int num_trailers, double trailer_length, double initial_link_length) :
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
            tractor_controller = builder.AddSystem<TractorController>();
            builder.Connect(tractor->state_output(), tractor_controller->state_input());
            builder.Connect(tractor_controller->driving_command_output(), tractor->driving_command_input());
            preceding_state_output = &(tractor->state_output());
            preceding_state_d_output = &(tractor->state_d_output());
        }
        else
        {
            SimpleCarState<double> initial_position;
            initial_position.set_x(-(trailer_length + initial_link_length) * i);
            SteeredTrailer* trailer = builder.AddSystem<SteeredTrailer>(initial_link_length, trailer_length, initial_position);
            trailer->set_name("trailer" + std::to_string(i));
            builder.Connect(*preceding_state_output, trailer->preceding_state_input());
            builder.Connect(*preceding_state_d_output, trailer->preceding_state_d_input());
            trailers.push_back(trailer);
            SteeredTrailerController* controller = builder.AddSystem<SteeredTrailerController>();
            builder.Connect(*preceding_state_output, controller->tractor_state_input());
            builder.Connect(controller->link_velocity_output(), trailer->link_velocity_input());
            builder.Connect(controller->steer_angle_output(), trailer->steer_angle_input());
            trailer_controllers.push_back(controller);
            preceding_state_output = &(trailer->state_output());
            preceding_state_d_output = &(trailer->state_d_output());
        }

        SimpleCarStateToPose* converter = builder.AddSystem<SimpleCarStateToPose>();
        builder.Connect(*preceding_state_output, converter->simple_car_state_input());
        int pose_output_idx = builder.ExportOutput(converter->pose_output());
        pose_output_indices.push_back(pose_output_idx);
    }
    // Visual output
    builder.BuildInto(this);
}

const OutputPort<double>& SteeredTrailerSystem::pose_output(int idx) const
{
    return this->get_output_port(pose_output_indices[idx]);
}

