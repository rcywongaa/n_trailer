#pragma once

#include <vector>

#include <drake/systems/framework/diagram.h>

#include "Tractor.hpp"
#include "Trailer.hpp"

class TrailerSystem : public drake::systems::Diagram<double>
{
    public:
        TrailerSystem(int num_trailers, double trailer_length);
        const drake::systems::InputPortDescriptor<double>& driving_command_input() const;
        // Only for rear trailer
        const drake::systems::OutputPort<double>& state_output() const;
        const drake::systems::OutputPort<double>& velocity_output() const;
        // Intended for visualization only
        const drake::systems::OutputPort<double>& visual_output() const;
    private:
        int num_trailers;
        Tractor* tractor;
        std::vector<Trailer*> trailers;
};
