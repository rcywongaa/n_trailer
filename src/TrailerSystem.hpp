#pragma once

#include <vector>

#include <drake/systems/framework/diagram.h>

#include "Tractor.hpp"
#include "Trailer.hpp"

class TrailerSystem : public drake::systems::Diagram<double>
{
    public:
        TrailerSystem(int num_trailers, double trailer_length);
        const drake::systems::InputPort<double>& driving_command_input() const;
        // Only for rear trailer
        const drake::systems::OutputPort<double>& state_output() const;
        const drake::systems::OutputPort<double>& state_d_output() const;
        // Intended for visualization only
        const drake::systems::OutputPort<double>& pose_output(int idx) const;
    private:
        int num_trailers;
        Tractor* tractor;
        std::vector<Trailer*> trailers;
        int state_output_idx;
        int state_d_output_idx;
        std::vector<int> pose_output_indices;
};
