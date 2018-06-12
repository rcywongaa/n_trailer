#pragma once

#include <vector>

#include <drake/systems/framework/diagram.h>

#include "Tractor.hpp"
#include "TractorController.hpp"
#include "SteeredTrailer.hpp"
#include "SteeredTrailerController.hpp"

// CONTROLLED trailer system
class SteeredTrailerSystem : public drake::systems::Diagram<double>
{
    public:
        SteeredTrailerSystem(int num_trailers, double trailer_length, double initial_link_length);
        // Intended for visualization only
        const drake::systems::OutputPort<double>& pose_output(int idx) const;
    private:
        int num_trailers;
        Tractor* tractor;
        TractorController* tractor_controller;
        std::vector<SteeredTrailer*> trailers;
        std::vector<SteeredTrailerController*> trailer_controllers;
        std::vector<int> pose_output_indices;
};

