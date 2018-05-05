#include <Eigen/Core>
#include <drake/systems/framework/leaf_system.h>
#include "drake/automotive/gen/driving_command.h"

class CarController : public drake::systems::LeafSystem<double>
{
    public:
        CarController();
        const drake::systems::InputPortDescriptor<double>& state_input() const;
        const drake::systems::InputPortDescriptor<double>& pose_input() const;
        const drake::systems::InputPortDescriptor<double>& velocity_input() const;
        const drake::systems::OutputPort<double>& driving_command_output() const;
    private:
        void create_drive_command(const drake::systems::Context<double>& context, drake::automotive::DrivingCommand<double>* output) const;

        int state_input_idx;
        int pose_input_idx;
        int velocity_input_idx;
        int driving_command_output_idx;
};


