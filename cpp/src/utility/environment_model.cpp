#include "reach_semantic/utility/environment_model.hpp"

using namespace semantic_reach;

Obstacle::state_map_t resample_trajectory(const Obstacle::state_map_t &current_trajectory, size_t step_width) {
    Obstacle::state_map_t resampled_trajectory{};
    for (const auto &[time_step, state] : current_trajectory) {
        if (time_step % step_width == 0) {
            state->setTimeStep(time_step / step_width);
            resampled_trajectory[time_step / step_width] = state;
        }
    }
    return resampled_trajectory;
}

void semantic_reach::resample_obstacle_states(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                              double obstacle_dt, double new_dt) {
    constexpr int numeric_scaling = 100;
    if (fmod((new_dt * numeric_scaling), (obstacle_dt * numeric_scaling)) != 0) {
        throw std::logic_error("New dt is not a multiple of obstacle dt");
    }
    auto step_width = static_cast<size_t>(round((new_dt * numeric_scaling) / (obstacle_dt * numeric_scaling)));
    if (step_width == 1) {
        return;
    }
    for (const auto &obs : obstacles) {
        auto current_state = obs->getCurrentState();
        auto prediction = obs->getTrajectoryPrediction();
        if (current_state && current_state->getTimeStep() % step_width != 0) {
            auto current_time_step = current_state->getTimeStep();
            auto mod = current_time_step % step_width;
            // Take the first state of the trajectory prediction that matches the step width as current state
            current_state = nullptr;
            for (auto timeStep = current_time_step + step_width - mod; timeStep <= obs->getFinalTimeStep();
                 timeStep += step_width) {
                if (prediction.count(timeStep) == 1) {
                    current_state = prediction.at(timeStep);
                    prediction.erase(timeStep);
                    break;
                }
            }
        }
        if (current_state) {
            current_state->setTimeStep(current_state->getTimeStep() / step_width);
        }
        obs->setCurrentState(current_state);
        obs->setTrajectoryPrediction(resample_trajectory(prediction, step_width));
        obs->setTrajectoryHistory(resample_trajectory(obs->getTrajectoryHistory(), step_width));
    }
}
