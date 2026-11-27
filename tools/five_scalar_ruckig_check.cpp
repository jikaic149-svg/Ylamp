// Host-only independent check using the project's existing Ruckig library.
#include <ruckig/ruckig.hpp>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 4) return 2;
    const double velocity = std::strtod(argv[1], nullptr);
    const double acceleration = std::strtod(argv[2], nullptr);
    const double jerk = std::strtod(argv[3], nullptr);
    if (!std::isfinite(velocity) || !std::isfinite(acceleration) ||
        !std::isfinite(jerk) || velocity <= 0 || acceleration <= 0 || jerk <= 0)
        return 2;
    ruckig::Ruckig<1> generator;
    std::cout << std::setprecision(17) << "{\"transitions\":[";
    for (int transition = 0; transition < 2; ++transition) {
        ruckig::InputParameter<1> input;
        input.control_interface = ruckig::ControlInterface::Velocity;
        input.current_position = {0.};
        input.current_velocity = {transition ? velocity : 0.};
        input.current_acceleration = {0.};
        input.target_velocity = {transition ? -velocity : velocity};
        input.target_acceleration = {0.};
        input.max_velocity = {velocity};
        input.max_acceleration = {acceleration};
        input.max_jerk = {jerk};
        ruckig::Trajectory<1> trajectory;
        if (generator.calculate(input, trajectory) < ruckig::Result::Working) return 3;
        if (transition) std::cout << ',';
        const double duration = trajectory.get_duration();
        std::cout << "{\"duration_s\":" << duration << ",\"samples\":[";
        for (int sample = 0; sample <= 200; ++sample) {
            std::array<double, 1> q{}, v{}, a{}, j{};
            size_t section{};
            const double t = duration * sample / 200.;
            trajectory.at_time(t, q, v, a, j, section);
            if (sample) std::cout << ',';
            std::cout << '[' << t << ',' << q[0] << ',' << v[0] << ','
                      << a[0] << ',' << j[0] << ']';
        }
        std::cout << "]}";
    }
    std::cout << "]}\n";
}
