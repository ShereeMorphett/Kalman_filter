#pragma once
#include <string>
#include <iostream>
#include <iomanip>
#include "types.hpp"

class Parser
{
public:
    Parser() = default;
    MeasurementData get_true_position() const;
    MeasurementData get_speed() const;
    MeasurementData get_acceleration() const;
    MeasurementData get_direction() const;
    MeasurementData get_position() const;

    MeasurementData parse_eigen_vec3(std::istringstream &data);
    void print_data(std::string const &label, MeasurementData const &data);
    void parse_data(std::string str_buffer);
    void read_data(int sock_fd);
    bool position_set = false;

private:
    const double kmh_to_ms = 1000. / 3600.;
    MeasurementData acceleration;
    MeasurementData direction;
    MeasurementData position;
    MeasurementData speed;
    MeasurementData true_position;
};
