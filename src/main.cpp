
#include <iostream>
#include <string>
#include <Eigen/Dense> //may only need the dense
#include "Kalman.hpp"

/*
OPTIONS:
    -a, --accsig <acceleration_sigma>
            Manually specify the accelerometer's error sigma

    -d, --duration <trajectory_duration>
            Specify trajectory duration in minutes from unsigned integer

        --debug
            Enables debug mode which provides true information in addition to the noised information

        --delta
            Print the difference between real position and received position at each reception.

    -e, --entropy
            Generate seed from entropy.

        --filterspeed
            Print the mean of response time from the filter at the end of transmission.

    -g, --gpssig <gps_sigma>
            Manually specify the GPS' error sigma

    -h, --help
            Print help information

    -n, --noise <noise_increase>
            Multiply the noise applied to the accelerometer and GPS

    -p, --port <port>
            Manually specify port for the server. Otherwise defaults to 4242. If the port is closed,
            program defaults to closest open port.

    -s, --seed <manual_seed>
            Generate seed from unsigned integer

    -V, --version
            Print version information
*/

int main()
{
        Kalman kalman_filter;
        kalman_filter.filter_loop();
        return 0;
}
