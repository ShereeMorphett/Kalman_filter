#include "Kalman.hpp"

Eigen::MatrixXd Kalman::get_body_to_inertial_rotation()
{
    Eigen::Vector3d angles = parser.get_direction();
    double sin_roll = sin(angles(0));
    double cos_roll = cos(angles(0));
    double sin_pitch = sin(angles(1));
    double cos_pitch = cos(angles(1));
    double sin_yaw = sin(angles(2));
    double cos_yaw = cos(angles(2));

    Eigen::Matrix3d r_roll{{1, 0, 0},
                           {0, cos_roll, -sin_roll},
                           {0, sin_roll, cos_roll}};

    Eigen::Matrix3d r_pitch{{cos_pitch, 0, sin_pitch},
                            {0, 1, 0},
                            {-sin_pitch, 0, cos_pitch}};

    Eigen::Matrix3d r_yaw{{cos_yaw, -sin_yaw, 0},
                          {sin_yaw, cos_yaw, 0},
                          {0, 0, 1}};

    /*
        As of now, I am fairly confident we are in a rh coordinate system,
        and we might be dealing with a 1-2-3 rotation sequence.
        As in first roll, then pitch, then yaw. Which is not terribly common. Could be first point of debug
        More common would be 3-2-1, which is yaw, pitch, roll (read from right to left).
        Based on the description in the subject, we should have actual euler angles
        and not rates of change. So we should be able to use the above matrices.
    */
    return r_yaw * r_pitch * r_roll; // Possibly switch yaw and roll
}

void Kalman::set_state_vector()
{
    state_vector.segment<3>(0) = parser.get_true_position();
    state_vector.segment<3>(3) = parser.get_speed();
}

void Kalman::set_state_transition_matrix()
{
    state_transition_matrix.setIdentity(6, 6);
    state_transition_matrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * dt;
    std::cout << state_transition_matrix << std::endl;
}

void Kalman::set_process_error_matrix()
{
    process_error_matrix = state_transition_matrix * error_covariance_matrix * state_transition_matrix.transpose();
    process_error_matrix *= dt; // intergration would be necessary here if we didn't have this artificial setup and varying noise covariances (e.g. gps becopmes more inaccurate if we move faster or something)
}

void Kalman::set_measurement_to_state_matrix()
{
    measurement_to_state_matrix.setZero();
    measurement_to_state_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
}

void Kalman::set_control_input_vector(Parser const &parser)
{

    /*
        since we're not tracking acceleration and orientation as state anymore and the vehicle moves
        only lognitudinally,
        we need to convert the body frame rotation and inertial frame acceleration
        into a net acceleration on the imu in the inertial frame.
        Then with the control input matrix, the acceleration is converted into a change in velocity
        and position.
    */
    Eigen::Vector3d acceleration = get_body_to_inertial_rotation() * parser.get_acceleration();
    control_input_vector.segment<3>(0) = acceleration;
}

void Kalman::set_control_input_matrix()
{
    control_input_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3) * dt * dt / 2;
    control_input_matrix.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity(3, 3) * dt;
}

void Kalman::set_measurement_noise_matrix()
{
    measurement_noise_matrix.setIdentity();
    measurement_noise_matrix.block<3, 3>(0, 0) *= variance_gps;
}

void Kalman::set_error_covariance_matrix()
{

    double variance_v = variance_accelerometer * dt + variance_gyroscope;
    error_covariance_matrix.Identity(6, 6);
    error_covariance_matrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3);

    error_covariance_matrix.block<3, 3>(0, 0) *= 0.5 * dt * dt;
    error_covariance_matrix.block<3, 3>(0, 3) *= 0.5 * dt * dt;
    error_covariance_matrix.block<3, 3>(3, 3) *= dt;

    Eigen::MatrixXd noiseDensityMatrix(6, 6);
    noiseDensityMatrix.setIdentity();

    // Covariances with position
    noiseDensityMatrix.block<3, 3>(0, 0) *= variance_gps + 0.5 * variance_accelerometer * dt * dt;
    noiseDensityMatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * (variance_v + 0.5 * variance_accelerometer * dt * dt);
    // noiseDensityMatrix.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity(3, 3) * (variance_v + 0.5 * variance_accelerometer * dt * dt); //covanriance matrices are always symmetric
    /*It could be valid to have the lower triangular matrix empty, as the early errors do not propagate backwards, if ever (ie uncertainty in the initial position and velocity are 0)*/
    // Convariances with velocity
    noiseDensityMatrix.block<3, 3>(3, 3) *= variance_v + variance_accelerometer * dt;
    error_covariance_matrix = error_covariance_matrix * noiseDensityMatrix;
}