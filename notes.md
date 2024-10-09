1. Create a UDP socket.
2. Send a message to the server.
3. Wait until a response from the server is received.
4. Process the reply and go back to step 2, if necessary.
5. Close socket descriptor and exit.


## Arguments :
domain – Specifies the communication
domain ( AF_INET for IPv4/ AF_INET6 for IPv6 )
type – Type of socket to be created
( SOCK_STREAM for TCP / SOCK_DGRAM for UDP )
protocol – Protocol to be used by the socket.
0 means using the default protocol for the address family.



## Understanding the Inputs:
Initial Position (X, Y, Z): Your starting true position in 3D space.
Initial Speed: Your speed at the beginning (in km/h).
Current Acceleration: The current acceleration in 3D space (in m/s²).
Current Direction (Euler Angles): The direction the vehicle is heading, given in Euler angles. You’ll need to convert this to a rotation matrix or quaternion for position update calculations.
GPS Position: Every 3 seconds, you get a GPS reading, which has noise and provides your position (X, Y, Z).
Noise Characteristics: Gaussian white noise added to your sensors, which will need to be modeled in your Kalman filter.


2. Kalman Filter Overview:
The Kalman filter estimates the true state (position and velocity) of the vehicle by using:


Position and velocity in 3D space.
Process model: This predicts the next state using acceleration data and integrates it over time.
Measurement model: Uses GPS data to correct the predicted state.
3. State Initialization:
The first input you receive will provide the necessary data to initialize your Kalman filter:

Position: Use the initial position directly as the first estimate of the position.
Velocity: Convert the speed from km/h to m/s and use the initial direction (Euler angles) to resolve the velocity vector in the XYZ frame. You’ll use this for the velocity components of the state.
Acceleration: This will be used in the process model to update your velocity estimates over time.
4. State Prediction (Process Model):
Use the acceleration and previous state to predict the new state:
​

6. Measurement and Process Noise:
You’ll model the measurement noise and process noise as covariance matrices:

Measurement noise covariance (R): Based on the given GPS noise, accelerometer noise, and gyroscope noise.
Process noise covariance (Q): Models the uncertainty in the process, based on your model assumptions.
7. Kalman Filter Loop:
Predict: Use the current state and acceleration to predict the next position and velocity.
Update: When a new GPS measurement arrives, correct the predicted state using the measurement.
Repeat: Continue the predict-update loop for each time step.
8. Estimation Output:
After each update, send the estimated position in the required format:

```X Y Z```

```1.7325073314060224 -2.2213777837034083 0.49999962025821726```

### Summary of Steps in Code:
Initialize the state vector with the initial position and velocity.
Set up the Kalman filter matrices (F, B, H, Q, R).
For each time step:
Update the state using the process model (acceleration).
If a GPS update is available, correct the state using the measurement model.
Send the estimated position.
This setup will allow you to estimate the vehicle’s position with the Kalman filter, handling noisy sensor data.