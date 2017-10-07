# estimatorComparator2017
Sources used to compare the MKF orientation estimation approaches.

There are 9 orientation estimation algorithms:
- 4 of them are based on the UKF (Manifold Unscented Kalman Filter).
- 4 of them are based on the EKF (Manifold Extended Kalman Filter).
- the last one is the Madgwick algorithm

Each version of the UKF and the EKF is different from the others by the used chart. 4 charts are studied:
- Orthographic
- Rodrigues Parameters
- Modified Rodrigues Parameters
- Rotation Vector

The sensor is an IMU, that provide measurements of acceleration, and angular velocity.

The code is shown running at:

Quick demonstration: https://www.youtube.com/watch?v=JpjQP7dHhgk

Short explanatory video: https://www.youtube.com/watch?v=YbhUdnTHcMo

Long explanatory video: https://www.youtube.com/watch?v=6oH4Stb2Ifs

About dead reckoning: https://www.youtube.com/watch?v=IS-jpxXiTjE
