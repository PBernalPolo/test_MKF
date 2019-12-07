
These files are the Java implementation of the orientation estimation algorithms, together with a processing sketch that is used to visualize the algorithms performance.

There are different orientation estimation algorithms:
- OrientationEstimator > MEKF > MEKFcO
- OrientationEstimator > MEKF > MEKFcRP
- OrientationEstimator > MEKF > MEKFcMRP
- OrientationEstimator > MEKF > MEKFcRV
- OrientationEstimator > MUKF > MUKFcO
- OrientationEstimator > MUKF > MUKFcRP
- OrientationEstimator > MUKF > MUKFcMRP
- OrientationEstimator > MUKF > MUKFcRV
- OrientationEstimator > MadgwickAHRS

The data is managed according to the following classes:
- MessageManager
- IPM > IPM_IMU > IPM_MPU6050
- IPM > IPM_MARG > IPM_MPU6050_HMC5883L
- IPM > IPM_IMU > IPM_AdafruitIMU9dof

The rest of the processing sketch is implemented in several files:
- test_MKF.pde
- Spacecraft.pde
- Fleet.pde
- myGUI.pde
- SerialPortManager.pde
- CommunicationManager.pde
- dataAdministrator.pde
