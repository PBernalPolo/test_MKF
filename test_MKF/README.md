
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
- IPM > IPM_IMU > IPM_MPU6050
- IPM > IPM_MARG > IPM_MPU6050_HMC5883L
- IPM > IPM_IMU > IPM_AdafruitIMU9dof

The processing sketch is implemented in several files:
- estimatorComparator.pde
- Spacecraft.pde
- Fleet.pde
- dataAdministrator.pde
- myGUI.pde
