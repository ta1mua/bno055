// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import frc.robot.utils.BNO055;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class Drivetrain extends SubsystemBase {
     
  private final Gyro m_gyro = new ADXRS450_Gyro();
  private static BNO055 imu;  
  private double[] pos = new double[3]; // [x,y,z] position data
	private BNO055.CalData cal;
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {    
    
    imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);    
  }
  
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
  

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   *//*
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }*/

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   *//*
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }
*/
  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   *//* 
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
  */
  

  /**Zeroes the heading of the robot */
  public void zeroHeading(){
    m_gyro.reset();
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(),360)*(DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  /** 
   * Returns the turn rate of the robot 
   * @return The turn rate of the robot, in degree per second
   * */
  public double getTurnRate(){
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //--------------------BNO055 ------------------------------
   /**
	 * The heading of the sensor (x axis) in continuous format. Eg rotating the
	 *   sensor clockwise two full rotations will return a value of 720 degrees.
	 *
	 * @return heading in degrees
     */
  public double getHeadingIMU() {
   	return imu.getHeading();
  }
  /**
     * Gets a vector representing the sensors position (heading, roll, pitch).
	 * heading:    0 to 360 degrees
	 * roll:     -90 to +90 degrees
	 * pitch:   -180 to +180 degrees
	 *
	 * For continuous rotation heading (doesn't roll over between 360/0) see
	 *   the getHeading() method.
	 *
	 * @return a vector [heading, roll, pitch]
	 */
  public double[] getVectorIMU() {
    return imu.getVector();
  }
  
  /**
    * @return true if the IMU is found on the I2C bus
  */
  public boolean isSensorPresentIMU() {
    return imu.isSensorPresent();
  }

  /** 
    * @return true when the IMU is initialized.
  */
  public boolean isInitializedIMU() {
    return imu.isInitialized();
  }

  /**
  * Gets current IMU calibration state.
  * @return each value will be set to 0 if not calibrated, 3 if fully
  *   calibrated.
  */
  public BNO055.CalData getCalibrationIMU() {
    return imu.getCalibration();
  }

  /**
  * Returns true if all required sensors (accelerometer, magnetometer,
  *   gyroscope) in the IMU have completed their respective calibration
  *   sequence.
  * @return true if calibration is complete for all sensors required for the
  *   mode the sensor is currently operating in. 
  */
  public boolean isCalibratedIMU() {
    return imu.isCalibrated();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */          
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyro450", getHeading());   
    if(imu.isInitialized()){
      pos = imu.getVector();      
      cal = imu.getCalibration();      
      //SmartDashboard.putNumber("IMU heading", getHeadingIMU());
      SmartDashboard.putNumber("IMU pos0", pos[0]); //şase yatay dönüş, heading
      SmartDashboard.putNumber("IMU pos1", pos[1]); //roll
      SmartDashboard.putNumber("IMU pos2", pos[2]); //şarj istasyonu eğimi, pitch
      SmartDashboard.putNumber("IMU kal gyro", cal.gyro);
      SmartDashboard.putNumber("IMU kal accel", cal.accel);
      SmartDashboard.putNumber("IMU kal mag", cal.mag);
    }

  }  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
