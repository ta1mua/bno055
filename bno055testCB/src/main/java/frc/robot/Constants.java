// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;        
    public static final int kLeftX=0, kLeftY=1, kRightX=4, kRightY=5, kLTrig=2, kRTrig=3; 
    public static final int kButtonA=1, kButtonB=2, kButtonX=3, kButtonY=4, kButtonLB=5, 
                            kButtonRB=6, kButtonSt=8, kButtonRSB=10, kButtonLSB=9;
  }

  /* Drive Constant */
  public static class DriveConstants {
    public static final int kLeftFrontMotorPort=3, kLeftRearMotorPort=2;
    public static final int kRightFrontMotorPort=5, kRightRearMotorPort=4;    

    
    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

  }  

}
