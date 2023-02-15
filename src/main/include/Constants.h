#pragma once
#include <units/angular_velocity.h>

namespace constants
{
  constexpr bool kUseStickBOSS = false;
  constexpr bool kSilenceJoystickWarnings = true;
  constexpr double kSwerveAngleBreak = 90;
  constexpr double kSwerveDriveSpeedFilter = 2.0;  //smaller is more filtering (slower to reach top speed) - will bypass filter when joystick released

  constexpr double kSelfLevelDeadband = 3.0;  //deadband before level correction is done
  constexpr double kSelfLevelGain = 0.35;     //gain applied to drive motor in response to angle
  constexpr double kRadtoDeg = 57.2957795;
  constexpr double kDeg2Rad = 0.017453292519943295;

  constexpr double kWheelbase = 30.0;
  constexpr double kWheelwidth = 30.0;

  constexpr int kFrontLeftTurn_ID = 6;
  constexpr int kFrontLeftDrive_ID = 5;
  constexpr int kFrontLeftEncoder_ID = 16;

  constexpr int kFrontRightTurn_ID = 9;
  constexpr int kFrontRightDrive_ID = 10;
  constexpr int kFrontRightEncoder_ID = 11;

  constexpr int kRearLeftTurn_ID = 14;
  constexpr int kRearLeftDrive_ID = 15;
  constexpr int kRearLeftEncoder_ID = 21;

  constexpr int kRearRightTurn_ID = 19;
  constexpr int kRearRightDrive_ID = 20;
  constexpr int kRearRightEncoder_ID = 7;

  constexpr int kWinch1_ID = 17;  //1ft = -12150  2ft = -24296
  constexpr int kWinch2_ID = 18;
  constexpr int kWinchCountsPerInch = 1012.4;
  constexpr int kIntake_ID = 22;
  constexpr int kArm_ID = 23;
  constexpr int kWrist_ID = 24;

  constexpr double kEncoderCountsPerDegree = 4096.0 / 360.0;

  //read these from cancoder absolute position when cancoder is configured to -180 to 180 range
  constexpr double kFrontRightOffset = 24.69;
  constexpr double kFrontLeftOffset = -38.4;
  constexpr double kRearLeftOffset = -67.2;
  constexpr double kRearRightOffset = -94.4;

  constexpr double kTurn_KP = 0.01; 
  constexpr double kTurn_KI = 0;
  constexpr double kTurn_KD = 0; 
  constexpr double kTurn_KF = 0;
  constexpr double kTurn_IZ = 20; 

  constexpr double kTurnPeakOutputForward = 0.5;
  constexpr double kTurnPeakOutputReverse = -0.5;
  //Supply Limiting is to prevent breakers tripping or brownouts
  constexpr double kTurnSupplyCurrentLimit = 20.0; //amps
  constexpr double kTurnPeakCurrentLimit = 25.0; //amps
  constexpr double kTurnPeakCurrentDuration = 25; //msecs
  //Stator limiting is to limit acceleration or heat
  constexpr double kTurnStatorCurrentLimit = 20.0;
  constexpr double kTurnStatorPeakCurrentLimit = 25.0; //amps
  constexpr double kTurnStatorPeakCurrentDuration = 15; //msecs
  constexpr double kTurnVoltageCompSaturation = 12.0;
  constexpr double kTurnClosedLoopRamp = 1.0;

  constexpr double kDrive_kF = 0.015;
  constexpr double kDrive_kP = 1.0;
  constexpr double kDrive_kI = 0.001;
  constexpr double kDrive_kD = 0.5;
  constexpr double kDriveOpenLoopRamp = 1.0;
  constexpr double kDrivePeakOutputForward = 0.3;
  constexpr double kDrivePeakOutputReverse = -0.3;
  //Supply Limiting is to prevent breakers tripping or brownouts
  constexpr double kDriveSupplyCurrentLimit = 30.0; //amps
  constexpr double kDrivePeakCurrentLimit = 35.0; //amps
  constexpr double kDrivePeakCurrentDuration = 25; //msecs
  //Stator limiting is to limit acceleration or heat
  constexpr double kDriveStatorCurrentLimit = 25.0;
  constexpr double kDriveStatorPeakCurrentLimit = 30.0; //amps
  constexpr double kDriveStatorPeakCurrentDuration = 15; //msecs
  constexpr double kDriveVoltageCompSaturation = 12.0;
  
  constexpr double kWinch_F = 0.9;     //FeedForward
  constexpr double kWinch_P = 2.3;     //Proportional Gain
  constexpr double kWinch_I = 0.005;   //Integral Gain
  constexpr double kWinch_D = 0.0;     //Derivative Gain 
  constexpr double kWinch_MotionCruiseVelocity = 60000;
  constexpr double kWinch_MotionAcceleration = 120000;
  constexpr double kWinch_MotionSCurveStrength = 5;
  constexpr double kWinch_ExtendLimit = 34500; //max possible move in any mode
  constexpr double kWinch_ExtendCounts = 31150;//extended target in manual mode
  constexpr double kWinch_RetractCounts = 10;  //retracted target in manual mode
  constexpr double kWinch_RetractLimit = 0;    //min possible move in any mode
  constexpr double kWinch_AllowableError = 20;

  constexpr double kContinuousCurrentLimit = 15.0;
  constexpr double kSupplyCurrentLimit = 20.0;
  constexpr double kPeakCurrentLimit = 20.0;
  constexpr double kPeakCurrentDuration = 25; //msecs
  constexpr double kVoltageCompSaturation = 12.0;

  constexpr double kOpenLoopRamp = 1.0;

  constexpr double kArmContinuousCurrentLimit = 20.0;
  constexpr double kArmSupplyCurrentLimit = 20.0;
  constexpr double kArmPeakCurrentLimit = 25.0;
  constexpr double kArmPeakCurrentDuration = 25; //msecs
  constexpr double kArmVoltageCompSaturation = 12.0;

  constexpr double kArm_F = 0.015;    //FeedForward
  constexpr double kArm_P = 0.05;     //Proportional Gain
  constexpr double kArm_I = 0.005;     //Integral Gain
  constexpr double kArm_D = 0.0;    //Derivative Gain 
  constexpr double kArm_AFF = 0.0;  //arbitrary FeedForward to account for gravity on the arm
  constexpr double kArm_MotionCruiseVelocity =50000;
  constexpr double kArm_MotionAcceleration = 100000;
  constexpr double kArm_MotionSCurveStrength = 5;
  
  constexpr double kArm_ForwardLimit = 592;   //max forward movement - should be forward stop
  constexpr double kArm_ReverseLimit = 492;   //min reverse movement - should be reverse stop
  constexpr double kArm_Range = kArm_ForwardLimit - kArm_ReverseLimit;
  constexpr double kArm_AllowableError = 0.03 * kArm_Range;   //deadband around target for position control      

  constexpr double kWrist_F = 0.2;     //FeedForward
  constexpr double kWrist_P = 0.5;     //Proportional Gain
  constexpr double kWrist_I = 0.0;   //Integral Gain
  constexpr double kWrist_D = 0.0;     //Derivative Gain 
  constexpr double kWrist_MotionCruiseVelocity = 5000;
  constexpr double kWrist_MotionAcceleration = 10000;
  constexpr double kWrist_MotionSCurveStrength = 5;

  constexpr double kWristCPR = 4096;
  constexpr double kWristGearRatio = 3.33;
  constexpr double kWristCountsPerRevolution = kWristCPR * kWristGearRatio; 
  constexpr double kWristDegreesToUnits = (kWristCPR * kWristGearRatio) / 360;
  constexpr double kWristUnitsToDegrees = 1 / kWristDegreesToUnits;
  constexpr double kWristRotateCounts = kWristCountsPerRevolution / 4;
        
  constexpr double kIntakeSpeed = 1.0;
  
  //auto profile - drive wheel position - convert Feet to position units (encoder count)
  constexpr double kDriveCPR = 2048;
  constexpr double kDriveGearRatio = 6.75;
  constexpr double kDriveFeetPerRotation = 1.04719;   //4" wheel
  constexpr double kDriveFeetToUnits = (kDriveCPR * kDriveGearRatio) / kDriveFeetPerRotation; 
  constexpr double kDriveUnitsToFeet = 1 / kDriveFeetToUnits;  
  //auto profile - drive wheel speed - convert from Feet/Sec to  units/100ms
  constexpr double kDriveFPSToUnits = ((kDriveFeetPerRotation * kDriveGearRatio) / 10) * kDriveCPR;  
  constexpr double kDriveUnitsToFPS = 1 / kDriveFPSToUnits;    

}