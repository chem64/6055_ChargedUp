#include "Robot.h"

enum Constants {kSlotIdx = 0, kPIDLoopIdx = 0, kTimeoutMs = 50};


void Robot::ConfigMotors()
{
  driveSCLC = SupplyCurrentLimitConfiguration{true,constants::kDriveSupplyCurrentLimit,constants::kDrivePeakCurrentLimit,constants::kDrivePeakCurrentDuration};
  driveStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kDriveStatorCurrentLimit,constants::kDriveStatorPeakCurrentLimit,constants::kDriveStatorPeakCurrentDuration};
  turnSCLC = SupplyCurrentLimitConfiguration{true,constants::kTurnSupplyCurrentLimit,constants::kTurnPeakCurrentLimit,constants::kTurnPeakCurrentDuration};
  turnStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kTurnStatorCurrentLimit,constants::kTurnStatorPeakCurrentLimit,constants::kTurnStatorPeakCurrentDuration};
  armSCLC = SupplyCurrentLimitConfiguration{true,constants::kArmSupplyCurrentLimit,constants::kArmPeakCurrentLimit,constants::kArmPeakCurrentDuration};

  can_frDrive.ConfigFactoryDefault(kTimeoutMs);
  can_frDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_frDrive.SetInverted(true);
  can_frDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_frDrive.SetNeutralMode(NeutralMode::Brake);
  can_frDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_frDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_frDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_frDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_frDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_frDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_frDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_frDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_frDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_frDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_frDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_frDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_frDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_frDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_frDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_frDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_frDrive.EnableVoltageCompensation(true);
  can_frDrive.SetSafetyEnabled(false);

  can_flDrive.ConfigFactoryDefault(kTimeoutMs);
  can_flDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_flDrive.SetInverted(true);
  can_flDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_flDrive.SetNeutralMode(NeutralMode::Brake);
  can_flDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_flDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_flDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_flDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_flDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_flDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_flDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_flDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_flDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_flDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_flDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_flDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_flDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_flDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_flDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_flDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_flDrive.EnableVoltageCompensation(true);
  can_flDrive.SetSafetyEnabled(false);

  can_rlDrive.ConfigFactoryDefault(kTimeoutMs);
  can_rlDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_rlDrive.SetInverted(true);
  can_rlDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_rlDrive.SetNeutralMode(NeutralMode::Brake);
  can_rlDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_rlDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_rlDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_rlDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_rlDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_rlDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_rlDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_rlDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_rlDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_rlDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_rlDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_rlDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_rlDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_rlDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_rlDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_rlDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_rlDrive.EnableVoltageCompensation(true);
  can_rlDrive.SetSafetyEnabled(false);

  can_rrDrive.ConfigFactoryDefault(kTimeoutMs);
  can_rrDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_rrDrive.SetInverted(true);
  can_rrDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_rrDrive.SetNeutralMode(NeutralMode::Brake);
  can_rrDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_rrDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_rrDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_rrDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_rrDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_rrDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_rrDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_rrDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_rrDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_rrDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_rrDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_rrDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_rrDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_rrDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_rrDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_rrDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_rrDrive.EnableVoltageCompensation(true);
  can_rrDrive.SetSafetyEnabled(false);

  can_frEncoder.ConfigFactoryDefault();
  can_frEncoder.ConfigSensorDirection(true,10);
  can_frEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_frEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_frEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_frTurn.ConfigFactoryDefault();
  can_frTurn.SetNeutralMode(NeutralMode::Brake);
  can_frTurn.ConfigNominalOutputForward(0);
	can_frTurn.ConfigNominalOutputReverse(0);
	can_frTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_frTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_frTurn.SetSafetyEnabled(false);
  can_frTurn.SetInverted(true);
  can_frTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_frTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_frTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_frTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);
  
  can_flEncoder.ConfigFactoryDefault();
  can_flEncoder.ConfigSensorDirection(true,10);
  can_flEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_flEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_flEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_flTurn.ConfigFactoryDefault();
  can_flTurn.SetNeutralMode(NeutralMode::Brake);
  can_flTurn.ConfigNominalOutputForward(0);
	can_flTurn.ConfigNominalOutputReverse(0);
	can_flTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_flTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_flTurn.SetSafetyEnabled(false);
  can_flTurn.SetInverted(true);
  can_flTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_flTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);

  can_rlEncoder.ConfigFactoryDefault();
  can_rlEncoder.ConfigSensorDirection(true,10);
  can_rlEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_rlEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_rlEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_rlTurn.ConfigFactoryDefault();
  can_rlTurn.SetNeutralMode(NeutralMode::Brake);
  can_rlTurn.ConfigNominalOutputForward(0);
	can_rlTurn.ConfigNominalOutputReverse(0);
	can_rlTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_rlTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_rlTurn.SetSafetyEnabled(false);
  can_rlTurn.SetInverted(true);
  can_rlTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_rlTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);

  can_rrEncoder.ConfigFactoryDefault();
  can_rrEncoder.ConfigSensorDirection(true,10);
  can_rrEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_rrEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_rrEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_rrTurn.ConfigFactoryDefault();
  can_rrTurn.SetNeutralMode(NeutralMode::Brake);
  can_rrTurn.ConfigNominalOutputForward(0);
	can_rrTurn.ConfigNominalOutputReverse(0);
	can_rrTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_rrTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_rrTurn.SetSafetyEnabled(false);
  can_rrTurn.SetInverted(true);
  can_rrTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_rrTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);

  can_winch1.ConfigFactoryDefault(kTimeoutMs);
  can_winch1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,kTimeoutMs);
  can_winch1.SetNeutralMode(NeutralMode::Brake);
  can_winch1.SetSensorPhase(false);
  can_winch1.SetInverted(false);
  can_winch1.SetSafetyEnabled(false);
  can_winch1.ConfigPeakCurrentLimit(constants::kPeakCurrentLimit, kTimeoutMs);
  can_winch1.ConfigPeakCurrentDuration(constants::kPeakCurrentDuration, kTimeoutMs);
  can_winch1.ConfigContinuousCurrentLimit(constants::kContinuousCurrentLimit, kTimeoutMs);
  can_winch1.EnableCurrentLimit(true);
  can_winch1.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_winch1.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_winch1.ConfigNominalOutputForward(0, kTimeoutMs);
  can_winch1.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_winch1.ConfigPeakOutputForward(1, kTimeoutMs);
  can_winch1.ConfigPeakOutputReverse(-1, kTimeoutMs);
  can_winch1.Config_kF(0, constants::kWinch_F, kTimeoutMs);
  can_winch1.Config_kP(0, constants::kWinch_P, kTimeoutMs);
  can_winch1.Config_kI(0, constants::kWinch_I, kTimeoutMs);
  can_winch1.Config_kD(0, constants::kWinch_D, kTimeoutMs);
  can_winch1.Config_IntegralZone(0, 400, kTimeoutMs);
  can_winch1.ConfigAllowableClosedloopError(0,constants::kWinch_AllowableError,kTimeoutMs); 
  can_winch1.ConfigMotionCruiseVelocity(constants::kWinch_MotionCruiseVelocity, kTimeoutMs);
  can_winch1.ConfigMotionAcceleration(constants::kWinch_MotionAcceleration, kTimeoutMs);
  can_winch1.ConfigMotionSCurveStrength(constants::kWinch_MotionSCurveStrength, kTimeoutMs);
  can_winch1.ConfigVoltageCompSaturation(constants::kVoltageCompSaturation);
  can_winch1.EnableVoltageCompensation(true);
  can_winch1.ConfigForwardSoftLimitThreshold(constants::kWinch_ExtendLimit);
  can_winch1.ConfigReverseSoftLimitThreshold(constants::kWinch_RetractLimit);
  can_winch1.ConfigForwardSoftLimitEnable(true);
  can_winch1.ConfigReverseSoftLimitEnable(true);
  
  can_winch2.ConfigFactoryDefault(kTimeoutMs);
  can_winch2.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,kTimeoutMs);
  can_winch2.SetNeutralMode(NeutralMode::Brake);
  can_winch2.SetSensorPhase(false);
  can_winch2.SetInverted(false);
  can_winch2.SetSafetyEnabled(false);
  can_winch2.ConfigPeakCurrentLimit(constants::kPeakCurrentLimit, kTimeoutMs);
  can_winch2.ConfigPeakCurrentDuration(constants::kPeakCurrentDuration, kTimeoutMs);
  can_winch2.ConfigContinuousCurrentLimit(constants::kContinuousCurrentLimit, kTimeoutMs);
  can_winch2.EnableCurrentLimit(true);
  can_winch2.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_winch2.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_winch2.ConfigNominalOutputForward(0, kTimeoutMs);
  can_winch2.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_winch2.ConfigPeakOutputForward(1, kTimeoutMs);
  can_winch2.ConfigPeakOutputReverse(-1, kTimeoutMs);
  can_winch2.Config_kF(0, constants::kWinch_F, kTimeoutMs);
  can_winch2.Config_kP(0, constants::kWinch_P, kTimeoutMs);
  can_winch2.Config_kI(0, constants::kWinch_I, kTimeoutMs);
  can_winch2.Config_kD(0, constants::kWinch_D, kTimeoutMs);
  can_winch2.Config_IntegralZone(0, 400, kTimeoutMs);
  can_winch2.ConfigAllowableClosedloopError(0,constants::kWinch_AllowableError,kTimeoutMs); 
  can_winch2.ConfigMotionCruiseVelocity(constants::kWinch_MotionCruiseVelocity, kTimeoutMs);
  can_winch2.ConfigMotionAcceleration(constants::kWinch_MotionAcceleration, kTimeoutMs);
  can_winch2.ConfigMotionSCurveStrength(constants::kWinch_MotionSCurveStrength, kTimeoutMs);
  can_winch2.ConfigVoltageCompSaturation(constants::kVoltageCompSaturation);
  can_winch2.EnableVoltageCompensation(true);
  can_winch2.ConfigForwardSoftLimitThreshold(constants::kWinch_ExtendLimit);
  can_winch2.ConfigReverseSoftLimitThreshold(constants::kWinch_RetractLimit);
  can_winch2.ConfigForwardSoftLimitEnable(true);
  can_winch2.ConfigReverseSoftLimitEnable(true);

  can_intake.ConfigFactoryDefault(kTimeoutMs);
  can_intake.ConfigOpenloopRamp(constants::kOpenLoopRamp, kTimeoutMs);
  can_intake.SetNeutralMode(NeutralMode::Brake);
  can_intake.SetInverted(true);
  can_intake.SetSafetyEnabled(false);
  can_intake.ConfigPeakCurrentLimit(constants::kPeakCurrentLimit, kTimeoutMs);
  can_intake.ConfigPeakCurrentDuration(constants::kPeakCurrentDuration, kTimeoutMs);
  can_intake.ConfigContinuousCurrentLimit(constants::kContinuousCurrentLimit, kTimeoutMs);
  can_intake.EnableCurrentLimit(true);
  can_intake.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_intake.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);
  can_intake.ConfigVoltageCompSaturation(constants::kVoltageCompSaturation);
  can_intake.EnableVoltageCompensation(true);

  can_arm.ConfigFactoryDefault(kTimeoutMs);
  can_arm.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_arm.SetNeutralMode(NeutralMode::Brake);
  can_arm.SetInverted(true);
  can_arm.SetSafetyEnabled(false);
  can_arm.ConfigSupplyCurrentLimit(armSCLC); //current limiting
  can_arm.ConfigOpenloopRamp(1.0, kTimeoutMs);
  can_arm.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_arm.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_arm.ConfigNominalOutputForward(0, kTimeoutMs);
  can_arm.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_arm.ConfigPeakOutputForward(1, kTimeoutMs);
  can_arm.ConfigPeakOutputReverse(-1, kTimeoutMs);
  can_arm.Config_kF(0, constants::kArm_F, kTimeoutMs);
  can_arm.Config_kP(0, constants::kArm_P, kTimeoutMs);
  can_arm.Config_kI(0, constants::kArm_I, kTimeoutMs);
  can_arm.Config_kD(0, constants::kArm_D, kTimeoutMs);
  can_arm.Config_IntegralZone(0, 400, kTimeoutMs);
  can_arm.ConfigAllowableClosedloopError(0,constants::kArm_AllowableError,kTimeoutMs);  
  can_arm.ConfigMotionCruiseVelocity(constants::kArm_MotionCruiseVelocity, 10);
  can_arm.ConfigMotionAcceleration(constants::kArm_MotionAcceleration, 10);
  can_arm.ConfigMotionSCurveStrength(constants::kArm_MotionSCurveStrength);
  can_arm.ConfigVoltageCompSaturation(constants::kVoltageCompSaturation);
  can_arm.EnableVoltageCompensation(true);
  can_arm.ConfigForwardSoftLimitThreshold(constants::kArm_ForwardLimit);
  can_arm.ConfigReverseSoftLimitThreshold(constants::kArm_ReverseLimit);
  can_arm.ConfigForwardSoftLimitEnable(true);
  can_arm.ConfigReverseSoftLimitEnable(true);

  can_wrist.ConfigFactoryDefault(kTimeoutMs);
  can_wrist.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,kTimeoutMs);
  can_wrist.SetNeutralMode(NeutralMode::Brake);
  can_wrist.SetSensorPhase(false);
  can_wrist.SetInverted(false);
  can_wrist.SetSafetyEnabled(false);
  can_wrist.ConfigPeakCurrentLimit(constants::kPeakCurrentLimit, kTimeoutMs);
  can_wrist.ConfigPeakCurrentDuration(constants::kPeakCurrentDuration, kTimeoutMs);
  can_wrist.ConfigContinuousCurrentLimit(constants::kContinuousCurrentLimit, kTimeoutMs);
  can_wrist.EnableCurrentLimit(true);
  can_wrist.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_wrist.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_wrist.ConfigNominalOutputForward(0, kTimeoutMs);
  can_wrist.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_wrist.ConfigPeakOutputForward(1, kTimeoutMs);
  can_wrist.ConfigPeakOutputReverse(1, kTimeoutMs);  
  can_wrist.Config_kF(0, constants::kWrist_F, kTimeoutMs);
  can_wrist.Config_kP(0, constants::kWrist_P, kTimeoutMs);
  can_wrist.Config_kI(0, constants::kWrist_I, kTimeoutMs);
  can_wrist.Config_kD(0, constants::kWrist_D, kTimeoutMs);
  can_wrist.Config_IntegralZone(0, 400, kTimeoutMs);
  can_wrist.ConfigAllowableClosedloopError(0,10,kTimeoutMs);  
  can_wrist.ConfigMotionCruiseVelocity(constants::kWrist_MotionCruiseVelocity, kTimeoutMs);
  can_wrist.ConfigMotionAcceleration(constants::kWrist_MotionAcceleration, kTimeoutMs);
  can_wrist.ConfigMotionSCurveStrength(constants::kWrist_MotionSCurveStrength, kTimeoutMs);
  can_wrist.ConfigVoltageCompSaturation(constants::kVoltageCompSaturation);
  can_wrist.EnableVoltageCompensation(true);
  
}
