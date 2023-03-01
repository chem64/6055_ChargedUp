#include "Robot.h"

void Robot::RobotInit() 
{
  ntBOSS = nt::NetworkTableInstance::GetDefault().GetTable("dashBOSS");
  ConfigMotors();
  //turn off warnings for joysticks not connected
  frc::DriverStation::SilenceJoystickConnectionWarning(constants::kSilenceJoystickWarnings);

  if(constants::kUseStickBOSS) stickBOSS = new frc::Joystick(0);
  else stickXbox = new frc::XboxController(0);
  stickPlayer = new frc::Joystick(1);
  ArmBrake.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0); 
  frSwerve.name = "FR";
  flSwerve.name = "FL";
  rlSwerve.name = "RL";
  rrSwerve.name = "RR";
  frTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  flTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  rlTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  rrTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  try
  {
    ahrs = new AHRS(frc::SPI::Port::kMXP);
    printf("NAVX Initialized OK");
    HeadingOffset = ahrs->GetYaw();
  }
  catch(const std::exception e)
  {
    printf("!!! NAVX ERROR !!!");
    HeadingOffset = 0;
  }
  AutoRev16_LeftBufStrm = new BufferedTrajectoryPointStream();
  AutoRev16_RightBufStrm = new BufferedTrajectoryPointStream();
  InitBuffer(AutoRev16_LeftBufStrm,auto16_L,auto16_size,false);
  InitBuffer(AutoRev16_RightBufStrm,auto16_R,auto16_size,false);
  AutoFwd2_LeftBufStrm = new BufferedTrajectoryPointStream();
  AutoFwd2_RightBufStrm = new BufferedTrajectoryPointStream();
  InitBuffer(AutoFwd2_LeftBufStrm,auto2_L,auto2_size,true);
  InitBuffer(AutoFwd2_RightBufStrm,auto2_R,auto2_size,true);

  //FMSMatch = frc::DriverStation::IsFMSAttached(); //is this a real match with FMS
  AutoTimer = new frc::Timer();
  AutoTimer->Start();
  ModeTimer = new frc::Timer();
  ModeTimer->Start();

  ClockStart = frc::Timer::GetFPGATimestamp();

}

void Robot::RobotPeriodic() 
{
  //only update this until match starts
  if(!MatchStart)
  {
    static int counter1 = 0;
    counter1++;
    if(counter1 >= 50) //1 secs
    {
      counter1=0;
      //Read AutoSelect and write AutoSelected on BOSS dashboard
      dAutoSelect = (int) ntBOSS->GetNumber("AutoSelect", 0.0);
      ntBOSS->PutNumber("AutoSelected", dAutoSelect);
      //allow BOSS dashboard to reset sticky faults
      int ResetStickyFaults = (int) ntBOSS->GetNumber("ResetStickyFaults", 0);
      if(ResetStickyFaults == 1)
      {
        ntBOSS->PutNumber("ResetStickyFaults", 0);
      }
      ntBOSS->PutNumber("ClockMode", 0);
    }
  }
  static int counter2 = 0;
  counter2++;
  if(counter2 >= 13) //250 msecs
  {
      ClockNow = frc::Timer::GetFPGATimestamp();
      counter2 = 0;
      if(IsAutonomousEnabled())
      {
        ntBOSS->PutNumber("ClockMode", 1);
        ntBOSS->PutNumber("ClockElapsed", std::clamp(15.0 - (double)(ClockNow - ClockStart),0.0,15.0));
      }
      if(IsTeleopEnabled())
      {
        ntBOSS->PutNumber("ClockMode", 2);
        ntBOSS->PutNumber("ClockElapsed", std::clamp(75.0 - (double)(ClockNow - ClockStart),0.0,75.0));
      }
      ntBOSS->PutNumber("CurMode", CurMode);
  }

  if(constants::kDebugValues)
  {
    static int counter3 = 0;
    counter3++;
    if(counter3 >= 50) //1 secs
    {
      counter3=0;
      ntBOSS->PutNumber("FR_DIR",frSwerve.turnPV);
      ntBOSS->PutNumber("FR_ERR",frTurnPID.GetPositionError());
      ntBOSS->PutNumber("FR_DIST", frSwerve.driveOUT);
      ntBOSS->PutNumber("FL_DIR",flSwerve.turnPV);
      ntBOSS->PutNumber("FL_ERR",flTurnPID.GetPositionError());
      ntBOSS->PutNumber("FL_DIST", flSwerve.driveOUT);
      ntBOSS->PutNumber("RL_DIR",rlSwerve.turnPV);
      ntBOSS->PutNumber("RL_ERR",rlTurnPID.GetPositionError());
      ntBOSS->PutNumber("RL_DIST", rlSwerve.driveOUT);
      ntBOSS->PutNumber("RR_DIR",rrSwerve.turnPV);
      ntBOSS->PutNumber("RR_ERR",rrTurnPID.GetPositionError());
      ntBOSS->PutNumber("RR_DIST", rrSwerve.driveOUT);
      ntBOSS->PutNumber("WinchCounts",can_winch1.GetSelectedSensorPosition());
      ntBOSS->PutNumber("Winch",fabs(can_winch1.GetSelectedSensorPosition())/constants::kWinchCountsPerInch);
      ntBOSS->PutNumber("Arm",can_arm.GetSelectedSensorPosition());
      ntBOSS->PutNumber("Wrist",can_wrist.GetSelectedSensorPosition());
      ntBOSS->PutNumber("Wrist_SP",Wrist_SP);
      ntBOSS->PutNumber("Brake",Brake_POS);
      //ntBOSS->PutNumber("joy_FORWARD", forward);
      //ntBOSS->PutNumber("joy_STRAFE", strafe);
      //ntBOSS->PutNumber("joy_ROTATE", rotate);
      ntBOSS->PutNumber("HeadingOffset", HeadingOffset);
      ntBOSS->PutNumber("Heading", Heading);
      ntBOSS->PutNumber("SwerveOrientationToField", SwerveOrientationToField);
      try
      {
        ntBOSS->PutNumber("ahrs_PITCH", ahrs->GetPitch());
        ntBOSS->PutNumber("ahrs_ROLL", ahrs->GetRoll());
      }
      catch(const std::exception e)
      {
        ntBOSS->PutNumber("ahrs_PITCH", 0);
        ntBOSS->PutNumber("ahrs_ROLL", 0);
      }
      ntBOSS->PutString("AutoStatus", "");
    }
  }
}

void Robot::AutonomousInit() 
{
  InitializeSteerAngles();
  //last chance check for auto selection
  dAutoSelect = (int) ntBOSS->GetNumber("AutoSelect", dAutoSelect);
  AutoState = 0;
  CurMode = 1;
}

void Robot::AutonomousPeriodic() 
{
  
  //execute the selected auto routine
  if(dAutoSelect == 1)  RunAuto_1();
  if(dAutoSelect == 2)  RunAuto_2();
}

void Robot::TeleopInit() 
{
  CurMode = 2;
}

void Robot::TeleopPeriodic() 
{
  bool button1_Pressed;
  if(constants::kUseStickBOSS)
  {
    forward = stickBOSS->GetRawAxis(1);
    strafe =  stickBOSS->GetRawAxis(0);
    rotate = stickBOSS->GetRawAxis(3);
    button1_Pressed = stickBOSS->GetRawButtonPressed(1);
  }
  else
  {
    forward = -(stickXbox->GetRightY());
    strafe =  stickXbox->GetRightX();
    rotate = stickXbox->GetLeftX();
    button1_Pressed = stickXbox->GetRawButtonPressed(1);
  }
  
  if(fabs(forward)<.15) {forward = 0;}
  double fforward = spdFilter.Calculate(forward);
  if(forward != 0) forward = fforward;
  if(fabs(strafe)<.15) {strafe = 0;}
  if(fabs(rotate)<.25) {rotate = 0;}

  DriveSwerve(forward, strafe, rotate);

  //toggle robot/field orientation for swerve drive
  if(button1_Pressed) {SwerveOrientationToField = !SwerveOrientationToField;}

  //test rotating arm
  double playerY = stickPlayer->GetRawAxis(1);
  if(fabs(playerY) < 0.15) playerY = 0.0;
  can_arm.Set(ControlMode::PercentOutput,playerY);

  //handle intake wheel rotation
  double player_axis3 = stickPlayer->GetRawAxis(3);
  double intake_speed = 0.0;
  if(player_axis3 < -0.75) intake_speed = -constants::kIntakeSpeed;
  if(player_axis3 > 0.75) intake_speed = constants::kIntakeSpeed;
  can_intake.Set(ControlMode::PercentOutput,intake_speed);

  //handle wrist
  bool player_button3 = stickPlayer->GetRawButtonPressed(3); //left top button
  bool player_button1 = stickPlayer->GetRawButtonPressed(1); //trigger
  bool player_button4 = stickPlayer->GetRawButtonPressed(4); //right top button
  if(player_button1 && !player_button3 && !player_button4) 
  {
    Wrist_SP = 0;
  }
  if(!player_button1 && player_button3 && !player_button4) 
  {
    Wrist_SP = -3141;
  }
  if(!player_button1 && !player_button3 && player_button4) 
  {
    Wrist_SP = 3141;
  }
  can_wrist.Set(ControlMode::MotionMagic,Wrist_SP);

  //handle winch
  double player_axis2 = stickPlayer->GetRawAxis(2)/2;
  //if (fabs(player_axis2) < 0.25) player_axis2 = 0.0;
  can_winch1.Set(ControlMode::PercentOutput,player_axis2);

  double winch_pos = can_winch1.GetSelectedSensorPosition();
  if(fabs(player_axis2)  < 0.15 || (winch_pos < 1500 && player_axis2 < 0.15)) ArmBrake.SetPosition(0.0); //engage brake
  else ArmBrake.SetPosition(1.0); //release brake
  Brake_POS = ArmBrake.GetPosition(); 
}

void Robot::DisabledInit() 
{
  StopAllDrives();
  CurMode = 0;
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() 
{
  CurMode = 3;
}

void Robot::TestPeriodic() 
{
  //Self_Level();
  
  /*To drive fully out, the position is set to 1.0 (2" stroke) 
  To drive half way, the position is set to 0.5 
  To drive fully in, the position is set to 0.0
  ArmBrake.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0); */
  /*static bool BrakeOn;
  if(m_driveController.GetRawButtonPressed(1)) BrakeOn = !BrakeOn;
  if(BrakeOn) ArmBrake.SetPosition(0.0);  //retracted in  brake on?
  if(!BrakeOn) ArmBrake.SetPosition(1.0); //extended out */
}

double Robot::GetHeading()
{
  //subtract the offset recorded at init
  double yaw = 0.0; 
  try
  {
    yaw = ahrs->GetYaw() - HeadingOffset;
    //normalize angle
    yaw = CheckWrap(yaw);
  }
  catch(const std::exception e)
  {
    yaw = 0.0;
  }
  return yaw;
}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif

