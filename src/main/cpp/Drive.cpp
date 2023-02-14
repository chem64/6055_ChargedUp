#include "Robot.h"

//wraps -180 to/from 180
double Robot::CheckWrap(double pos)
{
  double ret = pos;
  ret -= 360. * std::floor((ret + 180.) * (1. / 360.)); 
  return ret;
}

void Robot::StopAllDrives()
{
  can_rrTurn.StopMotor();
  can_frTurn.StopMotor();
  can_rlTurn.StopMotor();
  can_flTurn.StopMotor();
  can_rrDrive.StopMotor();
  can_frDrive.StopMotor();
  can_rlDrive.StopMotor();
  can_flDrive.StopMotor();
}

double Robot::GetEffectiveAngle(double actAngle,double flip)
{
  //get value of actual turn angle
  double ret = actAngle;
  //now get the effective angle (actual direction of movement)
  if (flip < 0) (ret<=0.0)?ret+=180.0:ret-=180.0;
  return ret;
}

void Robot::DriveSwerve(double FWD, double STR, double RCW)
{
 
  double L = constants::kWheelbase; //wheelbase (from center of front wheel to center of back wheel)
  double W = constants::kWheelwidth; //wheelwidth (from center of left wheel to center of right wheel)
  double R = sqrt((L * L) + (W * W));

  //get current heading
  Heading = GetHeading();
  if(SwerveOrientationToField)
  {
    //convert to radians
    double yaw = Heading / constants::kRadtoDeg;
    //recalculate joystick inputs for field orientation
    double tmp = FWD * cos(yaw) + STR * sin(yaw);
    STR = -FWD * sin(yaw) + STR * cos(yaw);
    FWD = tmp;
  }

  double A = STR - RCW * (L / R);
  double B = STR + RCW * (L / R);
  double C = FWD - RCW * (W / R);
  double D = FWD + RCW * (W / R);

  //joystick values are (-180 to 180 degrees)
  double ws1 = sqrt((B * B) + (C * C));
  double wa1 = atan2(B, C) * 180 / M_PI;
  double ws2 = sqrt((B * B) + (D * D));
  double wa2 = atan2(B, D) * 180 / M_PI;
  double ws3 = sqrt((A * A) + (D * D));
  double wa3 = atan2(A, D) * 180 / M_PI;
  double ws4 = sqrt((A * A) + (C * C));
  double wa4 = atan2(A, C) * 180 / M_PI;
 
  //normalize wheel speeds to max speed
  double max = ws1;
  if (ws2 > max)max = ws2;
  if (ws3 > max)max = ws3;
  if (ws4 > max)max = ws4;
  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }

  //if no joystick input - return without changing angles
  if (FWD == 0 && STR == 0 && RCW == 0)
  {
    StopAllDrives();
    return;
  }

  //Front Right
  //set setpoint to joystick
  frSwerve.turnSP = wa1;
  //actual angle is where encoder is pointing
  frSwerve.actAngle = CheckWrap(can_frEncoder.GetPosition()-constants::kFrontRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  frSwerve.turnPV = GetEffectiveAngle(frSwerve.actAngle,frSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(frSwerve.turnSP - frSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
    frSwerve.flip *= -1.0;
    frSwerve.turnPV = GetEffectiveAngle(frSwerve.actAngle,frSwerve.flip);
  }
  //calculate PID based on effective angle and setpoint
  frSwerve.turnOUT = std::clamp(frTurnPID.Calculate(frSwerve.turnPV,frSwerve.turnSP),-1.0,1.0);
  can_frTurn.Set(ControlMode::PercentOutput,frSwerve.turnOUT);
  frSwerve.driveOUT = ws1 * frSwerve.flip;
  can_frDrive.Set(ControlMode::PercentOutput,frSwerve.driveOUT);

  //Front Left
  //set setpoint to joystick
  flSwerve.turnSP = wa2;
  //actual angle is where encoder is pointing
  flSwerve.actAngle = CheckWrap(can_flEncoder.GetPosition()-constants::kFrontLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  flSwerve.turnPV = GetEffectiveAngle(flSwerve.actAngle,flSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(flSwerve.turnSP - flSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
    flSwerve.flip *= -1.0;
    flSwerve.turnPV = GetEffectiveAngle(flSwerve.actAngle,flSwerve.flip);
  }
  //calculate PID based on effective angle and setpoint
  flSwerve.turnOUT = std::clamp(flTurnPID.Calculate(flSwerve.turnPV,flSwerve.turnSP),-1.0,1.0);
  can_flTurn.Set(ControlMode::PercentOutput,flSwerve.turnOUT);
  flSwerve.driveOUT = ws2 * flSwerve.flip;
  can_flDrive.Set(ControlMode::PercentOutput,flSwerve.driveOUT);

  //Rear Left
  //set setpoint to joystick
  rlSwerve.turnSP = wa3;
  //actual angle is where encoder is pointing
  rlSwerve.actAngle = CheckWrap(can_rlEncoder.GetPosition()-constants::kRearLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  rlSwerve.turnPV = GetEffectiveAngle(rlSwerve.actAngle,rlSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(rlSwerve.turnSP - rlSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
    rlSwerve.flip *= -1.0;
    rlSwerve.turnPV = GetEffectiveAngle(rlSwerve.actAngle,rlSwerve.flip);
  }
  //calculate PID based on effective angle and setpoint
  rlSwerve.turnOUT = std::clamp(rlTurnPID.Calculate(rlSwerve.turnPV,rlSwerve.turnSP),-1.0,1.0);
  can_rlTurn.Set(ControlMode::PercentOutput,rlSwerve.turnOUT);
  rlSwerve.driveOUT = ws3 * rlSwerve.flip;
  can_rlDrive.Set(ControlMode::PercentOutput,rlSwerve.driveOUT);

  //Rear Right
  //set setpoint to joystick
  rrSwerve.turnSP = wa4;
  //actual angle is where encoder is pointing
  rrSwerve.actAngle = CheckWrap(can_rrEncoder.GetPosition()-constants::kRearRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  rrSwerve.turnPV = GetEffectiveAngle(rrSwerve.actAngle,rrSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(rrSwerve.turnSP - rrSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
    rrSwerve.flip *= -1.0;
    rrSwerve.turnPV = GetEffectiveAngle(rrSwerve.actAngle,rrSwerve.flip);
  }
  //calculate PID based on effective angle and setpoint
  rrSwerve.turnOUT = std::clamp(rrTurnPID.Calculate(rrSwerve.turnPV,rrSwerve.turnSP),-1.0,1.0);
  can_rrTurn.Set(ControlMode::PercentOutput,rrSwerve.turnOUT);
  rrSwerve.driveOUT = ws4 * rrSwerve.flip;
  can_rrDrive.Set(ControlMode::PercentOutput,rrSwerve.driveOUT);
}

