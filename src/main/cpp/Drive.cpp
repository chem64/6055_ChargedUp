#include "Robot.h"

//wraps -180 to/from 180
double Robot::CheckWrap(double pos)
{
  double ret = pos;
  ret -= 360.0 * std::floor((ret + 180.0) * (1.0 / 360.0)); 
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

//reverses angle by 180 degrees
double flip180(double angle)
{
  double ret = angle;

  if(ret <= 0.0) ret += 180.0;
  else ret -= 180.0;
  return ret;
}

//gets direction drive is moving
double Robot::GetEffectiveAngle(double actAngle,double flip)
{
  //get value of actual turn angle
  double ret = actAngle;
  //now get the effective angle (actual direction of movement)
  //if (flip < 0) (ret<=0.0)?ret+=180.0:ret-=180.0;
  if(flip < 0.0) ret = flip180(actAngle);
  return ret;
}

void Robot::PrintAngles(SwerveType st)
{
  if(constants::kDebugValues)
  {
    printf("%s: ang=%.1f asp=%.1f apv=%.1f inF=%.1f\n",st.name.c_str(),st.actAngle,st.actSP,st.actPV,st.inFlip);
    printf("sp=%.1f pv=%.1f outF=%.1f outT=%.1f outD=%.1f\n",st.turnSP,st.turnPV,st.outFlip,st.turnOUT,st.driveOUT);
  }
}

void Robot::InitializeSteerAngles()
{
  frSwerve.actAngle = CheckWrap(can_frEncoder.GetPosition()-constants::kFrontRightOffset);  
  frSwerve.actPV = GetEffectiveAngle(frSwerve.actAngle,frSwerve.inFlip);
  frSwerve.actSP = frSwerve.actPV;
  frSwerve.turnSP = frSwerve.actPV;
  frSwerve.turnPV = frSwerve.actPV;
  flSwerve.actAngle = CheckWrap(can_flEncoder.GetPosition()-constants::kFrontLeftOffset);  
  flSwerve.actPV = GetEffectiveAngle(flSwerve.actAngle,flSwerve.inFlip);
  flSwerve.actSP = flSwerve.actPV;
  flSwerve.turnSP = flSwerve.actPV;
  flSwerve.turnPV = flSwerve.actPV;
  rlSwerve.actAngle = CheckWrap(can_rlEncoder.GetPosition()-constants::kRearLeftOffset);  
  rlSwerve.actPV = GetEffectiveAngle(rlSwerve.actAngle,rlSwerve.inFlip);
  rlSwerve.actSP = rlSwerve.actPV;
  rlSwerve.turnSP = rlSwerve.actPV;
  rlSwerve.turnPV = rlSwerve.actPV;
  rrSwerve.actAngle = CheckWrap(can_rrEncoder.GetPosition()-constants::kRearRightOffset);  
  rrSwerve.actPV = GetEffectiveAngle(rrSwerve.actAngle,rrSwerve.inFlip);
  rrSwerve.actSP = rrSwerve.actPV;
  rrSwerve.turnSP = rrSwerve.actPV;
  rrSwerve.turnPV = rrSwerve.actPV;
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

  //Get wheel angles
  frSwerve.actAngle = CheckWrap(can_frEncoder.GetPosition()-constants::kFrontRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  frSwerve.actPV = GetEffectiveAngle(frSwerve.actAngle,frSwerve.inFlip);
  frSwerve.outFlip = frSwerve.inFlip;
  frSwerve.turnPV = frSwerve.actPV;
  flSwerve.actAngle = CheckWrap(can_flEncoder.GetPosition()-constants::kFrontLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  flSwerve.actPV = GetEffectiveAngle(flSwerve.actAngle,flSwerve.inFlip);
  flSwerve.outFlip = flSwerve.inFlip;
  flSwerve.turnPV = flSwerve.actPV;
  rlSwerve.actAngle = CheckWrap(can_rlEncoder.GetPosition()-constants::kRearLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  rlSwerve.actPV = GetEffectiveAngle(rlSwerve.actAngle,rlSwerve.inFlip);
  rlSwerve.outFlip = rlSwerve.inFlip;
  rlSwerve.turnPV = rlSwerve.actPV;  
  rrSwerve.actAngle = CheckWrap(can_rrEncoder.GetPosition()-constants::kRearRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  rrSwerve.actPV = GetEffectiveAngle(rrSwerve.actAngle,rrSwerve.inFlip);
  rrSwerve.outFlip = rrSwerve.inFlip;
  rrSwerve.turnPV = rrSwerve.actPV;
    
  //if no joystick input - don't change turn setpoints and zero speeds
  if (FWD == 0 && STR == 0 && RCW == 0)
  {
    ws1 = 0.0;
    ws2 = 0.0;
    ws3 = 0.0;
    ws4 = 0.0;
  }
  else //calculate turn setpoints and new effective angles
  {
    //Front Right
    frSwerve.actSP = wa1;
    frSwerve.turnSP = std::clamp(wa1,-180.0,180.0);
    if (CheckWrap(fabs(frSwerve.turnSP - frSwerve.actPV)) > constants::kSwerveAngleBreak)
    {
      frSwerve.outFlip = frSwerve.inFlip *= -1.0;
      frSwerve.turnPV = flip180(frSwerve.actPV);
    }
    
    //Front Left
    flSwerve.actSP = wa2;
    flSwerve.turnSP = std::clamp(wa2,-180.0,180.0);
    if (CheckWrap(fabs(flSwerve.turnSP - flSwerve.actPV)) > constants::kSwerveAngleBreak)
    {
      flSwerve.outFlip = flSwerve.inFlip *= -1.0;
      flSwerve.turnPV = flip180(flSwerve.actPV);
    }

    //Rear Left
    rlSwerve.actSP = wa3;
    rlSwerve.turnSP = std::clamp(wa3,-180.0,180.0);
    if (CheckWrap(fabs(rlSwerve.turnSP - rlSwerve.actPV)) > constants::kSwerveAngleBreak)
    {
      rlSwerve.outFlip = rlSwerve.inFlip *= -1.0;
      rlSwerve.turnPV = flip180(rlSwerve.actPV);
    }

    //Rear Right
    rrSwerve.actSP = wa4;
    rrSwerve.turnSP = std::clamp(wa4,-180.0,180.0);
    if (CheckWrap(fabs(rrSwerve.turnSP - rrSwerve.actPV)) > constants::kSwerveAngleBreak)
    {
      rrSwerve.outFlip = rrSwerve.inFlip *= -1.0;
      rrSwerve.turnPV = flip180(rrSwerve.actPV);
    }
  }
  
  //calculate turn PID based on effective angle and setpoint
  //output to drives
  frSwerve.turnOUT = std::clamp(frTurnPID.Calculate(frSwerve.turnPV,frSwerve.turnSP),-1.0,1.0);
  can_frTurn.Set(ControlMode::PercentOutput,frSwerve.turnOUT);
  frSwerve.driveOUT = ws1 * frSwerve.outFlip;
  can_frDrive.Set(ControlMode::PercentOutput,frSwerve.driveOUT);
  if(frTurnPID.GetPositionError() > 90.) PrintAngles(frSwerve);
  frSwerve.inFlip = frSwerve.outFlip;
    
  flSwerve.turnOUT = std::clamp(flTurnPID.Calculate(flSwerve.turnPV,flSwerve.turnSP),-1.0,1.0);
  can_flTurn.Set(ControlMode::PercentOutput,flSwerve.turnOUT);
  flSwerve.driveOUT = ws2 * flSwerve.outFlip;
  can_flDrive.Set(ControlMode::PercentOutput,flSwerve.driveOUT);
  if(flTurnPID.GetPositionError() > 90.) PrintAngles(flSwerve);
  flSwerve.inFlip = flSwerve.outFlip;
    
  rlSwerve.turnOUT = std::clamp(rlTurnPID.Calculate(rlSwerve.turnPV,rlSwerve.turnSP),-1.0,1.0);
  can_rlTurn.Set(ControlMode::PercentOutput,rlSwerve.turnOUT);
  rlSwerve.driveOUT = ws3 * rlSwerve.outFlip;
  can_rlDrive.Set(ControlMode::PercentOutput,rlSwerve.driveOUT);
  if(rlTurnPID.GetPositionError() > 90.) PrintAngles(rlSwerve);
  rlSwerve.inFlip = rlSwerve.outFlip;
  
  rrSwerve.turnOUT = std::clamp(rrTurnPID.Calculate(rrSwerve.turnPV,rrSwerve.turnSP),-1.0,1.0);
  can_rrTurn.Set(ControlMode::PercentOutput,rrSwerve.turnOUT);
  rrSwerve.driveOUT = ws4 * rrSwerve.outFlip;
  can_rrDrive.Set(ControlMode::PercentOutput,rrSwerve.driveOUT);
  if(rrTurnPID.GetPositionError() > 90.) PrintAngles(rrSwerve);
  rrSwerve.inFlip = rrSwerve.outFlip;
}
