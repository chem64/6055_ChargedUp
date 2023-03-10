#include "Robot.h"

bool Robot::AutoIsRunning()
{
  if (!can_flDrive.IsMotionProfileFinished() && !can_frDrive.IsMotionProfileFinished())
    return true;
  else return false;
}

void Robot::SteerAuto()
{
  frSwerve.turnSP = 0.0;
  frSwerve.turnPV = CheckWrap(can_frEncoder.GetPosition()-constants::kFrontRightOffset);
  frSwerve.turnOUT = std::clamp(frTurnPID.Calculate(frSwerve.turnPV,frSwerve.turnSP),-1.0,1.0);
  can_frTurn.Set(ControlMode::PercentOutput,frSwerve.turnOUT);
  flSwerve.turnSP = 0.0;
  flSwerve.turnPV = CheckWrap(can_flEncoder.GetPosition()-constants::kFrontLeftOffset);
  flSwerve.turnOUT = std::clamp(flTurnPID.Calculate(flSwerve.turnPV,flSwerve.turnSP),-1.0,1.0);
  can_flTurn.Set(ControlMode::PercentOutput,flSwerve.turnOUT);
  rlSwerve.turnSP = 0.0;
  rlSwerve.turnPV = CheckWrap(can_rlEncoder.GetPosition()-constants::kRearLeftOffset);
  rlSwerve.turnOUT = std::clamp(rlTurnPID.Calculate(rlSwerve.turnPV,rlSwerve.turnSP),-1.0,1.0);
  can_rlTurn.Set(ControlMode::PercentOutput,rlSwerve.turnOUT);
  rrSwerve.turnSP = 0.0;
  rrSwerve.turnPV = CheckWrap(can_rrEncoder.GetPosition()-constants::kRearRightOffset);
  rrSwerve.turnOUT = std::clamp(rrTurnPID.Calculate(rrSwerve.turnPV,rrSwerve.turnSP),-1.0,1.0);
  can_rrTurn.Set(ControlMode::PercentOutput,rrSwerve.turnOUT);
}

void Robot::ZeroDistance()
{
  //set ticks equal to zero
  can_frDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
  can_flDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
  can_rlDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
  can_rrDrive.GetSensorCollection().SetIntegratedSensorPosition(0,10);
}

void Robot::AutoReset()
{
  ZeroDistance();
  ntBOSS->PutString("AutoStatus", "RESET");
}

void Robot::Self_Level()
{
    double driveOut = 0.0;
    double rollDeg = ahrs->GetRoll();

    if(fabs(rollDeg) > constants::kSelfLevelDeadband)
    {
        driveOut = sin(rollDeg) * constants::kSelfLevelGain;  //roll in reverse direction of declining angle - drive toward the tilt up
        can_frDrive.Set(ControlMode::PercentOutput,std::clamp(driveOut,-1.0,1.0));
        can_flDrive.Set(ControlMode::PercentOutput,std::clamp(driveOut,-1.0,1.0));
        can_rlDrive.Set(ControlMode::PercentOutput,std::clamp(driveOut,-1.0,1.0));
        can_rrDrive.Set(ControlMode::PercentOutput,std::clamp(driveOut,-1.0,1.0));
    }
    else
    {
        can_frDrive.Set(ControlMode::PercentOutput,0.0);
        can_flDrive.Set(ControlMode::PercentOutput,0.0);
        can_rlDrive.Set(ControlMode::PercentOutput,0.0);
        can_rrDrive.Set(ControlMode::PercentOutput,0.0);
    }
}

void Robot::InitBuffer(BufferedTrajectoryPointStream *bufstrm, const double profile[][2], int totalCnt, bool reverse)
{
    bool forward = !reverse; 
    TrajectoryPoint point; 

    bufstrm->Clear(); // clear the buffer, in case it was used elsewhere 
    for (int i = 0; i < totalCnt; ++i) // Insert every point into buffer
    {
        double direction = forward ? +1 : -1;
        double positionFeet = profile[i][0];
        double velocityFeetPerSec = profile[i][1];
        int durationMilliseconds = 10; 
        
        // for each point, fill our structure and pass it to API 
        point.timeDur = durationMilliseconds;
        point.position = direction * positionFeet * constants::kDriveFeetToUnits; //Convert Feet to Units
        point.velocity = direction * velocityFeetPerSec * constants::kDriveFPSToUnits; //Convert Feet/sec to Units/100ms
        point.auxiliaryPos = 0;
        point.auxiliaryVel = 0;
        point.profileSlotSelect0 = 0; 
        point.profileSlotSelect1 = 0; 
        point.zeroPos = (i == 0); // set this to true on the first point 
        point.isLastPoint = ((i + 1) == totalCnt); // set this to true on the last point 
        point.arbFeedFwd = 0; // you can add a constant offset to add to PID[0] output here 
        bufstrm->Write(point);
    }
}

//place object, reverse 16.5 ft, forward 2.5 ft, balance on table
void::Robot::RunAuto_1() 
{
    switch(AutoState)
    {
        case 0:
            AutoTimer->Reset();
            AutoState = 10;
            ntBOSS->PutString("AutoStatus", "AUTO1_PLACE");
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 10: //place object
            if(AutoTimer->AdvanceIfElapsed((units::time::second_t) 1))
            {
                AutoState = 20;
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 20: //start drive control
            ntBOSS->PutString("AutoStatus", "AUTO1_REV16");
            can_flDrive.StartMotionProfile(*AutoRev16_LeftBufStrm, 10, ControlMode::MotionProfile);
            can_rlDrive.StartMotionProfile(*AutoRev16_LeftBufStrm, 10, ControlMode::MotionProfile);
            can_frDrive.StartMotionProfile(*AutoRev16_RightBufStrm, 10, ControlMode::MotionProfile);
            can_rrDrive.StartMotionProfile(*AutoRev16_RightBufStrm, 10, ControlMode::MotionProfile);
            AutoState = 30;
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 30: //process drive control
            //look for end of profile
            if(!AutoIsRunning())
            {
                AutoState = 40;
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 40: //start drive control
            ntBOSS->PutString("AutoStatus", "AUTO1_FWD2");
            can_flDrive.StartMotionProfile(*AutoFwd2_LeftBufStrm, 10, ControlMode::MotionProfile);
            can_rlDrive.StartMotionProfile(*AutoFwd2_LeftBufStrm, 10, ControlMode::MotionProfile);
            can_frDrive.StartMotionProfile(*AutoFwd2_RightBufStrm, 10, ControlMode::MotionProfile);
            can_rrDrive.StartMotionProfile(*AutoFwd2_RightBufStrm, 10, ControlMode::MotionProfile);
            AutoState = 50;
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 50: //process drive control
            //look for end of profile
            if(!AutoIsRunning())
            {
                AutoState = 60;
                ntBOSS->PutString("AutoStatus", "LEVEL UP");
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 60: //level up on charge table
            Self_Level();
            break;
        case 70:
            ntBOSS->PutString("AutoStatus", "AUTO1_DONE");
            StopAllDrives();
            break;
    }
   
}

//place object, reverse 16.5 ft, stop
void::Robot::RunAuto_2() 
{
    switch(AutoState)
    {
        case 0:
            AutoTimer->Reset();
            AutoState = 10;
            ntBOSS->PutString("AutoStatus", "AUTO2_PLACE");
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 10: //place object
            if(AutoTimer->AdvanceIfElapsed((units::time::second_t) 1))
            {
                AutoState = 20;
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 20: //start drive control
            ntBOSS->PutString("AutoStatus", "AUTO2_REV16");
            can_flDrive.StartMotionProfile(*AutoRev16_LeftBufStrm, 10, ControlMode::MotionProfile);
            can_rlDrive.StartMotionProfile(*AutoRev16_LeftBufStrm, 10, ControlMode::MotionProfile);
            can_frDrive.StartMotionProfile(*AutoRev16_RightBufStrm, 10, ControlMode::MotionProfile);
            can_rrDrive.StartMotionProfile(*AutoRev16_RightBufStrm, 10, ControlMode::MotionProfile);
            AutoState = 30;
            //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            break;
        case 30: //process drive control
            //look for end of profile
            if(!AutoIsRunning())
            {
                AutoState = 40;
                //if(constants::kAutoLoggingEnabled) ntBOSS->PutNumber("AutoState", AutoState);
            }
            break;
        case 40:
            ntBOSS->PutString("AutoStatus", "AUTO2_DONE");
            StopAllDrives();
            break;
    }
   
}
