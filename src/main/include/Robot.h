#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Constants.h"
#include "Auto_2.h"
#include "Auto_16.h"
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Servo.h>

class Robot : public frc::TimedRobot {
 public:
  
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  double GetHeading();
  double CheckWrap(double pos);
  double GetEffectiveAngle(double actAngle,double flip);
  void DriveSwerve(double FWD, double STR, double RCW);
  void ConfigMotors();
  void Self_Level();
  void RunAuto_1();
  bool AutoIsRunning();
  void AutoReset();
  void InitBuffer(BufferedTrajectoryPointStream *bufstrm, const double profile[][2], int totalCnt, bool reverse);
  void ZeroDistance();
  void StopAllDrives();

  private:

    frc::XboxController *m_driveController;
    frc::Joystick *m_stickBOSS;
    std::shared_ptr<nt::NetworkTable> ntBOSS;
    AHRS *ahrs;
    double HeadingOffset = 0;
    double Heading = 0;
    bool SwerveOrientationToField = false;
    
    //CAN Devices
    WPI_TalonFX m_frTurn{constants::kFrontRightTurn_ID};
    WPI_TalonFX m_frDrive{constants::kFrontRightDrive_ID};
    WPI_CANCoder m_frEncoder{constants::kFrontRightEncoder_ID};

    WPI_TalonFX m_flTurn{constants::kFrontLeftTurn_ID};
    WPI_TalonFX m_flDrive{constants::kFrontLeftDrive_ID};
    WPI_CANCoder m_flEncoder{constants::kFrontLeftEncoder_ID};
    
    WPI_TalonFX m_rlTurn{constants::kRearLeftTurn_ID};
    WPI_TalonFX m_rlDrive{constants::kRearLeftDrive_ID};
    WPI_CANCoder m_rlEncoder{constants::kRearLeftEncoder_ID};
    
    WPI_TalonFX m_rrTurn{constants::kRearRightTurn_ID};
    WPI_TalonFX m_rrDrive{constants::kRearRightDrive_ID};
    WPI_CANCoder m_rrEncoder{constants::kRearRightEncoder_ID};

    WPI_TalonSRX m_winch1{constants::kWinch1_ID};

    frc::Servo ArmBrake{0};         

    //SWERVE

    struct SwerveType
    {
      double actAngle = 0.0;  //current encoder position - encoder offset
      double turnSP = 0.0;    //desired direction from joystick
      double turnPV = 0.0;    //effective angle - direction wheel is moving
      double flip = 1.0;      //-1 if drive is reversed, 1 if drive is not reversed
      double turnOUT = 0.0;   //output to turn motor
      double driveOUT = 0.0;  //output to drive motor
    };

    SwerveType frSwerve;
    SwerveType flSwerve;
    SwerveType rlSwerve;
    SwerveType rrSwerve;
    
    frc2::PIDController m_TurnPID{constants::kTurn_KP, constants::kTurn_KI, constants::kTurn_KD};
    frc::SlewRateLimiter<units::scalar> spdFilter{2/1_s};
       
    double forward;
    double strafe;
    double rotate;
    
    SupplyCurrentLimitConfiguration driveSCLC;
    StatorCurrentLimitConfiguration driveStatorSCLC;
    SupplyCurrentLimitConfiguration turnSCLC;
    StatorCurrentLimitConfiguration turnStatorSCLC; 

    //Auto
    int AutoState = 0;
    frc::Timer *AutoTimer;
    frc::Timer *ModeTimer;
    units::time::second_t ClockStart;
    units::time::second_t ClockNow;
    
    BufferedTrajectoryPointStream *AutoRev16_LeftBufStrm;
    BufferedTrajectoryPointStream *AutoRev16_RightBufStrm;
    BufferedTrajectoryPointStream *AutoFwd2_LeftBufStrm;
    BufferedTrajectoryPointStream *AutoFwd2_RightBufStrm;

    //dashboard variables
    bool FMSMatch = false;  //true if attached to FMS
    bool MatchStart = false;
    int dAutoSelect = 0;
    int CurMode = 0;

  };
