// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PID.h"

#include <algorithm>
#include <cmath>

#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "frc/MathUtil.h"
#include "wpimath/MathShared.h"

using namespace frc2;

PIDControllerX::PIDControllerX(double Kp, double Ki, double Kd,
                             units::second_t period)
    : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_period(period) {
  if (period <= 0_s) {
    wpi::math::MathSharedStore::ReportError(
        "Controller period must be a non-zero positive number, got {}!",
        period.value());
    m_period = 20_ms;
    wpi::math::MathSharedStore::ReportWarning(
        "Controller period defaulted to 20ms.");
  }
  static int instances = 0;
  instances++;

  wpi::math::MathSharedStore::ReportUsage(
      wpi::math::MathUsageId::kController_PIDController2, instances);
  wpi::SendableRegistry::Add(this, "PIDController", instances);
}

void PIDControllerX::SetPID(double Kp, double Ki, double Kd) {
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;
}

void PIDControllerX::SetP(double Kp) {
  m_Kp = Kp;
}

void PIDControllerX::SetI(double Ki) {
  m_Ki = Ki;
}

void PIDControllerX::SetD(double Kd) {
  m_Kd = Kd;
}

double PIDControllerX::GetP() const {
  return m_Kp;
}

double PIDControllerX::GetI() const {
  return m_Ki;
}

double PIDControllerX::GetD() const {
  return m_Kd;
}

units::second_t PIDControllerX::GetPeriod() const {
  return m_period;
}

double PIDControllerX::GetPositionTolerance() const {
  return m_positionTolerance;
}

double PIDControllerX::GetVelocityTolerance() const {
  return m_velocityTolerance;
}

void PIDControllerX::SetSetpoint(double setpoint) {
  m_setpoint = setpoint;
  m_haveSetpoint = true;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError =
        frc::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period.value();
}

double PIDControllerX::GetSetpoint() const {
  return m_setpoint;
}

bool PIDControllerX::AtSetpoint() const {
  return m_haveMeasurement && m_haveSetpoint &&
         std::abs(m_positionError) < m_positionTolerance &&
         std::abs(m_velocityError) < m_velocityTolerance;
}

void PIDControllerX::EnableContinuousInput(double minimumInput,
                                          double maximumInput) {
  m_continuous = true;
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
}

void PIDControllerX::DisableContinuousInput() {
  m_continuous = false;
}

bool PIDControllerX::IsContinuousInputEnabled() const {
  return m_continuous;
}

void PIDControllerX::SetIntegratorRange(double minimumIntegral,
                                       double maximumIntegral) {
  m_minimumIntegral = minimumIntegral;
  m_maximumIntegral = maximumIntegral;
}

void PIDControllerX::SetTolerance(double positionTolerance,
                                 double velocityTolerance) {
  m_positionTolerance = positionTolerance;
  m_velocityTolerance = velocityTolerance;
}

double PIDControllerX::GetPositionError() const {
  return m_positionError;
}

double PIDControllerX::GetVelocityError() const {
  return m_velocityError;
}

double CalcWrap(double pos)
{
  double ret = pos;
  ret -= 360. * std::floor((ret + 180.) * (1. / 360.)); 
  return ret;
}

double PIDControllerX::Calculate(double measurement) {
  m_measurement = measurement;
  m_prevError = m_positionError;
  m_haveMeasurement = true;

  if (m_continuous) {
    //double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    //m_positionError = frc::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    m_positionError = CalcWrap(m_setpoint - m_measurement);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period.value();

  if (m_Ki != 0) {
    m_totalError =
        std::clamp(m_totalError + m_positionError * m_period.value(),
                   m_minimumIntegral / m_Ki, m_maximumIntegral / m_Ki);
  }

  return m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
}

double PIDControllerX::Calculate(double measurement, double setpoint) {
  m_setpoint = setpoint;
  m_haveSetpoint = true;
  return Calculate(measurement);
}

void PIDControllerX::Reset() {
  m_positionError = 0;
  m_prevError = 0;
  m_totalError = 0;
  m_velocityError = 0;
  m_haveMeasurement = false;
}

void PIDControllerX::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("PIDController");
  builder.AddDoubleProperty(
      "p", [this] { return GetP(); }, [this](double value) { SetP(value); });
  builder.AddDoubleProperty(
      "i", [this] { return GetI(); }, [this](double value) { SetI(value); });
  builder.AddDoubleProperty(
      "d", [this] { return GetD(); }, [this](double value) { SetD(value); });
  builder.AddDoubleProperty(
      "setpoint", [this] { return GetSetpoint(); },
      [this](double value) { SetSetpoint(value); });
}