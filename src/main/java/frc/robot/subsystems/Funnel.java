// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  TalonFX funnelMotor;
  /** Creates a new Funnel. */
  public Funnel() {
    funnelMotor = new TalonFX(7,"rio");

    var funnelConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var funnelMotionPIDConfigs = funnelConfigs.Slot0;
    funnelMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    funnelMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    funnelMotionPIDConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    funnelMotionPIDConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    funnelMotionPIDConfigs.kI = 0; // no output for integrated error
    funnelMotionPIDConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var funnelMotionConfigs = funnelConfigs.MotionMagic;
    funnelMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    funnelMotionConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    funnelMotionConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    funnelMotor.getConfigurator().apply(funnelConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void moveFunnelToPosition(double pos){
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    funnelMotor.setControl(m_request.withPosition(pos));
  }
  public void setFunnelVelocity(double velocity){
    // create a Motion Magic request, voltage output
    final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle(0);

    // set target position to 100 rotations
    funnelMotor.setControl(m_request.withVelocity(velocity));
  }
}
