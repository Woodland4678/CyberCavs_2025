// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;
  private DigitalInput atMaxClimb;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(5,"rio");
    atMaxClimb = new DigitalInput(0);

    var climberConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var climberMotionPIDConfigs = climberConfigs.Slot0;
    climberMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    climberMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    climberMotionPIDConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    climberMotionPIDConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    climberMotionPIDConfigs.kI = 0; // no output for integrated error
    climberMotionPIDConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var climberMotionConfigs = climberConfigs.MotionMagic;
    climberMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    climberMotionConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    climberMotionConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    climberMotor.getConfigurator().apply(climberConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean getAtMaxClimb() {
    return !atMaxClimb.get();
  }
  public void moveClimberToPosition(double pos){
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    climberMotor.setControl(m_request.withPosition(pos));
  }
}
