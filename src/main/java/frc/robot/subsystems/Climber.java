// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.IsPROLicensedValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Relay;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;
  private DigitalInput atMaxClimb;
  private double[] dashPIDS = new double[11];
  PowerDistribution PDH;
  boolean isLocked = false;
  Debouncer atMaxClimbTriggered = new Debouncer(0.3);
  Relay lockSolenoid;
  
  /** Creates a new Climber. */
  public Climber(PowerDistribution PDH) {
    climberMotor = new TalonFX(5,"DriveTrain");
    atMaxClimb = new DigitalInput(7);
    isLocked = true;
    this.PDH = PDH;
    this.isLocked = false;
    lockSolenoid = new Relay(3);
    var climberConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var climberMotionPIDConfigs = climberConfigs.Slot0;
    climberMotionPIDConfigs.kS = 0.0; // Add 0.25 V output to overcome static friction
    climberMotionPIDConfigs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    climberMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    climberMotionPIDConfigs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    climberMotionPIDConfigs.kI = 0; // no output for integrated error
    climberMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var climberMotionConfigs = climberConfigs.MotionMagic;
    climberMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    climberMotionConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    climberMotionConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)


    climberMotor.getConfigurator().apply(climberConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
    SmartDashboard.putBoolean("Climber is at max climb", getAtMaxClimb());
    // if (atMaxClimbTriggered.calculate(getAtMaxClimb())) {
    //   if (climberMotor.getMotorVoltage().getValueAsDouble() > 0) { //if we're at max climb don't allow a voltage set > 0 (which is in the pull in direction)
    //     setClimberVoltage(0);
    //   }      
    // }
    // else {
    //   atMaxClimbTriggered.calculate(false);
    // }
    // This method will be called once per scheduler run
  }
  public boolean getAtMaxClimb() {
    return atMaxClimb.get();
  }
  public void moveClimberToPosition(double pos){
    
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    
   // if (!isLocked) {
      climberMotor.setControl(m_request.withPosition(pos));
    //}
  }
  public double getClimberPosition(){

    return climberMotor.getPosition().getValueAsDouble();
  }
  public void setDashPIDS(double P, double I, double D, double P2, double I2, double D2, double P3, double I3, double D3, double Izone, double FF) {
    dashPIDS[0] = P;
    dashPIDS[1] = I;
    dashPIDS[2] = D;
    dashPIDS[3] = P2;
    dashPIDS[4] = I2;
    dashPIDS[5] = D2;
    dashPIDS[6] = P3;
    dashPIDS[7] = I3;
    dashPIDS[8] = D3;
    dashPIDS[9] = Izone;
    dashPIDS[10] = FF;
 }
 public double[] getDashPIDS() {
    return dashPIDS;
 }
 public void lock() {
  lockSolenoid.set(Relay.Value.kOff);
  PDH.setSwitchableChannel(false);
  isLocked = true;
 }
 public void unlock() {
  lockSolenoid.set(Relay.Value.kForward);
  PDH.setSwitchableChannel(true);
  isLocked = false;
 }
 public void setClimberVoltage(double voltage) {
  //if (!isLocked) {
    climberMotor.setVoltage(voltage);
  //}
 }
 public boolean getIsLocked() {
  return isLocked;
 }
 public boolean isClimberReady() {
  return climberMotor.isConnected();
 }
  
}
