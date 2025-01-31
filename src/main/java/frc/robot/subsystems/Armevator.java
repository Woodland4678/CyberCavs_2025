// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Armevator extends SubsystemBase {
  TalonFX elevatorMotor;
  TalonFX armMotor;
  SparkMax wristMotor;
  SparkMax endAffectorWheels;
  private SparkMaxConfig motorConfig;
  private SparkMaxConfig endAffectorConfig;
  private SparkClosedLoopController wristController;
  private SparkClosedLoopController endAffectorController;
  private DigitalInput hasCoral;
  private DigitalInput atStartPos; 
  private final DutyCycleEncoder armAbsolute; // Absoloute Encoder
  private final DutyCycleEncoder wristAbsolute; // Absoloute Encoder
  /** Creates a new Armevator. */
  public Armevator() {
    elevatorMotor = new TalonFX(0,"rio");
    armMotor = new TalonFX(1, "rio");
    wristMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
    wristController = wristMotor.getClosedLoopController();
    endAffectorWheels = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
    endAffectorController = endAffectorWheels.getClosedLoopController();
    hasCoral = new DigitalInput(1);
    armAbsolute = new DutyCycleEncoder(2);
    wristAbsolute = new DutyCycleEncoder(3);
    atStartPos = new DigitalInput(4);
    // in init function
    var elevatorConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var elevatorMotionPIDConfigs = elevatorConfigs.Slot0;
    elevatorMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    elevatorMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    elevatorMotionPIDConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorMotionPIDConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    elevatorMotionPIDConfigs.kI = 0; // no output for integrated error
    elevatorMotionPIDConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var elevatorMotionConfigs = elevatorConfigs.MotionMagic;
    elevatorMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    elevatorMotionConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    elevatorMotionConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    elevatorMotor.getConfigurator().apply(elevatorConfigs);

    // in init function
    var armConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var armMotionPIDConfigs = armConfigs.Slot0;
    armMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    armMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    armMotionPIDConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    armMotionPIDConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    armMotionPIDConfigs.kI = 0; // no output for integrated error
    armMotionPIDConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var armMotionConfigs = armConfigs.MotionMagic;
    armMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    armMotionConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    armMotionConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    armMotor.getConfigurator().apply(armConfigs);

    motorConfig = new SparkMaxConfig();

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.4)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      endAffectorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.4)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      endAffectorWheels.configure(endAffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
       
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    SmartDashboard.putNumber("Arm Absolute", getArmAbsolute());
    SmartDashboard.putNumber("Arm Position", getArmPosition());
    SmartDashboard.putNumber("Wrist Absolute", getWristAbsolute());
    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("End effector wheel speed", getEndAffectorWheelSpeed());
    SmartDashboard.putBoolean("Has Coral", hasCoral());
    SmartDashboard.putBoolean("Elevator at start point", isAtStartPos());
    // Lidar SmartDashboard needs to be added here
  }
  public void moveElevatorToPosition(double pos){
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    elevatorMotor.setControl(m_request.withPosition(pos));
  }

  public void moveArmToPosition(double pos){
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    armMotor.setControl(m_request.withPosition(pos));
  }

  public void moveWristToPosition(double pos){
      wristController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public void moveEndAffectorWheelsToPosition(double pos){
    endAffectorController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
}
  public void setEndAffectorVelocity(double velocity){
    endAffectorController.setReference(velocity,ControlType.kVelocity, ClosedLoopSlot.kSlot0);
}
  public boolean hasCoral() {
    return !hasCoral.get();
  }
  public double getElevatorPosition(){
    return elevatorMotor.getPosition().getValueAsDouble();
  }
  public double getArmAbsolute(){
    return armAbsolute.get();
  }
  public double getArmPosition(){
    return armMotor.getPosition().getValueAsDouble();
  }
  public double getWristAbsolute(){
    return wristAbsolute.get();
  }
  public double getWristPosition(){
    return wristMotor.getEncoder().getPosition();
  }
  public double getEndAffectorWheelSpeed(){
    return endAffectorWheels.get();
  }
  public boolean isAtStartPos(){
    return !atStartPos.get();


  }
  
  

}
