// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Armevator extends SubsystemBase {
  enum CoralStates {
    WAITING_FOR_CORAL,
    POSITION_CORAL_FOR_ARM_MOVE,
    POSITION_CORAL_FOR_SCORE,
    WAITING_FOR_SCORE,
  } 
  CoralStates cState;
  TalonFX elevatorMotor;
  TalonFX armMotor;
  SparkMax wristMotor;
  SparkMax endEffectorMotor;
  private int coralState = 0;
  private SparkMaxConfig wristMotorConfig;
  private SparkMaxConfig endEffectorConfig;
  private SparkClosedLoopController wristController;
  private SparkClosedLoopController endEffectorController;
  private DigitalInput hasCoral;
  private DigitalInput atStartPos; 
  private final DutyCycleEncoder armAbsolute; // Absoloute Encoder
  private final DutyCycleEncoder wristAbsolute; // Absoloute Encoder
  private boolean canArmMove;
  private boolean isArmAtRest;
  private final double coralPositionForArmMove = 0; //TODO find
  private final double coralPositionToScore = 0; //TODO find
  private int currentArmPositionID = 0;
  private int targetArmPositionID = 0;
  private double currentWristTarget = 0;
  /** Creates a new Armevator. */
  public Armevator() {
    cState = CoralStates.WAITING_FOR_CORAL;
    canArmMove = false;
    elevatorMotor = new TalonFX(0,"rio");
    armMotor = new TalonFX(1, "rio");
    wristMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
    wristController = wristMotor.getClosedLoopController();
    endEffectorMotor = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
    endEffectorController = endEffectorMotor.getClosedLoopController();
    hasCoral = new DigitalInput(1);
    armAbsolute = new DutyCycleEncoder(2);
    wristAbsolute = new DutyCycleEncoder(3);
    atStartPos = new DigitalInput(4);
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    //armFeedbackConfigs.SensorToMechanismRatio = 48; // 48:1 on the arm
    armFeedbackConfigs.RotorToSensorRatio = 48; //48:1
    armFeedbackConfigs.FeedbackRotorOffset = 0; //offset for the arm
    // in init function
    var elevatorConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var elevatorMotionPIDConfigs = elevatorConfigs.Slot0; //TODO tune for robot
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
    var armMotionPIDConfigs = armConfigs.Slot0; //TODO tune for robot
    armMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    armMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    armMotionPIDConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output    
    armMotionPIDConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    armMotionPIDConfigs.kI = 0; // no output for integrated error
    armMotionPIDConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    armMotionPIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    armMotionPIDConfigs.kG = 0.0;

    // set Motion Magic settings
    var armMotionConfigs = armConfigs.MotionMagic;
    armMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    armMotionConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    armMotionConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    armConfigs.withFeedback(armFeedbackConfigs);
    armMotor.getConfigurator().apply(armConfigs);

    wristMotorConfig = new SparkMaxConfig();

    wristMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.4) //TODO tune for robot
      .i(0)
      .d(0)
      .outputRange(-1, 1);
   
    EncoderConfig wristFeedbackConfig = new EncoderConfig();
    wristFeedbackConfig.positionConversionFactor(25); //TODO double check ratio here and check how to apply the factor
    wristMotorConfig.apply(wristFeedbackConfig);
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    
  
      endEffectorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.4) //TODO tune for robot
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      endEffectorMotor.configure(endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
       
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


    //State machine for handling coral positioning in the end effector
    switch(cState) {
      case WAITING_FOR_CORAL:
        if (hasCoral()) {
          canArmMove = false;
          cState = CoralStates.POSITION_CORAL_FOR_ARM_MOVE;
          endEffectorMotor.disable();
        }
      break;
      case POSITION_CORAL_FOR_ARM_MOVE:
        if (moveEndAffectorWheelsToPosition(coralPositionForArmMove) < 100) { //TODO tune this for robot
          canArmMove = true;
          moveArmToPosition(Constants.ArmConstants.restPosition.armTargetAngle);
          moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);
          moveWristToPosition(Constants.ArmConstants.restPosition.wristTarget);
        }
        if (currentArmPositionID == 2 && canArmMove) {
          cState = CoralStates.POSITION_CORAL_FOR_SCORE;
          endEffectorMotor.disable();
        }
      break;
      case POSITION_CORAL_FOR_SCORE:
        if (moveEndAffectorWheelsToPosition(coralPositionToScore) < 100){ //TODO tune for robot 
          canArmMove = true;
          cState = CoralStates.WAITING_FOR_SCORE; //move on to another state so we don't keep calling the posiiton control
        }
      break;
      case WAITING_FOR_SCORE:
        //do nothing really
      break;
    }
    if (getTargetArmPositionID() == 1 && !hasCoral()) {
      cState = CoralStates.WAITING_FOR_CORAL;
      setEndAffectorVelocity(8000); //TODO determine correct velocity
    } else if (!hasCoral()) {
      canArmMove = true;
    }
  }
  public void moveElevatorToPosition(double pos){
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    elevatorMotor.setControl(m_request.withPosition(pos));
  }

  public void moveArmToPosition(double pos){
    if (canArmMove()) {
      // create a Motion Magic request, voltage output
      final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

      // set target position to 100 rotations
      armMotor.setControl(m_request.withPosition(pos));
    }
  }

  public void moveWristToPosition(double pos){
    if (canArmMove()) {
      wristController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
      currentWristTarget = pos;
    }
  }
  public double moveEndAffectorWheelsToPosition(double pos){
    endEffectorController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
    return Math.abs(pos - endEffectorMotor.getEncoder().getPosition());
  }
  public void setEndAffectorVelocity(double velocity){
    endEffectorController.setReference(velocity,ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
  public void stopEndAffectorWheels() {
    endEffectorMotor.stopMotor();
  }
  public boolean hasCoral() {
    return !hasCoral.get();
  }
  public double getElevatorPosition(){
    return elevatorMotor.getPosition().getValueAsDouble();
  }
  public double getElevatorPositionError() {
    return elevatorMotor.getClosedLoopError(true).getValueAsDouble();
  }
  public double getArmPositionError() {
    return armMotor.getClosedLoopError(true).getValueAsDouble();
  }
  public double getWristPositionError() {
    return Math.abs(wristMotor.getEncoder().getPosition() - currentWristTarget);
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
    return endEffectorMotor.get();
  }
  public boolean isAtStartPos(){
    return !atStartPos.get();
  }
  public void setCurrentArmPositionID(int ID) {
    currentArmPositionID = ID;
  }
  public int getCurrentArmPositionID() {
    return currentArmPositionID;
  }
  public void setTargetArmPositionID(int ID) {
    targetArmPositionID = ID;
  }
  public int getTargetArmPositionID() {
    return targetArmPositionID;
  }
  public boolean canArmMove() {
    return canArmMove;
  }
  

}
