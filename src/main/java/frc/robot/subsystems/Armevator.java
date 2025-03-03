// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
  private int prevArmTarget = 0;
  private SparkMaxConfig wristMotorConfig;
  private SparkMaxConfig endEffectorConfig;
  private SparkClosedLoopController wristController;
  private SparkClosedLoopController endEffectorController;
  private DigitalInput hasCoral;
  //private DigitalInput atStartPos; 
  private final DutyCycleEncoder armAbsolute; // Absoloute Encoder
  private final AnalogInput wristAbsolute; // Absoloute Encoder
  private boolean canArmMove;
  private boolean isArmAtRest;
  private final double coralPositionForArmMove = 5.8; //TODO find
  private final double coralPositionToScore = -5.847; //TODO find
  private double elevatorPositionToMoveArm = -2.1;
  private int currentArmPositionID = 0;
  private int targetArmPositionID = 0;
  private double currentWristTarget = 0;
  private double[] dashPIDS = new double[11];
  private double armVolts = 0;
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
    hasCoral = new DigitalInput(3);
    armAbsolute = new DutyCycleEncoder(6);
    wristAbsolute = new AnalogInput(0);
    //atStartPos = new DigitalInput(4);
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    MotorOutputConfigs armMotorOutputConfigs = new MotorOutputConfigs();
    armFeedbackConfigs.SensorToMechanismRatio = 48; // 48:1 on the arm
   // armFeedbackConfigs.RotorToSensorRatio = 48; //48:1
    //armFeedbackConfigs.FeedbackRotorOffset = 0; //offset for the arm

    armMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    // in init function
    var elevatorConfigs = new TalonFXConfiguration();
    
    elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // set slot 0 gains
    var elevatorMotionPIDConfigs = elevatorConfigs.Slot0; //TODO tune for robot
    elevatorMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    elevatorMotionPIDConfigs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    elevatorMotionPIDConfigs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorMotionPIDConfigs.kP = 40; // A position error of 2.5 rotations results in 12 V output
    elevatorMotionPIDConfigs.kI = 0; // no output for integrated error
    elevatorMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    elevatorMotionPIDConfigs.GravityType = GravityTypeValue.Elevator_Static;
    elevatorMotionPIDConfigs.kG = 0.45;

    // set Motion Magic settings
    var elevatorMotionConfigs = elevatorConfigs.MotionMagic;
    elevatorMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 100 rps
    elevatorMotionConfigs.MotionMagicAcceleration = 300; // Target acceleration of 500 rps/s
    elevatorMotionConfigs.MotionMagicJerk = 1600; // Target jerk of 6000 rps/s/s (0.1 seconds)


    elevatorMotor.getConfigurator().apply(elevatorConfigs);
    

    // in init function
    var armConfigs = new TalonFXConfiguration();
    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // set slot 0 gains
    var armMotionPIDConfigs = armConfigs.Slot0; //TODO tune for robot
    armMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    armMotionPIDConfigs.kV = 2.1; // A velocity target of 1 rps results in 0.12 V output
    armMotionPIDConfigs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output    
    armMotionPIDConfigs.kP = 80; // A position error of 2.5 rotations results in 12 V output
    armMotionPIDConfigs.kI = 0; // no output for integrated error
    armMotionPIDConfigs.kD = 2.0; // A velocity error of 1 rps results in 0.1 V output
    armMotionPIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    armMotionPIDConfigs.kG = 0.45;

    // set Motion Magic settings
    var armMotionConfigs = armConfigs.MotionMagic;
    armMotionConfigs.MotionMagicCruiseVelocity = 1.65; // Target cruise velocity of 1.66 rps this is in mechanism rotations
    armMotionConfigs.MotionMagicAcceleration = 3.5; // Target acceleration of 4 rps/s (0.5 seconds)
    armMotionConfigs.MotionMagicJerk = 40; // Target jerk of 1600 rps/s/s (0.1 seconds)

    armConfigs.withFeedback(armFeedbackConfigs);
    armConfigs.withMotorOutput(armMotorOutputConfigs);
    armMotor.getConfigurator().apply(armConfigs);
    

    wristMotorConfig = new SparkMaxConfig();

    wristMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(2.5) //TODO tune for robot
      .i(0)
      .d(0)
      .outputRange(-1, 1);
   
    EncoderConfig wristFeedbackConfig = new EncoderConfig();
    wristFeedbackConfig.positionConversionFactor(0.04); //TODO double check ratio here and check how to apply the factor
    wristMotorConfig.apply(wristFeedbackConfig);
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    endEffectorConfig = new SparkMaxConfig();
  
      endEffectorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.1) //TODO tune for robot
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      endEffectorMotor.configure(endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      if (Math.abs(getArmPosition() + 0.25) > 0.2) {
        armMotor.setPosition(Constants.ArmConstants.armHomePosition);
      }
       
  }
  private VoltageOut vOut = new VoltageOut(0.0);
  private final SysIdRoutine armSysIDRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(7),
          Seconds.of(4),
          state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> armMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return armSysIDRoutine.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return armSysIDRoutine.dynamic(direction);
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
    SmartDashboard.putNumber("End Effector Wheel Position", endEffectorMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Has Coral", hasCoral());
    SmartDashboard.putNumber("Applied Arm Voltage", armVolts);
    SmartDashboard.putNumber("Current Arm Position", getCurrentArmPositionID());
    SmartDashboard.putNumber("Arm Position Error", getArmPositionError());
    SmartDashboard.putNumber("Elevator Error", getElevatorPositionError());
    SmartDashboard.putNumber("Arm Target Pos", getTargetArmPositionID());
   // SmartDashboard.putBoolean("Elevator at start point", isAtStartPos());
    // Lidar SmartDashboard needs to be added here
    // if (!hasCoral()) {
    //   setEndEffectorVoltage(-6);
    // }
    // else {
    //   endEffectorMotor.stopMotor();
    // }

    //State machine for handling coral positioning in the end effector
    //SmartDashboard.putString("cState", cState.toString());
    switch(cState) {
      case WAITING_FOR_CORAL:
        //setEndEffectorVoltage(-6);
        if (hasCoral()) {
          canArmMove = false;
          cState = CoralStates.POSITION_CORAL_FOR_ARM_MOVE;
          endEffectorMotor.stopMotor();
          endEffectorMotor.getEncoder().setPosition(0); //resest encoder position
        }
      break;
      case POSITION_CORAL_FOR_ARM_MOVE:
        if (moveEndAffectorWheelsToPosition(coralPositionForArmMove) < 0.5) { //TODO tune this for robot
             canArmMove = true;
            //endEffectorMotor.getEncoder().setPosition(0);
            moveArmToPosition(Constants.ArmConstants.restPosition.armTargetAngle);
            moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);
            moveWristToPosition(Constants.ArmConstants.restPosition.wristTarget);
        }
        if (getArmPosition() > -0.25 && canArmMove) {
          cState = CoralStates.POSITION_CORAL_FOR_SCORE;
          endEffectorMotor.getEncoder().setPosition(0);
          endEffectorMotor.disable();
        }
      break;
      case POSITION_CORAL_FOR_SCORE:
        if (moveEndAffectorWheelsToPosition(coralPositionToScore) < 1){ //TODO tune for robot 
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
      setEndEffectorVoltage(4); //TODO determine correct velocity
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
   // if (canArmMove()) {
      // create a Motion Magic request, voltage output
      final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

      // set target position to 100 rotations
      armMotor.setControl(m_request.withPosition(pos));
   // }
  }

  public void moveWristToPosition(double pos){
    //if (canArmMove()) {
      wristController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
      currentWristTarget = pos;
    //}
  }
  public double moveEndAffectorWheelsToPosition(double pos){
    endEffectorController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
    return Math.abs(pos - endEffectorMotor.getEncoder().getPosition());
  }
  public void setEndEffectorVoltage(double velocity){
    endEffectorController.setReference(velocity,ControlType.kVoltage, ClosedLoopSlot.kSlot0);
  }
  public void stopEndAffectorWheels() {
    endEffectorMotor.stopMotor();
  }
  public void stopArm() {
    armMotor.disable();
  }
  public void stopElevator() {
    elevatorMotor.disable();
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
    return wristAbsolute.getValue();
  }
  public double getWristPosition(){
    return wristMotor.getEncoder().getPosition();
  }
  public double getEndAffectorWheelSpeed(){
    return endEffectorMotor.get();
  }
  // public boolean isAtStartPos(){
  //   return !atStartPos.get();
  // }
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
 public void resetArmToAbsolute() {
  armMotor.setPosition(getArmAbsolute() - 0.672); //put arm straight down, then take the value and subtract 0.25, thats what goes here
 }
 public void increaseArmVoltage() {
  elevatorMotor.setVoltage(armVolts += 0.1);
 }
 public void decreaseArmVoltage() {
  elevatorMotor.setVoltage(armVolts -= 0.1);
 }
 public void setPrevArmTarget(int id) {
  prevArmTarget = id;
 }
 public int getPrevArmTarget() {
  return prevArmTarget;
 }
 public void spitCoral() {
  if (targetArmPositionID == 3) {
    setEndEffectorVoltage(-3.8);
  }
  else {
    setEndEffectorVoltage(-7);
  }
 }

 public boolean isWristReady() {
   if (Math.abs(getWristPosition() - Constants.WristConstants.wristHomePosition) < Constants.WristConstants.wristHomePosTolerance) {
     return true;
   }
   return false;
 }

 public boolean isShoulderReady() {
   if (Math.abs(getArmPosition() - Constants.ArmConstants.armHomePosition) < Constants.ArmConstants.armHomePosTolerance) {
     return true;
   }
   return false;
 }

 public boolean isElevatorReady() {
  if (Math.abs(getElevatorPosition() - Constants.ElevatorConstants.elevatorHomePosition) < Constants.ElevatorConstants.elevatorHomePosTolerance) {
    return true;
  }
  return false;
}
}
