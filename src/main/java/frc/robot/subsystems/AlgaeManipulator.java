// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeManipulator extends SubsystemBase {
  SparkMax manipulatorPositionMotor;
  SparkMax intakeMotor;
  int deployState = 0;
  private SparkMaxConfig manipulatorPositionConfig;
  private SparkMaxConfig intakeConfig;
  private SparkClosedLoopController manipulatorPositionController;
  private SparkClosedLoopController intakeController;
  private double[] dashPIDS = new double[11];
  private boolean isDeployed = false;
  private BooleanSupplier intakeStopped = new BooleanSupplier() {
    public boolean getAsBoolean() {
      return intakeMotor.getEncoder().getVelocity() < 5;
    }
  };
  private int intakeStoppedCount = 0;
  Trigger isIntakeStopped = new Trigger(intakeStopped).debounce(0.5);
  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulator() {
    manipulatorPositionMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
    manipulatorPositionController = manipulatorPositionMotor.getClosedLoopController();
    intakeMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
    intakeController = intakeMotor.getClosedLoopController();

     manipulatorPositionConfig = new SparkMaxConfig();

     manipulatorPositionConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      manipulatorPositionMotor.configure(manipulatorPositionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      intakeConfig = new SparkMaxConfig();

      intakeConfig.closedLoop
       .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
       // Set PID values for position control. We don't need to pass a closed
       // loop slot, as it will default to slot 0.
       .p(0.4)
       .i(0)
       .d(0)
       .outputRange(-1, 1);
 
       intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Manipulator Position", getPosition());
    SmartDashboard.putNumber("Algae Manipulator Speed", getSpeed());
    if (isDeployed) {
      switch(deployState) {
        case 0:
          if (intakeMotor.getEncoder().getVelocity() < 3000) {
            intakeStoppedCount++;
          }
          else {
            intakeStoppedCount = 0;
          }
          if (intakeStoppedCount > 15) {
            setIntakeSpeed(0);
            moveIntakeWheelsToPosition(intakeMotor.getEncoder().getPosition());
            deployState ++;
            intakeStoppedCount = 0;
          }
        break;
        case 1:

        break;
      }
    }
    // intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //   .p(dashPIDS[0])
    //   .i(dashPIDS[1])
    //   .d(dashPIDS[2])
    //   .iZone(dashPIDS[3])
    //   .outputRange(dashPIDS[4], dashPIDS[5]);
    // intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    
    // This method will be called once per scheduler run
  }

  public void moveManipulatorToPosition(double pos){
      manipulatorPositionController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public void setIntakeSpeed(double velocity){
    intakeController.setReference(velocity,ControlType.kVoltage, ClosedLoopSlot.kSlot0);

  }
  public void reverseIntakeWheels() {
    if (!isDeployed) {
      moveManipulatorToPosition(1.5);
    }
    setIntakeSpeed(-5);
  }
  public double getPosition(){
    return manipulatorPositionMotor.getEncoder().getPosition();

  }
  public double getSpeed(){
    return intakeMotor.getEncoder().getVelocity();

  }
  public void stopIntakeWheels() {
    intakeMotor.stopMotor();
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
 public void deploy() {
  moveManipulatorToPosition(9.2);
  isDeployed = true;
  setIntakeSpeed(5);
  deployState = 0;
 }
 public void retract() {
  setIntakeSpeed(0);
  moveManipulatorToPosition(0.6);
  isDeployed = false;
 }
 public void moveIntakeWheelsToPosition(double pos){
  intakeController.setReference(pos,ControlType.kPosition, ClosedLoopSlot.kSlot0);
 }
 public void resetEncoder() {
  manipulatorPositionMotor.getEncoder().setPosition(0);
 }
}
