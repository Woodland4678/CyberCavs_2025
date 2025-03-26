// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArm extends Command {
  /** Creates a new MoveArm. */
  Armevator S_Armevator;
  CommandSwerveDrivetrain S_Swerve;
  ArmPosition targetPosition;
  int currentArmPositionID;
  int moveState = 0;
  int prevTargetId;
  boolean isDone = false;
  boolean isArmAtRest = false;
  boolean isElevatorAtRest = false;
  boolean forceElevatorMove = false;
  boolean forceMove = false;
  boolean moveToRestFirst = true;
  boolean isAuto = false;
  double currentArmPositionTarget = -0.25;
  Debouncer coralGone = new Debouncer(0.1);
  Debouncer endEffectorWheelsOn = new Debouncer(0.1);
  //int pressedCount = 0;
  Debouncer armevatorAtRest = new Debouncer(0.08); //arm must be at rest for 0.1 seconds before moving on
  public MoveArm(ArmPosition targetPosition, Armevator S_Armevator, CommandSwerveDrivetrain S_Swerve, boolean forceElevatorMove, boolean forceMove) {
    this.S_Armevator = S_Armevator;
    this.targetPosition = targetPosition;
    this.S_Swerve = S_Swerve;
    this.forceElevatorMove = forceElevatorMove;
    this.forceMove = forceMove;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(S_Armevator);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAuto = DriverStation.isAutonomous();
    moveState = 0;
    //pressedCount++;
    isDone = false;
    isArmAtRest = false;
    isElevatorAtRest = false;
    currentArmPositionTarget = -0.25;
    //forceMove = false;
    armevatorAtRest.calculate(false);     
    currentArmPositionID = S_Armevator.getCurrentArmPositionID();
    S_Armevator.setTargetArmPositionID(targetPosition.positionID);
    if ((currentArmPositionID == 1 && targetPosition.positionID > 1) ) { //if we're moving to intake coral and we're not already in rest position, move to rest first //|| (currentArmPositionID != 1 && targetPosition.positionID == 1)
      //S_Armevator.moveArmToPosition(Constants.ArmConstants.restPosition.armTargetAngle);
      //S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);
      //S_Armevator.moveWristToPosition(Constants.ArmConstants.restPosition.wristTarget);
      //moveState = 0;
      moveToRestFirst = true;
    }
    else {
      moveToRestFirst = false;
    }
    if (targetPosition.positionID == 1) {
      if (S_Armevator.hasCoral()) {
        isDone = true;
        moveState = -1;
      }
    }   
    if (forceMove) {
      forceElevatorMove = true;
      if (moveToRestFirst) {
        moveState = 1;
      }
      else {
        moveState = 3;
      }
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Move arm state", moveState);
    SmartDashboard.putNumber("Move arm target ID", targetPosition.positionID);
    switch(moveState) {
      case 0:
        if (((S_Swerve.getDistanceLaser() > 90 || S_Swerve.getDistanceLaser() < 60) || (S_Armevator.getCurrentArmPositionID() == 2 && this.targetPosition.positionID == 1) || (isAuto)) && S_Armevator.canArmMove() && (S_Armevator.hasCoral() || this.targetPosition.positionID == 1 || this.targetPosition.positionID == 2)) {
          if (moveToRestFirst) {
            moveState++;
          }
          else {
            moveState = 3;
          }          
        }
      break;
      case 1:
        if (targetPosition.positionID == 6 || targetPosition.positionID == 2) { //if we're going to L4 swing the arm out right away
          currentArmPositionTarget = targetPosition.armTargetAngle;
          S_Armevator.moveArmToPosition(currentArmPositionTarget);
          S_Armevator.moveWristToPosition(targetPosition.wristTarget);
        }
        else {
          currentArmPositionTarget = Constants.ArmConstants.restPosition.armTargetAngle;
          S_Armevator.moveArmToPosition(currentArmPositionTarget);
          S_Armevator.moveWristToPosition(Constants.ArmConstants.restPosition.wristTarget);
        }
        S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);
        
        moveState++;
      break;
      case 2:
        double armTolerance = 0.008;
        if (targetPosition.positionID == 6) {
          armTolerance = 0.027;
        }
        if (Math.abs(S_Armevator.getArmPosition() - currentArmPositionTarget) < armTolerance) {
          isArmAtRest = true;
        }
        else {
          isArmAtRest = false;
        }
        if (Math.abs(S_Armevator.getElevatorPosition() - Constants.ArmConstants.restPosition.elevatorTarget) < 1.0) {
          isElevatorAtRest = true;
        }
        else {
          isElevatorAtRest = false;
        }
        SmartDashboard.putBoolean("IS arm at rest", isArmAtRest);
        SmartDashboard.putBoolean("Is elevator at rest", isElevatorAtRest);
        SmartDashboard.putBoolean("Can arm move", S_Armevator.canArmMove());
        //isElevatorAtRest = S_Armevator.getElevatorPositionError() < 0.05;        
        if (armevatorAtRest.calculate(isArmAtRest && isElevatorAtRest && S_Armevator.canArmMove())) { //both arm and elevator in position for defined amount of time, then we move on
            moveState++;
            S_Armevator.setCurrentArmPositionID(Constants.ArmConstants.restPosition.positionID);
          }
      break;
      case 3:
          S_Armevator.moveWristToPosition(targetPosition.wristTarget); 
          if (targetPosition.positionID == 6) {
            if (S_Swerve.getIsAutoAligning() || forceElevatorMove) {
              S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
              moveState++;
            }
            S_Armevator.moveArmToPosition(targetPosition.armTargetAngle);                      
          }
          else if (targetPosition.positionID == 4) {
            S_Armevator.moveElevatorToPosition(Constants.ArmConstants.L2Position.elevatorTarget - 10);          
            if (S_Swerve.getIsAutoAligning() || forceElevatorMove) {
              S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
              S_Armevator.moveArmToPosition(targetPosition.armTargetAngle);             
              moveState++;
            }            
          }
          else {
            if (S_Swerve.getIsAutoAligning() || forceElevatorMove) {
              S_Armevator.moveArmToPosition(targetPosition.armTargetAngle);              
              moveState++;
            }            
            S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
          }
          
          
      break;
      case 4:
          SmartDashboard.putNumber("Move Arm Elevator Error", Math.abs(S_Armevator.getElevatorPosition() - targetPosition.elevatorTarget));
          if (S_Armevator.getArmPositionError() < 0.003 //arm moving up is postitive error, so we don't really need to absolute it here
            && (Math.abs(S_Armevator.getElevatorPosition() - targetPosition.elevatorTarget) < 0.75)
            && (S_Armevator.getWristPositionError()) < 0.01) {
              S_Armevator.setCurrentArmPositionID(targetPosition.positionID);
              if (targetPosition.positionID != 6) {
                isDone = true;
              }
              else {
                moveState++;
                coralGone.calculate(false);
                endEffectorWheelsOn.calculate(false);
              }
              
          }
      break;
      case 5:
          if (coralGone.calculate(!S_Armevator.hasCoral()) ||  endEffectorWheelsOn.calculate(Math.abs(S_Armevator.getEndAffectorWheelSpeed()) > 100)) { 
            S_Armevator.moveArmToPosition(S_Armevator.getArmPosition() + 0.04);
            isDone = true;
            moveState++; //just make sure we're not here anymore
          }
      break;
      case 6:
          isDone = true;
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    moveState = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}