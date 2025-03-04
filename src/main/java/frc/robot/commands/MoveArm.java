// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  //int pressedCount = 0;
  Debouncer armevatorAtRest = new Debouncer(0.1); //arm must be at rest for 0.1 seconds before moving on
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
    moveState = 0;
    //pressedCount++;
    isDone = false;
    isArmAtRest = false;
    isElevatorAtRest = false;
    //forceMove = false;
    armevatorAtRest.calculate(false);     
    currentArmPositionID = S_Armevator.getCurrentArmPositionID();
    S_Armevator.setTargetArmPositionID(targetPosition.positionID);
    if ((currentArmPositionID == 1 && targetPosition.positionID > 1) || (currentArmPositionID != 1 && targetPosition.positionID == 1)) { //if we're moving to intake coral and we're not already in rest position, move to rest first
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
    
    switch(moveState) {
      case 0:
        if ((S_Swerve.getDistanceLaser() > 90 || S_Swerve.getDistanceLaser() < 60 || (S_Armevator.getCurrentArmPositionID() == 2 && this.targetPosition.positionID == 1 )) && S_Armevator.canArmMove() && (S_Armevator.hasCoral() || this.targetPosition.positionID == 1 || this.targetPosition.positionID == 2)) {
          if (moveToRestFirst) {
            moveState++;
          }
          else {
            moveState = 3;
          }          
        }
      break;
      case 1:
        S_Armevator.moveArmToPosition(Constants.ArmConstants.restPosition.armTargetAngle);
        S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);
        S_Armevator.moveWristToPosition(Constants.ArmConstants.restPosition.wristTarget);
        moveState++;
      break;
      case 2:
        if (S_Armevator.getArmPositionError() < 0.003) {
          isArmAtRest = true;
        }
        else {
          isArmAtRest = false;
        }
        if (S_Armevator.getElevatorPositionError() < 0.05) {
          isElevatorAtRest = true;
        }
        else {
          isElevatorAtRest = false;
        }
        //isElevatorAtRest = S_Armevator.getElevatorPositionError() < 0.05;        
        if (armevatorAtRest.calculate(isArmAtRest && isElevatorAtRest)) { //both arm and elevator in position for defined amount of time, then we move on
            moveState++;
            S_Armevator.setCurrentArmPositionID(Constants.ArmConstants.restPosition.positionID);
          }
      break;
      case 3:
          if (targetPosition.positionID == 6) {
            if (S_Swerve.getIsAutoAligning() || forceElevatorMove) {
              S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
              moveState++;
            }
            S_Armevator.moveArmToPosition(targetPosition.armTargetAngle); 
            S_Armevator.moveWristToPosition(targetPosition.wristTarget);           
          }
          else if (targetPosition.positionID == 4) {
            S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget - 10);
           // S_Armevator.moveWristToPosition(-0.24);
            if (S_Swerve.getIsAutoAligning() || forceElevatorMove) {
              S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
              S_Armevator.moveArmToPosition(targetPosition.armTargetAngle);
              S_Armevator.moveWristToPosition(targetPosition.wristTarget);
              moveState++;
            }
            if (S_Armevator.getElevatorPosition() < -5.9) {
              S_Armevator.moveWristToPosition(targetPosition.wristTarget);
            }
          }
          else {
            if (S_Swerve.getIsAutoAligning() || forceElevatorMove) {
              S_Armevator.moveArmToPosition(targetPosition.armTargetAngle); 
              S_Armevator.moveWristToPosition(targetPosition.wristTarget);
              moveState++;
            }
            if (S_Armevator.getElevatorPosition() < -5.9) {
              S_Armevator.moveWristToPosition(targetPosition.wristTarget);
            }
            S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
          }
          
          
      break;
      case 4:
          if (S_Armevator.getArmPositionError() < 0.003
            && S_Armevator.getElevatorPositionError() < 0.05
            && S_Armevator.getWristPositionError() < 0.015) {
              S_Armevator.setCurrentArmPositionID(targetPosition.positionID);
              isDone = true;
          }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
