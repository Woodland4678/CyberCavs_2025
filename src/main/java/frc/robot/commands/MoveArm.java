// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  int prevTargetId = 0;
  boolean isDone = false;
  boolean isArmAtRest = false;
  boolean isElevatorAtRest = false;
  boolean forceMove = false;
  Debouncer armevatorAtRest = new Debouncer(0.2); //arm must be at rest for 0.3 seconds before moving on
  public MoveArm(ArmPosition targetPosition, Armevator S_Armevator, CommandSwerveDrivetrain S_Swerve) {
    this.S_Armevator = S_Armevator;
    this.targetPosition = targetPosition;
    this.S_Swerve = S_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(S_Armevator);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    isArmAtRest = false;
    isElevatorAtRest = false;
    armevatorAtRest.calculate(false);
    if ((prevTargetId == targetPosition.positionID) || targetPosition.positionID == 1 || targetPosition.positionID == 3 || targetPosition.positionID == 2 || S_Armevator.getElevatorPosition() > targetPosition.elevatorTarget){ 
      forceMove = true;
    }
    else {
      forceMove = false;
    }
    prevTargetId = targetPosition.positionID;
    currentArmPositionID = S_Armevator.getCurrentArmPositionID();
    S_Armevator.setTargetArmPositionID(targetPosition.positionID);
    if ((currentArmPositionID == 1 && targetPosition.positionID > 1) || (currentArmPositionID != 1 && targetPosition.positionID == 1)) { //if we're moving to intake coral and we're not already in rest position, move to rest first
      S_Armevator.moveArmToPosition(Constants.ArmConstants.restPosition.armTargetAngle);
      S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);
      S_Armevator.moveWristToPosition(Constants.ArmConstants.restPosition.wristTarget);
      moveState = 0;
    }
    else {
      moveState = 1;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(moveState) {
      case 0:
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
      case 1:
          S_Armevator.moveArmToPosition(targetPosition.armTargetAngle);
          S_Armevator.moveWristToPosition(targetPosition.wristTarget);
          if (S_Swerve.getIsAutoAligning() || forceMove) {
            S_Armevator.moveElevatorToPosition(targetPosition.elevatorTarget);
            moveState++;
          }
      break;
      case 2:
          if (S_Armevator.getArmPositionError() < 0.003
            && S_Armevator.getElevatorPositionError() < 0.05
            && S_Armevator.getWristPositionError() < 0.05) {
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
