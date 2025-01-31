// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Armevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArm extends Command {
  /** Creates a new MoveArm. */
  Armevator S_Armevator;
  double armPosition, wristPosition, elevatorPosition;
  public MoveArm(double armPosition, double wristPosition, double elevatorPosition, Armevator S_Armevator) {
    this.S_Armevator = S_Armevator;
    this.armPosition = armPosition;
    this.wristPosition = wristPosition;
    this.elevatorPosition = elevatorPosition;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(S_Armevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    S_Armevator.moveArmToPosition(armPosition);
    S_Armevator.moveElevatorToPosition(elevatorPosition);
    S_Armevator.moveWristToPosition(wristPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
