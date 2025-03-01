// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployClimber extends Command {
  Climber S_Climber;
  int state = 0;
  double getFeetOutPosition = 13.0;
  double deployPosition = -50.0;
  boolean isDone = false;
  /** Creates a new DeployClimber. */
  public DeployClimber(Climber S_Climber) {
    this.S_Climber = S_Climber;
    addRequirements(S_Climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    isDone = false;
    S_Climber.moveClimberToPosition(getFeetOutPosition);
    if (S_Climber.getIsLocked()) {
      isDone = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case 0:
        if (Math.abs(S_Climber.getClimberPosition() - getFeetOutPosition) < 0.7){
          state++;
          S_Climber.moveClimberToPosition(deployPosition);
        }
      break;
      case 1:
        isDone = true;
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
