// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  double climbFastToPosition = 6;
  int state = 0;
  boolean isDone = false;
  Climber S_Climber;
  Debouncer isAtMaxClimb = new Debouncer(0.1);
  public Climb(Climber S_Climber) {
    this.S_Climber = S_Climber;
    addRequirements(S_Climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    isAtMaxClimb.calculate(false);
    S_Climber.moveClimberToPosition(climbFastToPosition);
    state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("State!!!!!!!!!!!!!!!", state);
    switch (state) {
      case 0:
        if (Math.abs(S_Climber.getClimberPosition() - climbFastToPosition) < 2.3) {
          state++;
          S_Climber.setClimberVoltage(6);
        }
      break;
      case 1:
        if (isAtMaxClimb.calculate(S_Climber.getAtMaxClimb())) {
          S_Climber.setClimberVoltage(0);
          //S_Climber.lock();
          state++;
        }
      break;
      case 2:
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
