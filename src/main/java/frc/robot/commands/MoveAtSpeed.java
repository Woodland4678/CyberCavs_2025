// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAtSpeed extends Command {
  CommandSwerveDrivetrain S_Swerve;
    private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    LinearVelocity test = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    
   
  /** Creates a new MoveAtSpeed. */
  public MoveAtSpeed(CommandSwerveDrivetrain S_Swerve) {
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    S_Swerve.setControl(
            m_driveRequestAutoAlign.withVelocityX(0.5)
                .withVelocityY(0.5)
                .withRotationalRate(0)
                .withDriveRequestType(DriveRequestType.Velocity)
          );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
