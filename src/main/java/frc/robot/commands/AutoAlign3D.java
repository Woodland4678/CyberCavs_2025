// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign3D extends Command {
  /** Creates a new AutoAlign3D. */
  CommandSwerveDrivetrain S_Swerve;
  double estimatedDistanceX = 0;
  double estimatedDistanceY = 0;
  char branch;
  double rControllerSetpoint = 0;
  Integer[] branchValues;
   PhoenixPIDController xController = new PhoenixPIDController(5.0, 0, 0.1); //for cm
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); //0.32, 0, 0.022 //for using camera pitch for lineup
  PhoenixPIDController yController = new PhoenixPIDController(5.6, 0, 0.2); //0.32, 0, 0.022 //for using lidar to line up
  PhoenixPIDController rController = new PhoenixPIDController(7.1, 0, 0.15);
  private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
   private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  public AutoAlign3D(CommandSwerveDrivetrain S_Swerve, Armevator S_Armevator, char branch, CommandXboxController joystick) {
    this.S_Swerve = S_Swerve;
    this.branch = branch;
    branchValues = Constants.SwerveConstants.aprilTagRotationValues.get(branch);
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance ally = DriverStation.getAlliance().get();
    
    rController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
     if (ally == Alliance.Red) {
      if (branchValues[0] < 0) {
        rControllerSetpoint = (branchValues[0] + 180); //rotation
      }
      else {
        rControllerSetpoint = branchValues[0] - 180;
      }
    }
    else {
      rControllerSetpoint = branchValues[0];
    }
    SmartDashboard.putNumber("Auto Align 3D r setpoint", rControllerSetpoint);
    rControllerSetpoint = Math.toRadians(rControllerSetpoint);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double targetAngle = -180;
    estimatedDistanceX = S_Swerve.bestAprilTagXMeters();
    estimatedDistanceY = S_Swerve.bestAprilTagYMeters();
    double currentAngle = S_Swerve.getgyroValue();
    if (currentAngle >= 0 && rControllerSetpoint < 0){
      currentAngle = currentAngle - 360;
    }
    if (Math.abs(xController.getPositionError()) < 0.1) {
      xController.setP(2);
    }
    else {
      xController.setP(5);
    }
    double adjustedX = estimatedDistanceY * Math.tan(Math.toRadians(currentAngle) - rControllerSetpoint);
    estimatedDistanceX = estimatedDistanceX + adjustedX;
    SmartDashboard.putNumber("Adjusted April tag distance", estimatedDistanceX);
    double xSpeed = xController.calculate(estimatedDistanceX, -0.2, Timer.getFPGATimestamp());
    double ySpeed = yController.calculate(estimatedDistanceY, 1.24, Timer.getFPGATimestamp());
    double rSpeed = rController.calculate(Math.toRadians(S_Swerve.getgyroValue()), rControllerSetpoint, Timer.getFPGATimestamp());
    S_Swerve.setControl(
      m_driveRequestAutoAlign
        .withVelocityX(-ySpeed)
          .withVelocityY(-xSpeed)
          .withRotationalRate(rSpeed)
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
