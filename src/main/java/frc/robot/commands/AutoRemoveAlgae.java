// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRemoveAlgae extends Command {
  CommandSwerveDrivetrain S_Swerve;
  Armevator S_Armevator;
  PhoenixPIDController xController = new PhoenixPIDController(0.06, 0, 0.002); //for cm
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); //0.32, 0, 0.022 //for using camera pitch for lineup
  PhoenixPIDController yController = new PhoenixPIDController(0.06, 0, 0.002); //0.32, 0, 0.022 //for using lidar to line up
  PhoenixPIDController rController = new PhoenixPIDController(0.13, 0, 0.00);
  double yControllerSetpoint = 0;
  double xControllerSetpoint = 0;
  double rControllerSetpoint = 0;
  int hasHighAlgae = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  double rSpeed = 0;
  boolean isDone = false;
  int state = 0;
  private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  /** Creates a new AutoRemoveAlgae. */
  public AutoRemoveAlgae(CommandSwerveDrivetrain S_Swerve, Armevator S_Armevator) {
    this.S_Armevator = S_Armevator;
    this.S_Swerve = S_Swerve;
    addRequirements(this.S_Swerve, this.S_Armevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    isDone = false;
    yControllerSetpoint = 98;
    yController.setTolerance(4);
    xController.setTolerance(8);
    rController.setTolerance(5);
    rController.enableContinuousInput(-180, 180);
    xControllerSetpoint = -5.0;
    if (Constants.SwerveConstants.aprilTagAlgaeData.containsKey(S_Swerve.getBestAprilTagID())) {
      rControllerSetpoint = Constants.SwerveConstants.aprilTagAlgaeData.get(S_Swerve.getBestAprilTagID())[0];
      hasHighAlgae = Constants.SwerveConstants.aprilTagAlgaeData.get(S_Swerve.getBestAprilTagID())[1];
      if (hasHighAlgae == 0) {
        isDone = true;
      }
    }
    else {
      isDone = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rSpeed = rController.calculate(S_Swerve.getgyroValue(), rControllerSetpoint, Timer.getFPGATimestamp());
    var xDist = (S_Swerve.getDistanceLaser()) * Math.tan(Math.toRadians(S_Swerve.getAprilTagX() + rController.getPositionError()));
    ySpeed = yController.calculate(S_Swerve.getDistanceLaser(), yControllerSetpoint, Timer.getFPGATimestamp());
    xSpeed = xController.calculate(xDist, xControllerSetpoint, Timer.getFPGATimestamp());
    S_Swerve.setControl(
        m_driveRequestAutoAlign.withVelocityX(-ySpeed)
            .withVelocityY(xSpeed)
            .withRotationalRate(rSpeed)
      );
    switch(state) {
      case 0:        
        if (yController.atSetpoint() && xController.atSetpoint() && rController.atSetpoint()) {
          state++;
        }
      break;
      case 1:
        if (S_Armevator.canArmMove()) {
          S_Armevator.moveArmToPosition(Constants.ArmConstants.highAlgaeRemoval.armTargetAngle);
          S_Armevator.moveWristToPosition(Constants.ArmConstants.highAlgaeRemoval.wristTarget);
          if (S_Armevator.getArmPositionError() < 0.003) {
            isDone = true;
            S_Armevator.setCurrentArmPositionID(Constants.ArmConstants.highAlgaeRemoval.positionID);
          }
        }
        
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (S_Armevator.getArmPosition() > 0.0) {
      S_Armevator.setCurrentArmPositionID(Constants.ArmConstants.highAlgaeRemoval.positionID);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
