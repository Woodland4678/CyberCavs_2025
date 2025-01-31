// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import java.security.Timestamp;
import java.security.cert.X509CRL;

import com.ctre.phoenix6.AllTimestamps;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCoralScore extends Command {
  CommandSwerveDrivetrain S_Swerve;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  PhoenixPIDController xController = new PhoenixPIDController(0.12, 0, 0);
  PhoenixPIDController yController = new PhoenixPIDController(0.6, 0, 0.03);
  PhoenixPIDController rController = new PhoenixPIDController(0.22, 0, 0.001);
  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double rSpeed = 0.0;
  int state = 0;
  double[] dashPIDS = new double[5];
  int isAtSetpointCnt;
  double degrees = 0;
  double xControllerSetpoint;
  double yControllerSetpoint;
  double rControllerSetpoint;
  AllTimestamps time = new AllTimestamps();
  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  /** Creates a new AutoAlignCoralScore. */
  public AutoAlignCoralScore(CommandSwerveDrivetrain S_Swerve, char branch) {
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xControllerSetpoint = (0); //left and right
    xController.setTolerance(0.6);
    yControllerSetpoint = (-13.34);// forward and back
    yController.setTolerance(0.5);
    rControllerSetpoint = (180); //rotation
    rController.setTolerance(1.0);
    state = 0;
    isAtSetpointCnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dashPIDS = S_Swerve.getDashPIDS();
    xController.setPID(dashPIDS[0], dashPIDS[1], dashPIDS[2]);
    xController.setIZone(dashPIDS[3]);
    
    //rController.setSetpoint(Constants.SwerveConstants.aprilTagRotationValues.get(S_Swerve.getBestAprilTagID())); //update the rcontroller target to the rotation target of the best april tag we see
    switch(state) {
      case 0:
        degrees = S_Swerve.getgyroValue();
        if (rController.getSetpoint() > 160 && degrees < 0) {
          degrees = 360 + degrees;
        }
        else if (rController.getSetpoint() < -160 && degrees > 0) {
          degrees = degrees - 360;
        }
        rSpeed = rController.calculate(degrees, rControllerSetpoint, time.getDeviceTimestamp().getTime());
        //rSpeed = 0;
        
        xSpeed = xController.calculate(S_Swerve.getAprilTagX(), xControllerSetpoint, time.getDeviceTimestamp().getTime());
        if (rController.getPositionError() > 5) {
          xSpeed *= 0.5;
        }
        //xSpeed = 0;
        ySpeed = yController.calculate(S_Swerve.getAprilTagY(), yControllerSetpoint, time.getDeviceTimestamp().getTime());
        if (xController.atSetpoint()) {
          xSpeed = 0;
        }
        if (yController.atSetpoint()) {
          ySpeed = 0;
        }
        if (rController.atSetpoint()) {
          rSpeed = 0;
        }
        if (S_Swerve.hasAprilTagTarget() == false){
          xSpeed = 0;
          ySpeed = 0;
          rSpeed = 0;
        }
        //ySpeed = 0;
        SmartDashboard.putNumber("Auto Align Coral X Speed", xSpeed);
        SmartDashboard.putNumber("Auto Align Coral R Speed", rSpeed);
        S_Swerve.setControl(
          m_driveRequest.withVelocityX(ySpeed)
              .withVelocityY(xSpeed)
              .withRotationalRate(rSpeed)
        );
        if (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint()) {
          isAtSetpointCnt++;
          if (isAtSetpointCnt > 10) {
            state++;
          }
        }
        else {
          isAtSetpointCnt = 0;
        }
        
        break;
      case 1:
        S_Swerve.setControl(brake);
       // S_Swerve.applyRequest(() -> brake);
        break;
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //S_Swerve.setSwerveToX();
    S_Swerve.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
