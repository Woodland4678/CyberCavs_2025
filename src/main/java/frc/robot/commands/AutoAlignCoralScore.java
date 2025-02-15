// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import java.security.Timestamp;
import java.security.cert.X509CRL;

import javax.xml.xpath.XPath;

import com.ctre.phoenix6.AllTimestamps;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCoralScore extends Command {
  CommandSwerveDrivetrain S_Swerve;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //PhoenixPIDController xController = new PhoenixPIDController(2, 0, 0.03); //for meters
  PhoenixPIDController xController = new PhoenixPIDController(0.04, 0, 0.001); //for cm
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); //0.32, 0, 0.022 //for using camera pitch for lineup
  PhoenixPIDController yController = new PhoenixPIDController(0.03, 0, 0.001); //0.32, 0, 0.022 //for using lidar to line up
  PhoenixPIDController rController = new PhoenixPIDController(0.12, 0, 0.00);
  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double rSpeed = 0.0;
  int state = 0;
  double[] dashPIDS = new double[11];
  int isAtSetpointCnt;
  double degrees = 0;
  double xControllerSetpoint;
  double yControllerSetpoint;
  double rControllerSetpoint;
 // StatusSignal time2 = new StatusSignal<>(getClass(), null, null);
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
    xControllerSetpoint = (17); //left and right
    xController.setTolerance(3.0);
    //yControllerSetpoint = (9);// forward and back
    yControllerSetpoint = 61;
    yController.setTolerance(3.0); //3cm
    rControllerSetpoint = (180); //rotation
    rController.setTolerance(0.5);
    state = 0;
    isAtSetpointCnt = 0;
    yController.reset();
    xController.reset();
    rController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dashPIDS = S_Swerve.getDashPIDS();

     xController.setPID(dashPIDS[0], dashPIDS[1], dashPIDS[2]);
     yController.setPID(dashPIDS[3], dashPIDS[4], dashPIDS[5]);
     xController.setIZone(dashPIDS[6]);
     yController.setIZone(dashPIDS[7]);
    // rController.setPID(dashPIDS[6], dashPIDS[7], dashPIDS[8]);
    // xController.setIZone(dashPIDS[3]);
    
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
        rSpeed = rController.calculate(degrees, rControllerSetpoint, Timer.getFPGATimestamp());
        //rSpeed = 0;
        //var estYDist = 0.0798/(Math.tan(Math.toRadians(S_Swerve.getAprilTagY()))); //0.0889 is the height diff between the camera and middle of the april tag (3.5 inches)
        var xDist = (S_Swerve.getDistanceLaser()) * Math.tan(Math.toRadians(S_Swerve.getAprilTagX() + rController.getPositionError()));
        xSpeed = xController.calculate(xDist, xControllerSetpoint,Timer.getFPGATimestamp());
        if (xSpeed < 0) { //a little bit of feedfoward
          xSpeed -= 0.07;
        }
        else {
          xSpeed += 0.07;
        }
        if (xSpeed > 4.0) {
          xSpeed = 4.0;
        }
        // if (Math.abs(rController.getPositionError()) > rController.getPositionTolerance()) {
        //   xSpeed = 0;
        // }
        //xSpeed = 0;
        //time2.refresh();
        // if (estYDist > 1.5 && estYDist < 4) {
        //   yController.setP(1.5);
        // }
        // else {
        //   yController.setP(3);
        // }
        ySpeed = yController.calculate(S_Swerve.getDistanceLaser(), yControllerSetpoint, Timer.getFPGATimestamp());
        if(ySpeed < 0) {
          ySpeed -= 0.1;
        }
        else {
          ySpeed += 0.1;
        }
        if (ySpeed < -4) {
          ySpeed = -4.0;
        }
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
        SmartDashboard.putNumber("auto align coral y-speed", ySpeed);
        //SmartDashboard.putNumber("auto align coral timestamp2", time2.getAllTimestamps().getBestTimestamp().getTime());
        SmartDashboard.putNumber("auto align coral timestamp", time.getSystemTimestamp().getTime());
        SmartDashboard.putBoolean("auto align coral is x done", xController.atSetpoint());
        SmartDashboard.putBoolean("auto align coral is y done", yController.atSetpoint());
        SmartDashboard.putBoolean("auto align coral is r done", rController.atSetpoint());
        SmartDashboard.putNumber("auto align coral x error", xController.getPositionError());
        SmartDashboard.putNumber("auto align coral y error", yController.getPositionError());
        SmartDashboard.putNumber("auto align coral r error", rController.getPositionError());
       // SmartDashboard.putNumber("auto align coral estimated y distance", estYDist);
        SmartDashboard.putNumber("auto align coral estimated x distance", xDist);
        
        SmartDashboard.putNumber("Auto Align Coral X Speed", xSpeed);
        SmartDashboard.putNumber("Auto Align Coral R Speed", rSpeed);
        S_Swerve.setControl(
          m_driveRequest.withVelocityX(-ySpeed)
              .withVelocityY(xSpeed)
              .withRotationalRate(rSpeed)
        );
        if (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint()) {
          isAtSetpointCnt++;
          if (isAtSetpointCnt > 10) {
            //state++;
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
