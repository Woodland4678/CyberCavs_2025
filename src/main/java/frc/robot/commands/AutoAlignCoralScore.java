// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
  PhoenixPIDController yController = new PhoenixPIDController(0.036, 0, 0.002); //0.32, 0, 0.022 //for using lidar to line up
  PhoenixPIDController rController = new PhoenixPIDController(7.1, 0, 0.15);
  SlewRateLimiter ySpeedLimit = new SlewRateLimiter(5.5);
  SlewRateLimiter xSpeedLimit = new SlewRateLimiter(5.5);
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
  char branch;
  int doneCnt = 0;
  boolean isDone = false;
  Integer[] branchValues;
  CommandXboxController joystick;
  Armevator S_Armevator;
  Debouncer AutoAlignDone = new Debouncer(0.1);
  Debouncer isCoralGone = new Debouncer(5, DebounceType.kFalling);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
 // StatusSignal time2 = new StatusSignal<>(getClass(), null, null);
  AllTimestamps time = new AllTimestamps();
  private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
   private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  /** Creates a new AutoAlignCoralScore. */
  public AutoAlignCoralScore(CommandSwerveDrivetrain S_Swerve, Armevator S_Armevator, char branch, CommandXboxController joystick) {
    this.S_Swerve = S_Swerve;
    this.branch = branch;
    this.S_Armevator = S_Armevator;
    branchValues = Constants.SwerveConstants.aprilTagRotationValues.get(branch);
    this.joystick = joystick;
    rController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    doneCnt = 0;
    isCoralGone.calculate(false);
    Alliance ally = DriverStation.getAlliance().get();    
    isDone = false;
    AutoAlignDone.calculate(false);
    if (branch == 'A' || branch == 'C' || branch == 'E' || branch == 'G' || branch == 'I' || branch == 'K') {
      xControllerSetpoint = (20.5);
    } else {
      xControllerSetpoint = (-11); //left and right: +22 for left -9 for right
    }
    xController.setTolerance(2.5);
    //yControllerSetpoint = (9);// forward and back
    yControllerSetpoint = 105;
    yController.setTolerance(2.5); //3cm
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
    rControllerSetpoint = Math.toRadians(rControllerSetpoint);
    
    rController.setTolerance(Math.toRadians(1.2));
    state = 0;
    isAtSetpointCnt = 0;
    yController.reset();
    xController.reset();
    rController.reset();
    S_Swerve.setAprilTagTargetRequest(branchValues[2]);
    S_Swerve.setIsAutoAligning(false);
    if (S_Armevator.getArmPosition() > 0.1) {
      yControllerSetpoint = 82;
      if (xControllerSetpoint < 0) {
        xControllerSetpoint = -14;
      }
      else {
        xControllerSetpoint = 20.5;
      }
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Armevator.getArmPosition() > 0.1) {
      yControllerSetpoint = 84;
      if (xControllerSetpoint < 0) {
        xControllerSetpoint = -14;
      }
      else {
        xControllerSetpoint = 20.5;
      }
      
    }
   // dashPIDS = S_Swerve.getDashPIDS();

    // xController.setPID(dashPIDS[0], dashPIDS[1], dashPIDS[2]);
     //yController.setPID(dashPIDS[3], dashPIDS[4], dashPIDS[5]);
     degrees = S_Swerve.getgyroValue();
      // if (rController.getSetpoint() > 160 && degrees < 0) {
      //   degrees = 360 + degrees;
      // }
      // else if (rController.getSetpoint() < -160 && degrees > 0) {
      //   degrees = degrees - 360;
      // }
      rSpeed = rController.calculate(Math.toRadians(degrees), rControllerSetpoint, Timer.getFPGATimestamp());
     //xController.setIZone(dashPIDS[6]);
     //yController.setIZone(dashPIDS[7]);
    // rController.setPID(dashPIDS[6], dashPIDS[7], dashPIDS[8]);
    // xController.setIZone(dashPIDS[3]);
    if (S_Swerve.getBestAprilTagID() != branchValues[2] || S_Swerve.getDistanceLaser() > 200 || S_Swerve.getDistanceLaser() < 10) {
      ySpeed = -joystick.getLeftY();
      xSpeed = -joystick.getLeftX();
      if (S_Armevator.getArmPosition() > 0.1) {
        ySpeed *= 0.7;
        xSpeed *= 0.7;
      }
      S_Swerve.setControl(
          m_driveRequestDrive.withVelocityX(ySpeed * MaxSpeed)
              .withVelocityY(xSpeed * MaxSpeed)
              .withRotationalRate(rSpeed)

        );
        state = 0;
    }
    else {
      //rController.setSetpoint(Constants.SwerveConstants.aprilTagRotationValues.get(S_Swerve.getBestAprilTagID())); //update the rcontroller target to the rotation target of the best april tag we see
      switch(state) {        
        case 0:
          if (yController.getPositionError() < 70 && Math.abs(xController.getPositionError()) < 20) { //TODO tune for robot
            S_Swerve.setIsAutoAligning(true);
          }
          else {
            S_Swerve.setIsAutoAligning(false);
          }
          if (Math.abs(xController.getPositionError()) < 10) {
            xController.setP(0.045); 
           // xController.setD(0.05);                       
          }
          else {
            xController.setP(0.04);           
          }
          if (Math.abs(yController.getPositionError()) < 10) {
            yController.setP(0.055);
           // yController.setD(0.3);
          }
          else {
            yController.setP(0.044);
          //  yController.setD(0.1);
          }
          //rSpeed = 0;
          //var estYDist = 0.0798/(Math.tan(Math.toRadians(S_Swerve.getAprilTagY()))); //0.0889 is the height diff between the camera and middle of the april tag (3.5 inches)
          var xDist = (S_Swerve.getDistanceLaser()) * Math.tan(Math.toRadians(S_Swerve.getAprilTagX() + rController.getPositionError()));
          // if (S_Armevator.getTargetArmPositionID() == 6) {
          //   xSpeed = xSpeedLimit.calculate(xController.calculate(xDist, xControllerSetpoint,Timer.getFPGATimestamp()));
          //   ySpeed = ySpeedLimit.calculate(yController.calculate(S_Swerve.getDistanceLaser(), yControllerSetpoint, Timer.getFPGATimestamp()));
          // }
         // else {
          xSpeed = xController.calculate(xDist, xControllerSetpoint,Timer.getFPGATimestamp());
          ySpeed = yController.calculate(S_Swerve.getDistanceLaser(), yControllerSetpoint, Timer.getFPGATimestamp());
          //}
          if (xSpeed < 0) { //a little bit of feedfoward
            xSpeed -= 0.03;
          }
          else {
            xSpeed += 0.03;
          }
          if (xSpeed > 2.5) {
            xSpeed = 2.5;
          }   
          else if (xSpeed < -2.5){
            xSpeed = -2.5;
          }       
          
          if(ySpeed < 0) {
            ySpeed -= 0.1;
          }
          else {
            ySpeed += 0.1;
          }
          if (ySpeed < -2.5) {
            ySpeed = -2.5;
          }
          else if (ySpeed > 2.5) {
            ySpeed = 2.5;
          }
          if (xController.atSetpoint()) {
            //xSpeed = 0;
          }
          if (yController.atSetpoint()) {
           // ySpeed = 0;
          }
          if (rController.atSetpoint()) {
           // rSpeed = 0;
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
            m_driveRequestAutoAlign.withVelocityX(-ySpeed)
                .withVelocityY(xSpeed)
                .withRotationalRate(rSpeed)
                .withDriveRequestType(DriveRequestType.Velocity)
          );
          if(AutoAlignDone.calculate(yController.atSetpoint() && xController.atSetpoint() && rController.atSetpoint() && (S_Armevator.getTargetArmPositionID() == S_Armevator.getCurrentArmPositionID()))) {
            state = 1;
            AutoAlignDone.calculate(false);
            isCoralGone.calculate(false);
          }
          // if (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint()) {
          //   isAtSetpointCnt++;
          //   if (isAtSetpointCnt > 10) {
          //     //state++;
          //   }
          // }
          // else {
          //   isAtSetpointCnt = 0;
          // }
          
          break;
        case 1:         
          S_Armevator.setEndEffectorVoltage(-8);
          if (!S_Armevator.hasCoral()) {
            doneCnt++;
          }
          else {
            doneCnt = 0;
          }
          if (doneCnt > 5) {
            isDone = true;
          }
          // if (isCoralGone.calculate(S_Armevator.hasCoral() == false)); {
          //   isDone = true;
          // } 
        break;
      }
      SmartDashboard.putBoolean("Is auto align done", isDone);
  }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //S_Swerve.setSwerveToX();
    //S_Swerve.stopDrive();
    S_Swerve.setIsAutoAligning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}