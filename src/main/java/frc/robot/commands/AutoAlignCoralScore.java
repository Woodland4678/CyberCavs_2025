// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  PhoenixPIDController xController = new PhoenixPIDController(0.044, 0, 0.0); //for cm
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); //0.32, 0, 0.022 //for using camera pitch for lineup
  PhoenixPIDController yController = new PhoenixPIDController(0.04, 0, 0.0); //0.32, 0, 0.022 //for using lidar to line up
  PhoenixPIDController rController = new PhoenixPIDController(0.15, 0, 0.00);
  // HolonomicDriveController hController = new HolonomicDriveController(
  //   new PIDController(4, 0, 0),
  //   new PIDController(4, 0, 0),
  //   new ProfiledPIDController(0.1, 0, 0, new Constraints(3, 4)));
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
  boolean isDone = false;
  Integer[] branchValues;
  CommandXboxController joystick;
  Armevator S_Armevator;
  boolean isSwitchedPID = false;
  Debouncer AutoAlignDone = new Debouncer(0.2);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
 // StatusSignal time2 = new StatusSignal<>(getClass(), null, null);
  AllTimestamps time = new AllTimestamps();
  private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
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
    rController.enableContinuousInput(-180, 180);
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isSwitchedPID = false;
    Alliance ally = DriverStation.getAlliance().get();    
    isDone = false;
    AutoAlignDone.calculate(false);
    if (branch == 'A' || branch == 'C' || branch == 'E' || branch == 'G' || branch == 'I' || branch == 'K') {
      xControllerSetpoint = (0.24);
    } else {
      xControllerSetpoint = (-0.122); //left and right: +22 for left -9 for right
    }
    xController.setTolerance(0.02);
    //yControllerSetpoint = (9);// forward and back
    yControllerSetpoint = 1.158;
    yController.setTolerance(0.030); //3cm
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
    
    rController.setTolerance(1.5);
    state = 0;
    isAtSetpointCnt = 0;
    yController.reset();
    xController.reset();
   // rController.reset();
    S_Swerve.setAprilTagTargetRequest(branchValues[2]);
    S_Swerve.setIsAutoAligning(false);
    // if (S_Armevator.getArmPosition() > 0.1) {
    //   yControllerSetpoint = 0.758;
    //   if (xControllerSetpoint < 0) {
    //     xControllerSetpoint = -0.174;
    //   }
    //   else {
    //     xControllerSetpoint = 0.2;
    //   }
      
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Armevator.getArmPosition() > 0.1) {
      yControllerSetpoint = 0.862; //82.2 for lidar
      if (xControllerSetpoint < 0) {
        xControllerSetpoint = -0.144; //-12.4 for lidar
      }
      else {
        xControllerSetpoint = 0.181; //17.5 for liar
      }
      
    }
    dashPIDS = S_Swerve.getDashPIDS();

    // xController.setPID(dashPIDS[0], dashPIDS[1], dashPIDS[2]);
    // yController.setPID(dashPIDS[3], dashPIDS[4], dashPIDS[5]);
     degrees = S_Swerve.getgyroValue();
      // if (rController.getSetpoint() > 160 && degrees < 0) {
      //   degrees = 360 + degrees;
      // }
      // else if (rController.getSetpoint() < -160 && degrees > 0) {
      //   degrees = degrees - 360;
      // }
      rSpeed = rController.calculate(degrees, rControllerSetpoint, Timer.getFPGATimestamp());
    // xController.setIZone(dashPIDS[6]);
    // yController.setIZone(dashPIDS[7]);
    // rController.setPID(dashPIDS[6], dashPIDS[7], dashPIDS[8]);
    // xController.setIZone(dashPIDS[3]);
    if (S_Swerve.getBestAprilTagID() != branchValues[2] || S_Swerve.getDistanceLaser() > 250 || S_Swerve.getDistanceLaser() < 10) {
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
          if (Math.abs(yController.getPositionError()) < 0.90 && Math.abs(xController.getPositionError()) < 0.30) { //TODO tune for robot
            S_Swerve.setIsAutoAligning(true);
          }
          else {
            S_Swerve.setIsAutoAligning(false);
          }
          if (Math.abs(xController.getPositionError()) < 0.07) {
            xController.setP(3.6);
            rController.setP(0.08);
            yController.setP(2.6);
            
          }
          else {
            xController.setP(2.6);
            rController.setP(0.08);
            yController.setP(2.0);
          }
          //rSpeed = 0;
         // var estYDist = 4.422/(Math.tan(Math.toRadians(S_Swerve.getAprilTagY() + 15.87))); //4.422 is height difference from middle of the camera to middle of the april tag, 15.87 is the angle of the camera
          var xDist = (S_Swerve.getDistanceLaser()/100) * Math.tan(Math.toRadians((S_Swerve.getAprilTagX() * 1.038) + rController.getPositionError()));
         // xDist /= 100;
          //var xDist = (estYDist) * Math.tan(Math.toRadians(S_Swerve.getAprilTagX() + rController.getPositionError()));
          // if (S_Armevator.getTargetArmPositionID() == 6) {
          //   xSpeed = xSpeedLimit.calculate(xController.calculate(xDist, xControllerSetpoint,Timer.getFPGATimestamp()));
          //   ySpeed = ySpeedLimit.calculate(yController.calculate(S_Swerve.getDistanceLaser(), yControllerSetpoint, Timer.getFPGATimestamp()));
          // }
         // else {
        // var speeds = hController.calculate(new Pose2d(0,0, new Rotation2d(S_Swerve.getgyroValue())), new Pose2d(xControllerSetpoint - xDist, yControllerSetpoint - (S_Swerve.getDistanceLaser()/100), new Rotation2d(rControllerSetpoint)),3.0, new Rotation2d(rControllerSetpoint));
         xSpeed = xController.calculate(xDist, xControllerSetpoint,Timer.getFPGATimestamp());
         ySpeed = yController.calculate(S_Swerve.getDistanceLaser()/100, yControllerSetpoint, Timer.getFPGATimestamp());
          //}
          if (xSpeed < 0) { //a little bit of feedfoward
            xSpeed -= 0.05;
          }
          else {
            xSpeed += 0.05;
          }
          // if (xSpeed > 2.5) {
          //   xSpeed = 2.5;
          // }   
          // else if (xSpeed < -2.5){
          //   xSpeed = -2.5;
          // }       
          
          if(ySpeed < 0) {
            ySpeed -= 0.05;
          }
          else {
            ySpeed += 0.05;
          }
          // if (ySpeed < -2.5) {
          //   ySpeed = -2.5;
          // }
          // else if (ySpeed > 2.5) {
          //   ySpeed = 2.5;
          // }
          if (xController.atSetpoint()) {
            //xSpeed = 0;
          }
          if (yController.atSetpoint()) {
            //ySpeed = 0;
          }
          if (rController.atSetpoint()) {
           // rSpeed = rSpeed/2;
          }          
          SmartDashboard.putNumber("auto align coral y-speed", ySpeed);
          //SmartDashboard.putNumber("auto align coral timestamp2", time2.getAllTimestamps().getBestTimestamp().getTime());
          SmartDashboard.putNumber("auto align coral timestamp", time.getSystemTimestamp().getTime());
          SmartDashboard.putBoolean("auto align coral is x done", xController.atSetpoint());
          SmartDashboard.putBoolean("auto align coral is y done", yController.atSetpoint());
          SmartDashboard.putBoolean("auto align coral is r done", rController.atSetpoint());
          SmartDashboard.putNumber("auto align coral x error", xController.getPositionError());
          SmartDashboard.putNumber("Auto align coral yaw + r error", S_Swerve.getAprilTagX() + rController.getPositionError());
          SmartDashboard.putNumber("auto align coral y error", yController.getPositionError());
          SmartDashboard.putNumber("auto align coral r error", rController.getPositionError());
        // SmartDashboard.putNumber("auto align coral estimated y distance", estYDist);
          SmartDashboard.putNumber("auto align coral estimated x distance", xDist);
          SmartDashboard.putNumber("auto align y setpoint", yControllerSetpoint);
          SmartDashboard.putNumber("Auto align x setpoint", xControllerSetpoint);
          
          SmartDashboard.putNumber("Auto Align Coral X Speed", xSpeed);
          SmartDashboard.putNumber("Auto Align Coral R Speed", rSpeed);
          S_Swerve.setControl(
            m_driveRequestAutoAlign.withVelocityX(-ySpeed)
                .withVelocityY(xSpeed)
                .withRotationalRate(rSpeed)
          );
          if(AutoAlignDone.calculate(yController.atSetpoint() && xController.atSetpoint() && (S_Armevator.getTargetArmPositionID() == S_Armevator.getCurrentArmPositionID()))) {
            state = 1;
            AutoAlignDone.calculate(false);
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
          S_Swerve.setControl(
            m_driveRequestAutoAlign.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
          );     
          S_Armevator.setEndEffectorVoltage(7);
          if (AutoAlignDone.calculate(!S_Armevator.hasCoral())); {
            isDone = true;
          } 
        break;
      }
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
