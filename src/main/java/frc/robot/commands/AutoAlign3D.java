// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign3D extends Command {
  /** Creates a new AutoAlign3D. */
  CommandSwerveDrivetrain S_Swerve;
  Armevator S_Armevator;
  double estimatedDistanceX = 0;
  double estimatedDistanceY = 0;
  char branch;
  int state = 0;
  boolean isDone = false;
  double rControllerSetpoint = 0;
  double xSpeed, ySpeed, rSpeed;
  Integer[] branchValues;
  CommandXboxController joystick;
   PhoenixPIDController xController = new PhoenixPIDController(3.0, 0, 0.1); //for m
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); 
  PhoenixPIDController yController = new PhoenixPIDController(3.4, 0, 0.0);
  PhoenixPIDController rController = new PhoenixPIDController(3, 0, 0.0);
  double xControllerSetpoint, yControllerSetpoint;
  Debouncer AutoAlignDone = new Debouncer(0.1);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final SwerveRequest.ApplyRobotSpeeds m_driveRequestAutoAlign = new SwerveRequest.ApplyRobotSpeeds()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
   private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
 
 
   public AutoAlign3D(CommandSwerveDrivetrain S_Swerve, Armevator S_Armevator, char branch, CommandXboxController joystick) {
    this.S_Swerve = S_Swerve;
    this.S_Armevator = S_Armevator;
    this.branch = branch;
    this.joystick = joystick;
    branchValues = Constants.SwerveConstants.aprilTagRotationValues.get(branch);
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xSpeed = 0;
    ySpeed = 0;
    rSpeed = 0;   
    state = 0;
    isDone = false;
    Optional<Alliance> ally = DriverStation.getAlliance();
    /******************* */
    //*X & Y controller setups */
    //******************* */
    
    if (branch == 'A' || branch == 'C' || branch == 'E' || branch == 'G' || branch == 'I' || branch == 'K') {
      xControllerSetpoint = (-0.16);
    } else {
      xControllerSetpoint = (0.2); 
    }
    xController.setTolerance(0.03);    
    yControllerSetpoint = 1.05;
    yController.setTolerance(0.025); //2.5cm
   
    /******************* */
    //*R controller setups */
    //******************* */
    rController.setTolerance(0.0261799); //1.5 degrees in radians
    rController.enableContinuousInput(-Math.PI, Math.PI);
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
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
    }
    else {
      rControllerSetpoint = branchValues[0];
    }
    rControllerSetpoint = Math.toRadians(rControllerSetpoint);
    SmartDashboard.putNumber("Auto align 3D rController setpoint!", rControllerSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var dashPIDS = S_Swerve.getDashPIDS();
    
    // xController.setPID(dashPIDS[0], dashPIDS[1], dashPIDS[2]);
    // yController.setPID(dashPIDS[3], dashPIDS[4], dashPIDS[5]);
     //rController.setPID(dashPIDS[6], dashPIDS[7], dashPIDS[8]);
    //************* */
    //*L4 settings */
    //*********** */
    if (S_Armevator.getArmPosition() > 0.1) {
      yControllerSetpoint = 0.89;
      if (xControllerSetpoint < 0) {
        xControllerSetpoint = -0.17;
      }
      else {
        xControllerSetpoint = 0.2;
      }
      
    }    
    rSpeed = rController.calculate(Math.toRadians(S_Swerve.getgyroValue()), rControllerSetpoint, Timer.getFPGATimestamp());
    if (S_Swerve.getBestAprilTagID() != branchValues[2] || S_Swerve.getDistanceLaser() > 150 || S_Swerve.getDistanceLaser() < 10) {
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
      switch(state) {
        case 0:
          estimatedDistanceX = S_Swerve.bestAprilTagXMeters();
          estimatedDistanceY = S_Swerve.bestAprilTagYMeters();       
          if (Math.abs(xController.getPositionError()) < 0.08) {
            xController.setP(6.5);
           // rController.setP(4);
            xController.setD(0.1);
          }
          else {
            xController.setP(5.9);
            xController.setD(0.1);
           // rController.setP(5.7);
           // rController.setD(0.05);
          }
          double adjustedX = estimatedDistanceY * Math.tan(rController.getPositionError());
          estimatedDistanceX = estimatedDistanceX + adjustedX;
        // SmartDashboard.putNumber("Adjusted April tag distance", estimatedDistanceX);
          xSpeed = xController.calculate(estimatedDistanceX, xControllerSetpoint, Timer.getFPGATimestamp());
          ySpeed = yController.calculate(estimatedDistanceY, yControllerSetpoint, Timer.getFPGATimestamp());
          if (yController.getPositionError() < 0.75 && Math.abs(xController.getPositionError()) < 0.30) { //when this is true the MoveArm command completes the arm movement
            S_Swerve.setIsAutoAligning(true);
          }
          else {
            S_Swerve.setIsAutoAligning(false);
          }
          SmartDashboard.putNumber("auto align coral y-speed", ySpeed);
          //SmartDashboard.putNumber("auto align coral timestamp2", time2.getAllTimestamps().getBestTimestamp().getTime());
         // SmartDashboard.putNumber("auto align coral timestamp", time.getSystemTimestamp().getTime());
          SmartDashboard.putBoolean("auto align coral is x done", xController.atSetpoint());
          SmartDashboard.putBoolean("auto align coral is y done", yController.atSetpoint());
          SmartDashboard.putBoolean("auto align coral is r done", rController.atSetpoint());
          SmartDashboard.putNumber("auto align coral x error", xController.getPositionError());
          SmartDashboard.putNumber("auto align coral y error", yController.getPositionError());
          SmartDashboard.putNumber("auto align coral r error", rController.getPositionError());
        // SmartDashboard.putNumber("auto align coral estimated y distance", estYDist);
          SmartDashboard.putNumber("auto align coral estimated x distance", estimatedDistanceX);
          
          SmartDashboard.putNumber("Auto Align Coral X Speed", xSpeed);
          SmartDashboard.putNumber("Auto Align Coral R Speed", rSpeed);
         
          // S_Swerve.setControl(
          //   m_driveRequestAutoAlign
          //     .withVelocityX(-ySpeed)
          //       .withVelocityY(-xSpeed)
          //       .withRotationalRate(rSpeed)
          //       .withDriveRequestType(DriveRequestType.Velocity)
          // );
          S_Swerve.setControl(
              m_driveRequestAutoAlign.withSpeeds(new ChassisSpeeds(-ySpeed, -xSpeed, rSpeed))
                );
          if(AutoAlignDone.calculate(yController.atSetpoint() && xController.atSetpoint() && rController.atSetpoint() && (S_Armevator.getTargetArmPositionID() == S_Armevator.getCurrentArmPositionID()))) {
            state = 1;
            AutoAlignDone.calculate(false);
          }          
        break;
        case 1:
          S_Armevator.setEndEffectorVoltage(-6);
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
    S_Swerve.setIsAutoAligning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
