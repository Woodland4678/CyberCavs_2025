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
   PhoenixPIDController xController = new PhoenixPIDController(5.0, 0, 0.1); //for m
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); 
  PhoenixPIDController yController = new PhoenixPIDController(5.6, 0, 0.2);
  PhoenixPIDController rController = new PhoenixPIDController(7.1, 0, 0.15);
  double xControllerSetpoint, yControllerSetpoint;
  Debouncer AutoAlignDone = new Debouncer(0.1);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
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
      xControllerSetpoint = (0.2);
    } else {
      xControllerSetpoint = (-0.13); 
    }
    xController.setTolerance(0.02);    
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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //************* */
    //*L4 settings */
    //*********** */
    if (S_Armevator.getArmPosition() > 0.1) {
      yControllerSetpoint = 0.87;
      if (xControllerSetpoint < 0) {
        xControllerSetpoint = -0.14;
      }
      else {
        xControllerSetpoint = 0.205;
      }
      
    }    
    rSpeed = rController.calculate(Math.toRadians(S_Swerve.getgyroValue()), rControllerSetpoint, Timer.getFPGATimestamp());
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
      switch(state) {
        case 0:
          estimatedDistanceX = S_Swerve.bestAprilTagXMeters();
          estimatedDistanceY = S_Swerve.bestAprilTagYMeters();       
          if (Math.abs(xController.getPositionError()) < 0.1) {
            xController.setP(2);
            rController.setP(4);
          }
          else {
            xController.setP(5);
            rController.setP(7.1);
          }
          double adjustedX = estimatedDistanceY * Math.tan(rController.getPositionError());
          estimatedDistanceX = estimatedDistanceX + adjustedX;
        // SmartDashboard.putNumber("Adjusted April tag distance", estimatedDistanceX);
          xSpeed = xController.calculate(estimatedDistanceX, xControllerSetpoint, Timer.getFPGATimestamp());
          ySpeed = yController.calculate(estimatedDistanceY, yControllerSetpoint, Timer.getFPGATimestamp());
          if (yController.getPositionError() < 0.7 && Math.abs(xController.getPositionError()) < 0.20) { //when this is true the MoveArm command completes the arm movement
            S_Swerve.setIsAutoAligning(true);
          }
          else {
            S_Swerve.setIsAutoAligning(false);
          }
          S_Swerve.setControl(
            m_driveRequestAutoAlign
              .withVelocityX(-ySpeed)
                .withVelocityY(-xSpeed)
                .withRotationalRate(rSpeed)
                .withDriveRequestType(DriveRequestType.Velocity)
          );
          if(AutoAlignDone.calculate(yController.atSetpoint() && xController.atSetpoint() && rController.atSetpoint() && (S_Armevator.getTargetArmPositionID() == S_Armevator.getCurrentArmPositionID()))) {
            state = 1;
            AutoAlignDone.calculate(false);
          }          
        break;
        case 1:
          S_Armevator.setEndEffectorVoltage(6);
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
