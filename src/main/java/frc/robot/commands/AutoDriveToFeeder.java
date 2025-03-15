// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import javax.xml.crypto.dsig.spec.XSLTTransformParameterSpec;
import javax.xml.xpath.XPath;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveToFeeder extends Command {
  CommandSwerveDrivetrain S_Swerve;
  double angleTarget;
  CommandXboxController controller;
  PhoenixPIDController rController = new PhoenixPIDController(0.12, 0, 0);
  PhoenixPIDController yController = new PhoenixPIDController(0.04, 0, 0.002);
  private final SwerveRequest.RobotCentric m_driveRequestRobotCentric = new SwerveRequest.RobotCentric();
  private final SwerveRequest.FieldCentric m_driveRequestFieldCentric = new SwerveRequest.FieldCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  double rControllerTarget;
  double yControllerTarget;
  double rSpeed = 0;
  double ySpeed = 0;
  double angle = 0;
  boolean isDone = false;
  Debouncer coralIncoming = new Debouncer(0.1);
  Armevator S_Armevator;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  /** Creates a new AutoDriveToFeeder. */
  public AutoDriveToFeeder(CommandSwerveDrivetrain S_Swerve, Armevator S_Armevator, double angleTarget, CommandXboxController controller) {
    
    this.S_Swerve = S_Swerve;
    this.angleTarget = angleTarget;
    this.controller = controller;
   
    this.S_Armevator = S_Armevator;
    rController.enableContinuousInput(-180, 180);
   
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //angle = angleTarget;
    Alliance ally = DriverStation.getAlliance().get();
    if (ally == Alliance.Red) {
      if (angleTarget < 0) {
        angle = angleTarget + 180;
      }
      else {
        angle = angleTarget - 180;
      }
    }
    rControllerTarget = angle;
    rController.setTolerance(10);
    yControllerTarget = 9;
    coralIncoming.calculate(false);
    isDone = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    double degrees = S_Swerve.getgyroValue();
    rSpeed = rController.calculate(degrees, rControllerTarget, Timer.getFPGATimestamp());
    if (coralIncoming.calculate((S_Swerve.getChuteLidar() < 40 && S_Swerve.getChuteLidar() > 1) || S_Armevator.hasCoral())) {
      isDone = true;
      ySpeed = 0;
      rSpeed = 0;
    }
    if ((rController.atSetpoint() && S_Swerve.getRearLidar() < 100) || S_Swerve.getRearLidar() < 40) {
      ySpeed = yController.calculate(S_Swerve.getRearLidar(), yControllerTarget, Timer.getFPGATimestamp());
      S_Swerve.setControl(
        m_driveRequestRobotCentric.withVelocityX(ySpeed)
            .withVelocityY(0)
            .withRotationalRate(rSpeed)
      );
    }
    else {
      ySpeed = -controller.getLeftY() * MaxSpeed;
      S_Swerve.setControl(
        m_driveRequestFieldCentric.withVelocityX(ySpeed)
            .withVelocityY(-controller.getLeftX() * MaxSpeed)
            .withRotationalRate(rSpeed)
      );
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
