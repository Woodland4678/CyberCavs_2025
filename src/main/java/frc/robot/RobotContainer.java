// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlignCoralScore;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.035)
        .withDriveRequestType(DriveRequestType.Velocity);
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber S_Climber = new Climber();
    public final Armevator S_Armevator = new Armevator();
    public final AlgaeManipulator S_AlgaeManipulator = new AlgaeManipulator();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {        
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        //autoChooser = null;
       
    }

    private void configureBindings() {
        /*
         * Driver button 1 (X) is for deploying the climber.
         * Driver button 2 (A) is for moving the algae manipulator to the intake position and starting the intake wheels.
         * Driver button 3 (B) is for moving the algae manipulator to the rest position.
         * Driver button 4 (Y) is for completing the climb.
         * Driver button 5 (LB) is for rotating the robot for the left human player station.
         * Driver button 6 (RB) is for rotating the robot for the right human player station.
         * Driver button 7 (LT) is for scoring algae.
         * Driver button 8 (RT) is for spitting out the coral
         * Driver button 9 (LSB) is for driving the robot.
         * Driver button 10 (RSB) is for steering the robot.
         * Driver button 11 (BACK) is for resetting the gyro.
         * Driver button 12 (START) is for rotating for the climb.
        
         * Operator buttons 1-12 are for moving the robot to the appropriate branch.
         * Operator buttons 13-16 are fo moving the arm to the appropriate level of coral.
         * Operator button 17 is for moving the arm to the rest position.
         * Operator button 18 is for moving the arm to the intake position.
         * Operator button 19 is for intaking the coral.
         * Operator button 20 is for starting and stopping the algae wheels.
         * Operator button 21 is for moving the elevator to the limit switch.
         * Operator button 22 is for calibrating the arm.
         */


         joystick.x().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(0))); // Deploy climber position, tune value later.
         joystick.y().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(10))); // Climb position, tune value later.
         joystick.a().onTrue(new InstantCommand(() -> S_AlgaeManipulator.moveManipulatorToPosition(20))); // Intake position, tune values later.
         joystick.a().onTrue(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(5000)));
         joystick.b().onTrue(new InstantCommand(() -> S_AlgaeManipulator.moveManipulatorToPosition(30))); // Rest postition, tune value later.
         joystick.b().onTrue(new InstantCommand(() -> S_AlgaeManipulator.stopIntakeWheels()));
         joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Armevator.setEndAffectorVelocity(5000))); // Wheel velocity, tune value later.
         joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Armevator.stopEndAffectorWheels()));

        snapDrive.HeadingController = new PhoenixPIDController(10, 0, 0);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> 
            snapDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withTargetDirection(Rotation2d.fromDegrees(45)))
        );

        joystick.leftBumper().whileTrue(
            drivetrain.applyRequest(() -> 
            snapDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withTargetDirection(Rotation2d.fromDegrees(-45)))
        );

        /* joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.rightTrigger().whileTrue(new AutoAlignCoralScore(drivetrain, 'A'));

        operatorController.button(1).whileTrue(new AutoAlignCoralScore(drivetrain, 'A'));
        operatorController.button(2).whileTrue(new AutoAlignCoralScore(drivetrain, 'B'));
        operatorController.button(3).whileTrue(new AutoAlignCoralScore(drivetrain, 'C'));
        operatorController.button(4).whileTrue(new AutoAlignCoralScore(drivetrain, 'D'));
        operatorController.button(5).whileTrue(new AutoAlignCoralScore(drivetrain, 'E'));
        operatorController.button(6).whileTrue(new AutoAlignCoralScore(drivetrain, 'F'));
        operatorController.button(7).whileTrue(new AutoAlignCoralScore(drivetrain, 'G'));
        operatorController.button(8).whileTrue(new AutoAlignCoralScore(drivetrain, 'H'));
        operatorController.button(9).whileTrue(new AutoAlignCoralScore(drivetrain, 'I'));
        operatorController.button(10).whileTrue(new AutoAlignCoralScore(drivetrain, 'J'));
        operatorController.button(11).whileTrue(new AutoAlignCoralScore(drivetrain, 'K'));
        operatorController.button(12).whileTrue(new AutoAlignCoralScore(drivetrain, 'L'));
        //joystick.y().onTrue(new InstantCommand(() -> drivetrain.setSwerveToX()));

        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putNumber("Drivetrain max speed", MaxSpeed);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return Commands.print("No autonomous command configured");
    }
    public void setDashboardPIDs(double P, double I, double D, double Izone, double FF) {
        drivetrain.setDashPIDS(P, I, D, Izone, FF); 
    }
}
