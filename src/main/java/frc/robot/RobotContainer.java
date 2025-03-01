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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlignCoralPath;
import frc.robot.commands.AutoAlignCoralScore;
import frc.robot.commands.AutoDriveToFeeder;
import frc.robot.commands.Climb;
import frc.robot.commands.DeployClimber;
import frc.robot.commands.MoveArm;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double v = 0;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.035)
        .withDriveRequestType(DriveRequestType.Velocity);
    
    private LEDStrip ledStrip;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    PowerDistribution PDH = new PowerDistribution();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber S_Climber = new Climber(PDH);
    public final Armevator S_Armevator = new Armevator();
    public final AlgaeManipulator S_AlgaeManipulator = new AlgaeManipulator();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        ledStrip = LEDStrip.getInstance();
        configureBindings();
        NamedCommands.registerCommand("AutoScoreJ", new AutoAlignCoralScore(drivetrain, S_Armevator, 'J', joystick));
        NamedCommands.registerCommand("InitElevator", new InstantCommand(() -> S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget)));
        new EventTrigger("MoveArmToL4").onTrue(new MoveArm(Constants.ArmConstants.L4Position, S_Armevator, drivetrain, false, false));
        new EventTrigger("MoveArmToIntake").onTrue(new MoveArm(Constants.ArmConstants.intakePosition, S_Armevator, drivetrain, true, false));
        NamedCommands.registerCommand("AutoLineupFeeder", new AutoDriveToFeeder(drivetrain, S_Armevator, -54, joystick));
        NamedCommands.registerCommand("AutoScoreK", new AutoAlignCoralScore(drivetrain, S_Armevator, 'K', joystick));
        NamedCommands.registerCommand("AutoScoreL", new AutoAlignCoralScore(drivetrain, S_Armevator, 'L', joystick));
        NamedCommands.registerCommand("AutoScoreA", new AutoAlignCoralScore(drivetrain, S_Armevator, 'A', joystick));
        autoChooser = AutoBuilder.buildAutoChooser("Left4L4");
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


        // joystick.x().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(0))); // Deploy climber position, tune value later.
         //joystick.y().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(10))); // Climb position, tune value later.
         //joystick.a().onTrue(new InstantCommand(() -> S_AlgaeManipulator.moveManipulatorToPosition(20))); // Intake position, tune values later.
         //joystick.a().onTrue(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(5000)));
        // joystick.b().onTrue(new InstantCommand(() -> S_AlgaeManipulator.moveManipulatorToPosition(30))); // Rest postition, tune value later.
         //joystick.b().onTrue(new InstantCommand(() -> S_AlgaeManipulator.stopIntakeWheels()));


        // joystick.pov(0).onTrue(new InstantCommand(() -> S_Armevator.moveArmToPosition(0.00))); //0.25 should be straight up
        // joystick.pov(180).onTrue(new InstantCommand(() -> S_Armevator.moveArmToPosition(-0.25))); //-0.25 should be straight down
         //joystick.pov(270).onTrue(new InstantCommand(() -> S_Armevator.moveArmToPosition(-0.1))); //-0.25 should be straight down
        // joystick.pov(90).onTrue(new InstantCommand(() -> S_Armevator.moveElevatorToPosition(-4.5))); //TODO find this test value
         //joystick.pov(270).onTrue(new InstantCommand(() -> S_Armevator.moveElevatorToPosition(-16))); //TODO find this test value
         //joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Armevator.setEndAffectorVelocity(5000))); // Wheel velocity, tune value later.
         //joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Armevator.stopEndAffectorWheels()));
         //joystick.pov(0).onTrue(new InstantCommand(() -> S_AlgaeManipulator.moveManipulatorToPosition(0))); //0.25 should be straight up
         //joystick.pov(180).onTrue(new InstantCommand(() -> S_AlgaeManipulator.moveManipulatorToPosition(7))); //-0.25 should be straight down
        //  joystick.pov(90).onTrue(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(-4.0))); //TODO find this test value
        //  joystick.pov(270).onTrue(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(4.0))); //TODO find this test value
        //  joystick.pov(90).onFalse(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(0))); //TODO find this test value
        //  joystick.pov(270).onFalse(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(0))); //TODO find this test value
       // joystick.pov(90).onTrue(new InstantCommand(() -> S_Armevator.setEndAffectorVelocity(3)));
        // joystick.pov(270).onTrue(new InstantCommand(() -> S_Armevator.setEndAffectorVelocity(-3)));
        // joystick.pov(90).onFalse(new InstantCommand(() -> S_Armevator.setEndAffectorVelocity(0)));
        // joystick.pov(270).onFalse(new InstantCommand(() -> S_Armevator.setEndAffectorVelocity(0)));
       // joystick.pov(270).onTrue(new InstantCommand(() -> S_Armevator.moveWristToPosition(0.05)));
        //joystick.pov(90).onTrue(new InstantCommand(() -> S_Armevator.moveWristToPosition(0.25)));
       // joystick.pov(270).onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(18.0))); //13 for double bar climb
       // joystick.pov(90).onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(-47.33)));
       // joystick.pov(0).onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(0.06)));
        //joystick.pov(180).onTrue(new InstantCommand(() -> S_Armevator.increaseArmVoltage()));
        //joystick.pov(0).onTrue(new InstantCommand(() -> S_Armevator.decreaseArmVoltage()));
        //joystick.pov(0).onTrue(new MoveArm(Constants.ArmConstants.L4Position, S_Armevator));
        //joystick.pov(90).onTrue(new MoveArm(Constants.ArmConstants.L3Position, S_Armevator));
        //joystick.pov(180).onTrue(new MoveArm(Constants.ArmConstants.L2Position, S_Armevator));
        //joystick.pov(270).onTrue(new MoveArm(Constants.ArmConstants.L1Position, S_Armevator));
        //joystick.pov(0).onTrue(new InstantCommand(() -> S_Armevator.moveElevatorToPosition(-16)));
        //joystick.pov(180).onTrue(new InstantCommand(() -> S_Armevator.moveElevatorToPosition(-2)));
        //joystick.a().onTrue(new MoveArm(Constants.ArmConstants.intakePosition, S_Armevator));
        //joystick.b().onTrue(new MoveArm(Constants.ArmConstants.restPosition, S_Armevator));
        joystick.leftBumper().onTrue(new InstantCommand(() -> S_AlgaeManipulator.deploy()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> S_AlgaeManipulator.retract()));
        //joystick.pov(90).onTrue(new InstantCommand(() -> S_Armevator.moveEndAffectorWheelsToPosition(100)));
        //joystick.pov(90).onTrue(new InstantCommand(() -> S_Armevator.moveEndAffectorWheelsToPosition(0)));
        
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
        // joystick.rightBumper().whileTrue(
        //     drivetrain.applyRequest(() -> 
        //     snapDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        //     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        //     .withTargetDirection(Rotation2d.fromDegrees(53)))
        // );

        // joystick.leftBumper().whileTrue(
        //     drivetrain.applyRequest(() -> 
        //     snapDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        //     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        //     .withTargetDirection(Rotation2d.fromDegrees(-53)))
        // );
        joystick.x().whileTrue(new AutoDriveToFeeder(drivetrain, S_Armevator, -54, joystick)); //left
        joystick.b().whileTrue(new AutoDriveToFeeder(drivetrain, S_Armevator, 54, joystick)); //right

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
        // joystick.back().and(joystick.y()).whileTrue(S_Armevator.sysIDDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(S_Armevator.sysIDDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(S_Armevator.sysIDQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(S_Armevator.sysIDQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //joystick.rightTrigger().whileTrue(new AutoAlignCoralScore(drivetrain, 'A'));
        //joystick.rightTrigger().whileTrue(new AutoAlignCoralPath(drivetrain));
        joystick.rightTrigger().whileTrue(new InstantCommand(() -> S_Armevator.setEndEffectorVoltage(2.8)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Armevator.setEndEffectorVoltage(0)));
       // joystick.rightTrigger().whileTrue(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(-7)));
        //joystick.rightTrigger().onFalse(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(0)));
        joystick.leftTrigger().whileTrue(new InstantCommand(() -> S_AlgaeManipulator.setIntakeSpeed(-5)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> S_AlgaeManipulator.stopIntakeWheels()));
       //joystick.leftTrigger().whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'A',joystick));

        operatorController.button(1).onTrue(new MoveArm(Constants.ArmConstants.L1Position, S_Armevator, drivetrain, true, false));
        operatorController.button(2).onTrue(new MoveArm(Constants.ArmConstants.L2Position, S_Armevator, drivetrain, false, false));
        operatorController.button(3).onTrue(new MoveArm(Constants.ArmConstants.L3Position, S_Armevator, drivetrain, false, false));
        operatorController.button(4).onTrue(new MoveArm(Constants.ArmConstants.L4Position, S_Armevator, drivetrain, false, false));
        operatorController.button(19).onTrue(new MoveArm(Constants.ArmConstants.intakePosition, S_Armevator, drivetrain, true, false));
        operatorController.button(13).onTrue(new MoveArm(Constants.ArmConstants.restPosition, S_Armevator, drivetrain, true, false));
       // operatorController.button(1).onTrue(new MoveArm(Constants.ArmConstants.highAlgaeRemoval, S_Armevator, drivetrain, true));


        operatorController.button(15).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'A', joystick));
        operatorController.button(16).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'B', joystick));
        operatorController.button(14).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'C', joystick));
        operatorController.button(12).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'D', joystick));
        operatorController.button(10).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'E', joystick));
        operatorController.button(8).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'F', joystick));
        operatorController.button(6).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'G', joystick));
        operatorController.button(5).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'H', joystick));
        operatorController.button(7).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'I', joystick));
        operatorController.button(9).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'J', joystick));
        operatorController.button(11).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'K', joystick));
        operatorController.button(20).whileTrue(new AutoAlignCoralScore(drivetrain, S_Armevator, 'L', joystick));

        operatorController.button(17).onTrue(new InstantCommand(() -> S_Climber.lock()));
        operatorController.button(17).onFalse(new InstantCommand(() -> S_Climber.unlock()));

        operatorController.axisGreaterThan(3, 0.115).onTrue(new DeployClimber(S_Climber));
        operatorController.axisGreaterThan(5, 0.115).onTrue(new Climb(S_Climber));

        //Buttons to force the arm to move
        operatorController.axisGreaterThan(4, 0.115).and(operatorController.button(1)).onTrue(new MoveArm(Constants.ArmConstants.L1Position, S_Armevator, drivetrain, true, true));
        operatorController.axisGreaterThan(4, 0.115).and(operatorController.button(2)).onTrue(new MoveArm(Constants.ArmConstants.L2Position, S_Armevator, drivetrain, true, true));
        operatorController.axisGreaterThan(4, 0.115).and(operatorController.button(3)).onTrue(new MoveArm(Constants.ArmConstants.L3Position, S_Armevator, drivetrain, true, true));
        operatorController.axisGreaterThan(4, 0.115).and(operatorController.button(4)).onTrue(new MoveArm(Constants.ArmConstants.L4Position, S_Armevator, drivetrain, true, true));
        operatorController.axisGreaterThan(4, 0.115).and(operatorController.button(19)).onTrue(new MoveArm(Constants.ArmConstants.intakePosition, S_Armevator, drivetrain, true, true));
        operatorController.axisGreaterThan(4, 0.115).and(operatorController.button(13)).onTrue(new MoveArm(Constants.ArmConstants.restPosition, S_Armevator, drivetrain, true, true));
       
        
        //operatorController.axisGreaterThan(6, 0.115).onTrue(); //to use the analog buttons
        //joystick.y().onTrue(new InstantCommand(() -> drivetrain.setSwerveToX()));

        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putNumber("Drivetrain max speed", MaxSpeed);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return Commands.print("No autonomous command configured");
    }
    public void setDashboardPIDs(double P, double I, double D, double P2, double I2, double D2, double P3, double I3, double D3, double Izone, double FF) {
        drivetrain.setDashPIDS(P, I, D, P2, I2, D2, P3, I3, D3, Izone, FF); 
        S_Armevator.setDashPIDS(P, I, D, P2, I2, D2, P3, I3, D3, Izone, FF); 
        S_Climber.setDashPIDS(P, I, D, P2, I2, D2, P3, I3, D3, Izone, FF); 
        S_AlgaeManipulator.setDashPIDS(P, I, D, P2, I2, D2, P3, I3, D3, Izone, FF); 
    }
    public void resetArmPosition() {
        S_Armevator.resetArmToAbsolute();
    }
    public void initialElevatorRaise(){
        S_Armevator.moveElevatorToPosition(Constants.ArmConstants.restPosition.elevatorTarget);

    }
    public double[] getOperatorAxis() {
        double[] allAxis = {operatorController.getRawAxis(0), 
                            operatorController.getRawAxis(1), 
                            operatorController.getRawAxis(2),
                            operatorController.getRawAxis(3),
                            operatorController.getRawAxis(4),
                            operatorController.getRawAxis(5),
                            operatorController.getRawAxis(6),
                            operatorController.getRawAxis(7)};
        return allAxis;
    }
    public void lockClimber() {
        S_Climber.lock();
    }
    public boolean isElevatorReady() {
        return S_Armevator.isElevatorReady();
    }
    public boolean isShoulderReady() {
        return S_Armevator.isShoulderReady();
    }
    public boolean isWristReady() {
        return S_Armevator.isWristReady();
    }
    public boolean isClimberReady() {
        // return S_Arm.isShoulderReady();
        return true; // TODO: ???
    }
    public boolean isFrontLeftSwerveReady() {
        return drivetrain.isModuleReady(0);
        // return true;
    }
    public boolean isFrontRightSwerveReady() {
        return drivetrain.isModuleReady(1);
        // return false;
    }
    public boolean isBackLeftSwerveReady() {
        return drivetrain.isModuleReady(2);
        // return false;
    }
    public boolean isBackRightSwerveReady() {
        return drivetrain.isModuleReady(3);
        // return false;
    }
    public boolean isGyroReady(){
        // return S_Swerve.isGyroReady(); 
        return true; // TODO: ???
        //return false;
    }
    public boolean isAprilTagCameraReady(){
        // return S_Swerve.isAprilTagCameraReady();
        return true; // TODO: ???        
        //return true;
    }    
    public boolean isFrontLidarReady(){
        //return S_Swerve.isLimelightReady();
        return true; // TODO: ???
        //return true;
    }
    public boolean isRearLidarReady(){
        // return S_Swerve.isLimelightReady();
        return true; // TODO: ???
        //return true;
       }
    public boolean isChuteLidarReady(){
        //return S_Swerve.isLimelightReady();
        return true; // TODO: ???
        //return true;
       } 
  }
