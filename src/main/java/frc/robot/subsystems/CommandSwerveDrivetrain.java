package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

import org.opencv.core.Size;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private List<PhotonPipelineResult> rpiCoralScoreResult;
    private PhotonCamera rpi;
    private PhotonCamera driverCam;
    private double bestAprilTagTargetX;    
    private double bestAprilTagTargetY;
    private double bestAprilTagTargetSize = 0;
    private int bestAprilTagTargetID;
    private double[] dashPIDS = new double[11];
    private double distanceLaserAvg = 0;
    private double bestAprilTagXMeters = 0.0;
    private double bestAprilTagYMeters = 0.0;
    private DutyCycle distanceLaser;
    private DutyCycle rearLidar;
    private DutyCycle chuteLidar;
    private Transform3d cameraToTag;
    private boolean hasAprilTagTarget = false;
    private int aprilTagTargetRequest = 7;
    private boolean isAutoAligning = false;
    private double distanceLaserSum = 0;
    private int distanceLaserSumSize = 10;
    double[] distanceLidarReadings = new double[10];
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        distanceLaser = new DutyCycle(new DigitalInput(0));
        rearLidar = new DutyCycle(new DigitalInput(1));
        chuteLidar = new DutyCycle(new DigitalInput(2));
        rpi = new PhotonCamera("Arducam_Main");
       // rpi.setPipelineIndex(1);
        driverCam = new PhotonCamera("driverCam");
        driverCam.setDriverMode(true);
        
        for (int i = 0; i < distanceLaserSumSize; i++) {
            distanceLidarReadings[i] = 0;
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        distanceLaser = new DutyCycle(new DigitalInput(0));
        rearLidar = new DutyCycle(new DigitalInput(1));
        chuteLidar = new DutyCycle(new DigitalInput(2));
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        distanceLaser = new DutyCycle(new DigitalInput(0));
        rearLidar = new DutyCycle(new DigitalInput(1));
        chuteLidar = new DutyCycle(new DigitalInput(2));
        configureAutoBuilder();
    }
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(7, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
      //  rpi.setPipelineIndex(2);
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        rpiCoralScoreResult = rpi.getAllUnreadResults();   
       // PhotonTargetSortMode test;
       // test = PhotonTargetSortMode.Largest;
       // rpiCoralScoreResult.sort(new Comparator<T>() {
       
        
        if (!rpiCoralScoreResult.isEmpty()) {
            var res = rpiCoralScoreResult.get(rpiCoralScoreResult.size() - 1);
            if (res.hasTargets()) {
                bestAprilTagTargetSize = 0;
                hasAprilTagTarget = true;
                //SmartDashboard.putNumber("April tag target #", res.getTargets().size());
                //var bestTarget = res.getBestTarget();
                for (int i = 0; i < (res.getTargets().size()); i++) {
                    var currentTag = res.getTargets().get(i);
                    if (currentTag.area > bestAprilTagTargetSize) {
                        bestAprilTagTargetX = currentTag.yaw;
                        bestAprilTagTargetY = currentTag.pitch;
                        bestAprilTagTargetID = currentTag.fiducialId;
                        bestAprilTagTargetSize = currentTag.area;
                        bestAprilTagXMeters = currentTag.bestCameraToTarget.getTranslation().getY();
                        bestAprilTagYMeters = currentTag.bestCameraToTarget.getTranslation().getX();
                    }
                    //SmartDashboard.putNumber("April targets list size", res.getTargets().size());
                    // if (res.getTargets().get(i).getFiducialId() == (aprilTagTargetRequest)) {
                    //     bestAprilTagTargetX = res.getTargets().get(i).yaw;
                    //     bestAprilTagTargetY = res.getTargets().get(i).pitch;
                    //     bestAprilTagTargetID = res.getTargets().get(i).fiducialId;
                    //     i = res.getTargets().size(); //TODO kinda jank, maybe change later
                    // }
                }
               
               // cameraToTag = bestTarget.getBestCameraToTarget();
            }
            else {
                hasAprilTagTarget = false;
               bestAprilTagTargetX =0;
               bestAprilTagTargetY = 0;
               bestAprilTagTargetID = 0;
               bestAprilTagXMeters = 0;
               bestAprilTagYMeters = 0;
            }
        }
        else {
            hasAprilTagTarget = false;
           // bestAprilTagTargetX =0;
           // bestAprilTagTargetY = 0;
            //bestAprilTagTargetID = 0;
        }
        // for (int i =  0; i < distanceLaserSumSize - 1; i++) {
        //     distanceLidarReadings[i] = distanceLidarReadings[i + 1];
        // }
        // distanceLidarReadings[distanceLaserSumSize - 1] = getDistanceLaser();
        // for (int i = 0; i < distanceLaserSumSize; i++) {
        //     distanceLaserSum += distanceLidarReadings[i];
        // }
        // distanceLaserAvg = distanceLaserSum / 10.0;
        // distanceLaserSum = 0;
        SmartDashboard.putNumber("April Tag Best ID", bestAprilTagTargetID);
        SmartDashboard.putNumber("April Tag X", bestAprilTagTargetX);
        SmartDashboard.putNumber("April Tag Y", bestAprilTagTargetY);
        SmartDashboard.putNumber("Gyro", getgyroValue());
      //  SmartDashboard.putString("2D pose X", this.getState().Pose.getMeasureX().toString());
       // SmartDashboard.putString("2D pose Y", this.getState().Pose.getMeasureY().toString());
        SmartDashboard.putNumber("Distance Laser", this.getDistanceLaser());
        SmartDashboard.putNumber("Distance Laser Average", distanceLaserAvg);
        SmartDashboard.putNumber("Rear Lidar", getRearLidar());
        SmartDashboard.putNumber("Chute Lidar", getChuteLidar());
         SmartDashboard.putNumber("Robot Speed Y", getRobotSpeeds().vxMetersPerSecond);
         SmartDashboard.putNumber("Robot speeds X", getRobotSpeeds().vyMetersPerSecond);
         SmartDashboard.putNumber("April Tag X meters", bestAprilTagXMeters);
         SmartDashboard.putNumber("April Tag Y meters", bestAprilTagYMeters);
         SmartDashboard.putBoolean("Is RPI camera sending data", rpi.isConnected());
        // SmartDashboard.putNumber("Robot Module Speed", this.getState().ModuleStates[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("Module 0 RPS", this.getModule(0).getDriveMotor().getRotorVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Module 1 RPS", this.getModule(1).getDriveMotor().getRotorVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Module 2 RPS", this.getModule(2).getDriveMotor().getRotorVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Module 3 RPS", this.getModule(3).getDriveMotor().getRotorVelocity().getValueAsDouble());

        //  var estYDist = 4.422/(Math.tan(Math.toRadians(bestAprilTagTargetY + 15.87))); //4.422 is height difference from middle of the camera to middle of the april tag, 15.87 is the angle of the camera       
        //  var xDist = (estYDist) * Math.tan(Math.toRadians(bestAprilTagTargetX));
        // SmartDashboard.putNumber("Camera est Y dist", estYDist);
        // SmartDashboard.putNumber("Camera est X dist", xDist);
         
        //SmartDashboard.putNumber("Path Coral Align Estimated Y", cameraToTag.getX());
        //SmartDashboard.putNumber("Path Coral Align Estimated X", cameraToTag.getY());
        //SmartDashboard.putString("cameraToTag", cameraToTag.toString());

    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
     public double bestAprilTagXMeters() {
        return bestAprilTagXMeters;
     }
     public double bestAprilTagYMeters() {
        return bestAprilTagYMeters;
     }
     public double getAprilTagX() {
        return bestAprilTagTargetX;
     }
     public double getAprilTagY() {
         return bestAprilTagTargetY;
     }
     public int getBestAprilTagID() {
         return bestAprilTagTargetID;
     }
     public Transform3d getCameraToTarget() {
        return cameraToTag;
     }
     public boolean hasAprilTagTarget() {
         return hasAprilTagTarget;
     }
     public double getgyroValue() {
         
         return this.getState().Pose.getRotation().getDegrees();
     }
     public void stopDrive() {
         for (int i = 0; i < 4; i++ ) {
            this.getModules()[i].getDriveMotor().stopMotor();
            
         }
     }
     public void setDashPIDS(double P, double I, double D, double P2, double I2, double D2, double P3, double I3, double D3, double Izone, double FF) {
        dashPIDS[0] = P;
        dashPIDS[1] = I;
        dashPIDS[2] = D;
        dashPIDS[3] = P2;
        dashPIDS[4] = I2;
        dashPIDS[5] = D2;
        dashPIDS[6] = P3;
        dashPIDS[7] = I3;
        dashPIDS[8] = D3;
        dashPIDS[9] = Izone;
        dashPIDS[10] = FF;
     }
     public double[] getDashPIDS() {
        return dashPIDS;
     }
     public double getDistanceLaser() {
        return distanceLaser.getOutput() * 400;
    }
    public double getRearLidar(){
        return rearLidar.getOutput() * 400; 
    }
    public double getChuteLidar() {
        return chuteLidar.getOutput() * 800; //20 is the lowest consistent reading
    }
    public void setAprilTagTargetRequest(int tagID) {
        aprilTagTargetRequest = tagID;
    }
    public ChassisSpeeds getRobotSpeeds() {
        double fieldYSpeed = this.getState().Speeds.vxMetersPerSecond;
        double fieldXSpeed = this.getState().Speeds.vyMetersPerSecond;
        double fieldRotationSpeed = this.getState().Speeds.omegaRadiansPerSecond;     
        Rotation2d robotAngle = this.getState().RawHeading;  
        return this.getState().Speeds; 
        //return ChassisSpeeds.fromFieldRelativeSpeeds(fieldYSpeed, fieldXSpeed, fieldRotationSpeed, robotAngle);
    }
    public boolean getIsAutoAligning() {
        return isAutoAligning;
    }
    public void setIsAutoAligning(boolean setVal) {
        isAutoAligning = setVal;
    }
    public double getDistanceLaserAverage() {
        return distanceLaserAvg;
    }
    public boolean isModuleReady(int module) {
        double degrees = this.getState().ModulePositions[module].angle.getDegrees();

         if (degrees != 0) {
          return true;
         }
        return false;
    }
    public boolean isAprilTagCameraReady() {
        return rpi.isConnected();
    }    
    public boolean isFrontLidarReady() {
        return ((getDistanceLaser() > 0));
    }
    public boolean isRearLidarReady() {
        return (getRearLidar() > 0);
    }
    public boolean isChuteLidarReady() {
        return (getChuteLidar() > 10 && getChuteLidar() < 50);
    }
    public boolean isGyroReady() {
        return (getgyroValue() > 0);
    }

}
