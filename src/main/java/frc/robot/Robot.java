// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  double tunePID_KP;
  double tunePID_KI;
  double tunePID_KD;
  double tunePID_KIz;
  double tunePID_KFF;
  double tunePID_KF_kS;
  double tunePID_KF_kV;
  double tunePID_KF_kA;

  private final RobotContainer m_robotContainer;

  public Robot() {
    //DataLogManager.start();
    m_robotContainer = new RobotContainer();
    tunePID_KP =0; 
    tunePID_KI = 0;
    tunePID_KD = 0; 
    tunePID_KIz = 0; 
    tunePID_KFF = 0; 
    SmartDashboard.putNumber("TunePID P Gain", tunePID_KP);
    SmartDashboard.putNumber("TunePID I Gain", tunePID_KI);
    SmartDashboard.putNumber("TunePID D Gain", tunePID_KD);
    SmartDashboard.putNumber("TunePID I Zone", tunePID_KIz);
    SmartDashboard.putNumber("TunePID Feed Forward", tunePID_KFF);
    SmartDashboard.putNumber("TunePID FF kS", tunePID_KP);
    SmartDashboard.putNumber("TunePID FF kV", tunePID_KI);
    SmartDashboard.putNumber("TunePID FF kA", tunePID_KD);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SignalLogger.start();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double tunePID_Dashboard_P = SmartDashboard.getNumber("TunePID P Gain", 0);
    double tunePID_Dashboard_I = SmartDashboard.getNumber("TunePID I Gain", 0);
    double tunePID_Dashboard_D = SmartDashboard.getNumber("TunePID D Gain", 0);
    double tunePID_Dashboard_Iz = SmartDashboard.getNumber("TunePID I Zone", 0);
    double tunePID_Dashboard_FF = SmartDashboard.getNumber("TunePID Feed Forward", 0);
    double tunePID_Dashboard_FF_kS = SmartDashboard.getNumber("TunePID FF kS", 0);
    double tunePID_Dashboard_FF_kV = SmartDashboard.getNumber("TunePID FF kV", 0);
    double tunePID_Dashboard_FF_kA = SmartDashboard.getNumber("TunePID FF kA", 0);
    double tuneShot_Dashboard_LeftRPM = SmartDashboard.getNumber("TuneShot Left RPM", 0);
    double tuneShot_Dashboard_RightRPM = SmartDashboard.getNumber("TuneShot Right RPM", 0);
    if (tunePID_Dashboard_P != tunePID_KP || tunePID_Dashboard_I != tunePID_KI || tunePID_Dashboard_D != tunePID_KD || tunePID_Dashboard_Iz != tunePID_KIz || tunePID_Dashboard_FF != tunePID_KFF || tunePID_Dashboard_FF_kS != tunePID_KF_kS || tunePID_Dashboard_FF_kV != tunePID_KF_kV || tunePID_Dashboard_FF_kA != tunePID_KF_kA) {
      m_robotContainer.setDashboardPIDs(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D,tunePID_Dashboard_Iz, tunePID_Dashboard_FF);
      tunePID_KP = tunePID_Dashboard_P;
      tunePID_KI = tunePID_Dashboard_I;
      tunePID_KD = tunePID_Dashboard_D;
      tunePID_KIz = tunePID_Dashboard_Iz;
      tunePID_KFF = tunePID_Dashboard_FF;
      tunePID_KF_kS = tunePID_Dashboard_FF_kS;
      tunePID_KF_kV = tunePID_Dashboard_FF_kV;
      tunePID_KF_kA = tunePID_Dashboard_FF_kA;
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
