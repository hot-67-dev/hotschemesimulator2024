// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.Pigeon2;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  // private CustomHolonomicDriveController holoDriveControl; // for setting gains specific to swerve, will do later
  private RobotContainer m_robotContainer;


  private static Pigeon2 m_pigeon;


  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_pigeon = new Pigeon2(50, "drivetrain");
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("theta", m_pigeon.getAngle()); // pidgeon bc reseting pos pissed me off
  }

  @Override
  public void autonomousInit() {
    m_pigeon.reset();
    m_robotContainer.autonSetup();
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autonDriveTrajectory();
  }

  @Override
  public void teleopInit() {
    m_pigeon.reset();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.stickDrive();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationUpdate();
  }
}
