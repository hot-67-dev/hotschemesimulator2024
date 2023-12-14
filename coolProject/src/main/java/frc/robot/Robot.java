// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.trajectory.CustomHolonomicDriveController;
import frc.robot.trajectory.CustomTrajectoryGenerator;

import frc.robot.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.List;


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
  private List<Waypoint> m_balls;
  private CustomTrajectoryGenerator m_TrajectoryGenerator;
  private TrajectoryConfig m_TrajectoryConfig;
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_TrajectoryGenerator = new CustomTrajectoryGenerator();
    m_balls = List.of(new Waypoint(new Translation2d(0,0)), new Waypoint(new Translation2d(1, 1)));
    m_TrajectoryConfig = new TrajectoryConfig(1.5, .2);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    m_TrajectoryGenerator.generate(m_TrajectoryConfig, m_balls);
  }

  @Override
  public void teleopInit() {
    m_robotContainer.stickDrive();
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
    // updateSimState(0.02, 12); static ref error (fix later)
  }
}
