// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.Waypoint;
import frc.robot.trajectory.CustomHolonomicDriveController;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class RobotContainer {
  final double MaxSpeed = 2; // 6 meters per second desired top speed (lowered for mayas house so she doesnt get murdered)
  final double MaxAngularRate = 2 * Math.PI; //a rotation per second max angular velocity
  final double leftXdb = .015;
  final double leftYdb = .015;
  final double rightXdb = .02;

    // pid for autons
  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  private final CustomHolonomicDriveController customHolonomicDriveController = new CustomHolonomicDriveController(xController, yController, thetaController);

  private List<Waypoint> m_balls;
  private CustomTrajectoryGenerator m_TrajectoryGenerator;
  private TrajectoryConfig m_TrajectoryConfig;

  private Rotation2d robotRotation = new Rotation2d(0);
  private Pose2d robotPose = new Pose2d(0, 0, robotRotation);

  // // all meters per sec
  // private static double maxVelocity;
  // private static double maxAccel;
  // private static double maxCentripAccel;

  private final Timer autonTimer = new Timer();

  double leftX;
  double leftY;
  double rightX;


  /* Setting up bindings for necessary control of the swerve drive platform */
  XboxController  joystick = new XboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  // set LY deadband
  private double deadbandLeftY(){
    if(Math.abs(joystick.getLeftY()* joystick.getLeftY()) > leftYdb){
      leftY = joystick.getLeftY();

      return leftY;
    } else {
      leftY = 0;

      return leftY;
    }

  }

  // set LX deadband
  private double deadbandLeftX(){
    if(Math.abs(joystick.getLeftX()* joystick.getLeftX()) > leftXdb){
      leftX = joystick.getLeftX() ;

      return leftX;
    } else {
      leftX = 0;

      return leftX;
    }
  }

  // set RX deadband
  private double deadbandRightX(){
      if(Math.abs(joystick.getRightX() * joystick.getRightX()) > rightXdb){
    rightX = joystick.getRightX();

    return rightX;
    } else {
      rightX = 0;

      return rightX;
    }
  }

  // method called in teleop, drives swerve with joysticks
  public void stickDrive() {
    // sets current control to drive
    drivetrain.setControl(drive.withVelocityX(-deadbandLeftY()* MaxSpeed).withVelocityY(-deadbandLeftX() * MaxSpeed).withRotationalRate(-deadbandRightX() * MaxAngularRate));
    
    // brakemode
    if (joystick.getAButton()) {
      drivetrain.setControl(brake);
    }
    
    // pointmode
    if (joystick.getBButton()) {
      drivetrain.setControl(point.withModuleDirection(new Rotation2d(-deadbandLeftY(), -deadbandLeftX())));
    }

    // sim offset to match field
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // constructor :3
  public RobotContainer() {
    m_TrajectoryGenerator = new CustomTrajectoryGenerator();
    m_balls = List.of(new Waypoint(new Translation2d(0,0)), new Waypoint(new Translation2d(1, 1)));
    m_TrajectoryConfig = new TrajectoryConfig(1.5, .2);
  }

  public void simulationUpdate() {
    drivetrain.updateSimState(0.02, 12);

  }

  public void autonSetup() {
    m_TrajectoryGenerator.generate(m_TrajectoryConfig, m_balls);
    // autonTimer.restart();
    // m_TrajectoryGenerator.getDriveTrajectory().sample(autonTimer.get());
  }

  // to fix when auton works
  public void autonDrivePeriodic() {
    // m_TrajectoryGenerator.getDriveTrajectory().sample(5).poseMeters;
    drivetrain.setControl(brake);
    logger.getPose2d().getY();
  } // TOD0: add holo controller for velocity
}
