// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.Waypoint;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.trajectory.CustomHolonomicDriveController;


public class RobotContainer {
  final double MaxSpeed = 3; // 6 meters per second desired top speed (lowered for mayas house so I dont get murdered)
  final double MaxAccel = .8;
  final double MaxAngularRate = 2 * Math.PI; // 1 rotation per second max angular velocity
  final double MaxAngularAccel = Math.PI; 
  final double leftXdb = .018;
  final double leftYdb = .018;
  final double rightXdb = .02;

  private final double driveKP = 6;
  private final double turnKP = 8;

  // pid for autons
  private final ProfiledPIDController xController = new ProfiledPIDController(driveKP, 0, 0.0, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  private final ProfiledPIDController yController = new ProfiledPIDController(driveKP, 0, 0.0, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(turnKP, 0.0, 0.0, new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularAccel));

  // private final HolonomicDriveController HoloDriveController = new HolonomicDriveController(xController, yController, thetaController); // built in controller
  private final CustomHolonomicDriveController HoloDriveController = new CustomHolonomicDriveController(xController, yController, thetaController); // custom controller


  // trajectory planner setup, m_points is waypoints in trajectory
  private List<Pose2d> m_poses = List.of(new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
                                          new Pose2d(2, -.5, Rotation2d.fromDegrees(0)),
                                          new Pose2d(0, .5, Rotation2d.fromDegrees(0)),
                                          new Pose2d(1, 0, Rotation2d.fromDegrees(0)));

  public List<Waypoint> m_points = List.of(new Waypoint().fromHolonomicPose(m_poses.get(0)),
                                          new Waypoint().fromHolonomicPose(m_poses.get(1)));
  private CustomTrajectoryGenerator m_TrajectoryGenerator;
  private TrajectoryConfig m_TrajectoryConfig;




  private final Timer autonTimer = new Timer();

  // controller db local vars
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
    drivetrain.setControl(drive.withVelocityX(-deadbandLeftY()* MaxSpeed)
                              .withVelocityY(-deadbandLeftX() * MaxSpeed)
                              .withRotationalRate(-deadbandRightX() * MaxAngularRate));
    
    // brakemode
    if (joystick.getAButton()) {
      drivetrain.setControl(brake);
    }
    
    // pointmode
    if (joystick.getBButton()) {
      drivetrain.setControl(point.withModuleDirection(new Rotation2d(-deadbandLeftY(), -deadbandLeftX())));
    }

    // reset pose2d
    if (joystick.getYButton()) {
      logger.resetAdjPose();
    }

    // sim offset to match field
    if (Utils.isSimulation()) {
      // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // constructor :3
  public RobotContainer() {
    
  }

  public void simulationUpdate() {
    drivetrain.updateSimState(0.02, 12);

  }

  // should probably made an auton class when I started :3


  public void autonSetup() {
    m_TrajectoryGenerator = new CustomTrajectoryGenerator();
    m_TrajectoryConfig = new TrajectoryConfig(.3, .01);
    m_TrajectoryConfig.setStartVelocity(0);
    m_TrajectoryConfig.setEndVelocity(0);
    m_TrajectoryGenerator.generate(m_TrajectoryConfig, m_points);
    HoloDriveController.setTolerance(new Pose2d(.3, .3, Rotation2d.fromDegrees(15))); // T0D0: fix holo drive tolarances to actual fucking numbers not a random ass pose 2d bc this code cant get any less readable
    
    drivetrain.registerTelemetry(logger::telemeterize);
    autonTimer.restart();
  }

  // to fix
  public void autonDriveTrajectory() {
    drivetrain.registerTelemetry(logger::telemeterize);

    ChassisSpeeds autoSpeeds = HoloDriveController.calculate(logger.getAdjustedPose2d(), 
                  m_TrajectoryGenerator.getDriveTrajectory().sample(autonTimer.get()),
                  m_TrajectoryGenerator.getHolonomicRotationSequence().sample(autonTimer.get())
                  );
    drivetrain.setControl(drive.withVelocityX(autoSpeeds.vxMetersPerSecond)
                              .withVelocityY(autoSpeeds.vyMetersPerSecond)
                              .withRotationalRate(autoSpeeds.omegaRadiansPerSecond));
  }
}
