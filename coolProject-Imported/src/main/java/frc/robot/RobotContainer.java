// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.InvalidParameterException;
import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  final double MaxSpeed = 2; // 6 meters per second desired top speed (lowered for mayas house so I dont get murdered)
  final double MaxAccel = 1;
  final double MaxAngularRate = Math.PI * 2; // 1 rotation per second max angular velocity
  final double MaxAngularAccel = Math.PI; 
  final double leftXdb = .1;
  final double leftYdb = .1; 
  final double leftA = 1.0101010101;

  final double rightXdb = .15;
  final double rightA = 1.02301790281;

  private final double driveKP = 2;
  private final double turnKP = 2;

  private static Pigeon2 m_pigeon;

  Pose2d lastTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  // double lastTarget = 0;

  // pid for autons
  private final ProfiledPIDController xController = new ProfiledPIDController(driveKP, .6, .025, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  private final ProfiledPIDController yController = new ProfiledPIDController(driveKP, .6, .025, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(turnKP, 0.0, 0.0, new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularAccel));

  private final ProfiledPIDController velocityController = new ProfiledPIDController(driveKP, 0, 0, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  // private final HolonomicDriveController HoloDriveController = new HolonomicDriveController(xController, yController, thetaController); // built in controller
  private final CustomHolonomicDriveController HoloDriveController = new CustomHolonomicDriveController(xController, yController, thetaController); // custom controller


  // trajectory planner setup, m_points is waypoints in trajectory
  private List<Pose2d> m_poses = List.of(new Pose2d(4 , 0 ,Rotation2d.fromDegrees(0)),
                                        new Pose2d(5.5 , 1.5 , Rotation2d.fromDegrees(0)),
                                        new Pose2d(3.7, 3.1, Rotation2d.fromDegrees(0)),
                                        new Pose2d(8, 2, Rotation2d.fromDegrees(0)),
                                        new Pose2d(1, 1.5, Rotation2d.fromDegrees(0))
                                          );
                                        
  // for loops dont like making new objects??? im sure i did somthing dumb - Maya
  public List<Waypoint> m_points = List.of(new Waypoint().fromHolonomicPose(m_poses.get(0)),
                                          new Waypoint().fromHolonomicPose(m_poses.get(1)),
                                          new Waypoint().fromHolonomicPose(m_poses.get(2)),
                                          new Waypoint().fromHolonomicPose(m_poses.get(3)),
                                          new Waypoint().fromHolonomicPose(m_poses.get(4))
                                          );

  // trajectory object
  private CustomTrajectoryGenerator m_TrajectoryGenerator;
  private TrajectoryConfig m_TrajectoryConfig;

  private CustomTrajectoryGenerator m_dynamicTrajectoryGenerator;
  private TrajectoryConfig m_dynamicTrajectoryConfig;

  private int poseCounter;

  // local stick vars for optimization not in method for intermethod sharing
  double LY;
  double LX;
  double RX;

  private final Timer autonTimer = new Timer();

  private final Timer dynamicTimer = new Timer();

  /* Setting up bindings for necessary control of the swerve drive platform */
  XboxController  joystick = new XboxController(0); // My joystick

  SwerveDrivetrain drivetrain  = TunerConstants.DriveTrain;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); // field-centric
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.Idle coast = new SwerveRequest.Idle();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);


  // constructor :3 should probably move code
  public RobotContainer() {
    m_pigeon = new Pigeon2(50, "drivetrain");
  }

  // set LY deadband
  private double deadbandLeftY() {
    LY = joystick.getLeftY();
    if (LY > leftYdb) {

      return (1 / (1 - leftYdb)) * (LY - leftYdb);
    } else if (LY < -leftYdb) {
      
      return (-1 / (-1 - leftYdb)) * (LY + leftYdb); // should optimise by calculating in var deff at class start
    } else {

      return 0;
    }
  }

  // set LX deadband
  private double deadbandLeftX() {
    LX = joystick.getLeftX();
    if (LX > leftXdb) {

      return (1 / (1 - leftXdb)) * (LX - leftXdb);
    } else if (LX < -leftXdb) {
      
      return (-1 / (-1 - leftXdb)) * (LX + leftXdb); // should optimise by calculating in var deff at class start
    } else {

      return 0;
    }
  }

  // set RX deadband
  private double deadbandRightX() {
    RX = joystick.getRightX();
    if (RX > rightXdb) {

      return (1 / (1 - rightXdb)) * (RX - rightXdb);
    } else if (RX < -rightXdb) {
      
      return (-1 / (-1 - rightXdb)) * (RX + rightXdb); // should optimise by calculating in var deff at class start
    } else {

      return 0;
    }
  }


  // go to https://www.desmos.com/calculator/lj6z51c59x to generate the A constant
  private double smartDeadbandLeftY(double dbWidth, double a) {
    LY = joystick.getLeftY();
    if (Math.abs(LY) >= dbWidth) {

      return a * (LY + dbWidth) * LY * (LY - dbWidth);
    } else {

      return 0;
    }
  }


    // set LX deadband
  private double smartDeadbandLeftX(double dbWidth, double a) {
    LX = joystick.getLeftX();
    if (Math.abs(LX) > dbWidth) {
      
      return a * (LX + dbWidth) * LX * (LX - dbWidth);
    } else {

      return 0;
    }
  }

  // set RX deadband
  private double smartDeadbandRightX(double dbWidth, double a) {
    RX = joystick.getRightX();
    if (Math.abs(RX) > dbWidth) {

    return a * (RX + dbWidth) * RX * (RX - dbWidth);
    } else {

      return 0;
    }
  }



  // method called in teleop, drives swerve with joysticks
  public void stickDrive() {
    // sets current control to drive

    // smart deadband sticks better low speed control (cubic)
    drivetrain.setControl(drive.withVelocityX(-smartDeadbandLeftY(leftYdb, leftA) * MaxSpeed)
                              .withVelocityY(-smartDeadbandLeftX(leftXdb, leftA) * MaxSpeed)
                              .withRotationalRate(-smartDeadbandRightX(rightXdb, rightA) * MaxAngularRate)); 
    
     // normal(ish) deadband sticks better high speed control (translated linear)
    // drivetrain.setControl(drive.withVelocityX(-deadbandLeftY() * MaxSpeed)
    //                           .withVelocityY(-deadbandLeftX() * MaxSpeed)
    //                           .withRotationalRate(-deadbandRightX() * MaxAngularRate));

    // brakemode
    if (joystick.getAButton()) {
      drivetrain.setControl(brake);
    }
    
    // pointmode
    if (joystick.getBButton()) {
      drivetrain.setControl(point.withModuleDirection(new Rotation2d(-deadbandLeftY(), -deadbandLeftX())));
    }

    // reset pose2d (for auton development)
    if (joystick.getYButton()) {
      logger.resetAdjPose();
    }

    // reset pidgeon
    if (joystick.getXButton()) {
      m_pigeon.reset();
    }

    // sim offset to match field
    if (Utils.isSimulation()) {
      // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  public void simulationUpdate() {
    drivetrain.updateSimState(0.02, 12);

  }

  // should probably made an auton class when I started :3


  public void autonSetup() {
    m_pigeon.reset();

    drivetrain.registerTelemetry(logger::telemeterize);

    m_TrajectoryGenerator = new CustomTrajectoryGenerator();
    m_TrajectoryConfig = new TrajectoryConfig(MaxSpeed, MaxAccel);
    m_TrajectoryConfig.setStartVelocity(logger.velocities.getNorm());
    m_TrajectoryConfig.setEndVelocity(0);
    m_TrajectoryGenerator.generate(m_TrajectoryConfig, m_points);
    HoloDriveController.setTolerance(new Pose2d(.2, .2, Rotation2d.fromDegrees(5))); // T0D0: fix holo drive tolarances to actual fucking numbers not a random ass pose 2d bc this code cant get any less readable
    
    xController.reset(logger.adjustedPose.getX(), logger.velocities.getX());
    yController.reset(logger.adjustedPose.getY(), logger.velocities.getY());
    thetaController.reset(logger.pose.getRotation().getDegrees(), logger.velocities.getAngle().getDegrees()); // assumes no rotational movement


    autonTimer.restart();
  }

  public void autonDriveTrajectory() {
    drivetrain.registerTelemetry(logger::telemeterize);
    ChassisSpeeds autoSpeeds = HoloDriveController.calculate(logger.adjustedPose, 
                        m_TrajectoryGenerator.getDriveTrajectory().sample(autonTimer.get()),
                        m_TrajectoryGenerator.getHolonomicRotationSequence().sample(autonTimer.get())
                       );
    if (!HoloDriveController.atReference()) {
      drivetrain.setControl(drive.withVelocityX(autoSpeeds.vxMetersPerSecond)
                                .withVelocityY(autoSpeeds.vyMetersPerSecond)
                                .withRotationalRate(autoSpeeds.omegaRadiansPerSecond));

    } // else {
    //   drivetrain.setControl(brake);
    // }
  }


  public void resetBetterDynamics(Pose2d poseTolarance, Double velocityTolarance) {
    drivetrain.registerTelemetry(logger::telemeterize);


    xController.reset(logger.adjustedPose.getX(), logger.velocities.getX());
    yController.reset(logger.adjustedPose.getY(), logger.velocities.getY());
    velocityController.reset(logger.adjustedPose.getTranslation().getNorm(), logger.velocities.getNorm());
    thetaController.reset(logger.pose.getRotation().getDegrees(), logger.velocities.getAngle().getDegrees());

    xController.setTolerance(poseTolarance.getX());
    yController.setTolerance(poseTolarance.getY());
    velocityController.setTolerance(velocityTolarance);
    thetaController.setTolerance(poseTolarance.getRotation().getDegrees());

    poseCounter = 0;
  }

  public void betterDynamics(Pose2d targetPose2d) {
    drivetrain.registerTelemetry(logger::telemeterize);

    ChassisSpeeds autoChassisSpeed = HoloDriveController.calculate(logger.adjustedPose, targetPose2d, velocityController.calculate(logger.velocities.getNorm(), logger.adjustedPose.minus(targetPose2d).getTranslation().getNorm()), Rotation2d.fromDegrees(0), 0);    
    drivetrain.setControl(drive.withVelocityX(autoChassisSpeed.vxMetersPerSecond).withVelocityY(autoChassisSpeed.vyMetersPerSecond)); // thetaController.calculate(logger.velocities.getAngle().getDegrees(), targetPose2d.getRotation().getDegrees()))

    if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
      poseCounter ++;
    } else if (poseCounter >= 10) {  
      drivetrain.setControl(brake); //might be more efficent to have the velocitycontroll to be in an if statment as to not have to calc pid (depends on rio load)
    }
  }
}
