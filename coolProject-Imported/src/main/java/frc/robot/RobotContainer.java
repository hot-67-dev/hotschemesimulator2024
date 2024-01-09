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
  final double MaxSpeed = 3; // 6 meters per second desired top speed (lowered for mayas house so I dont get murdered)
  final double MaxAccel = .8;
  final double MaxAngularRate = Math.PI; // 1 rotation per second max angular velocity
  final double MaxAngularAccel = Math.PI * .5; 
  final double leftXdb = .1;
  final double leftYdb = .1; 
  final double leftA = 1.0101010101;

  final double rightXdb = .15;
  final double rightA = 1.02301790281;

  private final double driveKP = 6;
  private final double turnKP = 2;

  private static Pigeon2 m_pigeon;

  Pose2d lastTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  // double lastTarget = 0;

  // pid for autons
  private final ProfiledPIDController xController = new ProfiledPIDController(driveKP, 0, 0.0, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  private final ProfiledPIDController yController = new ProfiledPIDController(driveKP, 0, 0.0, new TrapezoidProfile.Constraints(MaxSpeed, MaxAccel));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(turnKP, 0.0, 0.0, new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularAccel));

  // private final HolonomicDriveController HoloDriveController = new HolonomicDriveController(xController, yController, thetaController); // built in controller
  private final CustomHolonomicDriveController HoloDriveController = new CustomHolonomicDriveController(xController, yController, thetaController); // custom controller


  // trajectory planner setup, m_points is waypoints in trajectory
  private List<Pose2d> m_poses = List.of(new Pose2d(6 , 2.5 ,Rotation2d.fromDegrees(0)),
                                        new Pose2d(9.3 , -2.5 , Rotation2d.fromDegrees(0)),
                                        new Pose2d(6.9, -6.25, Rotation2d.fromDegrees(0)),
                                        new Pose2d(3, -3, Rotation2d.fromDegrees(0)),
                                        new Pose2d(2.5, 0, Rotation2d.fromDegrees(0))
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
    // drivetrain.setControl(drive.withVelocityX(-smartDeadbandLeftY(leftYdb, leftA) * MaxSpeed)
    //                           .withVelocityY(-smartDeadbandLeftX(leftXdb, leftA) * MaxSpeed)
    //                           .withRotationalRate(-smartDeadbandRightX(rightXdb, rightA) * MaxAngularRate)); 
    
     // normal(ish) deadband sticks better high speed control (translated linear)
    drivetrain.setControl(drive.withVelocityX(-deadbandLeftY() * MaxSpeed)
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

    m_TrajectoryGenerator = new CustomTrajectoryGenerator();
    m_TrajectoryConfig = new TrajectoryConfig(MaxSpeed, MaxAccel);
    m_TrajectoryConfig.setStartVelocity(0);
    m_TrajectoryConfig.setEndVelocity(0);
    m_TrajectoryGenerator.generate(m_TrajectoryConfig, m_points);
    HoloDriveController.setTolerance(new Pose2d(.05, .05, Rotation2d.fromDegrees(5))); // T0D0: fix holo drive tolarances to actual fucking numbers not a random ass pose 2d bc this code cant get any less readable
    
    xController.reset(logger.adjustedPose.getX(), logger.velocities.getX());
    yController.reset(logger.adjustedPose.getY(), logger.velocities.getY());
    thetaController.reset(logger.pose.getRotation().getDegrees(), 0); // assumes no rotational movement

    drivetrain.registerTelemetry(logger::telemeterize);
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

    } else {
      drivetrain.setControl(brake);
    }
  }

  public void setDynamicTrajectory(Pose2d targetPose2d) {
    m_pigeon.reset();

    drivetrain.registerTelemetry(logger::telemeterize);
    if (!logger.adjustedPose.equals(targetPose2d)) {
      // all this shit for like .2 meters of accuracy bc im too lazy to code pid
      m_dynamicTrajectoryGenerator = new CustomTrajectoryGenerator();
      m_dynamicTrajectoryConfig = new TrajectoryConfig(MaxSpeed, MaxAccel);
      m_dynamicTrajectoryConfig.setStartVelocity(0); // probably not but whatever
      m_dynamicTrajectoryConfig.setEndVelocity(0);
      m_dynamicTrajectoryGenerator.generate(m_dynamicTrajectoryConfig, List.of(new Waypoint().fromHolonomicPose(logger.adjustedPose),
                                                                              new Waypoint().fromHolonomicPose(targetPose2d)));
      HoloDriveController.setTolerance(new Pose2d(.05, .05, Rotation2d.fromDegrees(5))); // T0D0: fix holo drive tolarances to actual fucking numbers not a random ass pose 2d bc this code cant get any less readable
      
      xController.reset(logger.adjustedPose.getX(), logger.velocities.getX());
      yController.reset(logger.adjustedPose.getY(), logger.velocities.getY());
      thetaController.reset(logger.pose.getRotation().getDegrees(), 0); // assumes no rotational movement

      // t0d0 change trajectory generator to raw xytheta controller line --> xController.calculate(logger.adjustedPose.getX(), targetPose2d.getX());
      poseCounter = 0;
      dynamicTimer.restart();
    } else {
      // throw new InvalidParameterException("Already at target position!"); // look dad! i did the error message!!!!! // commented to let code still run (make it a warning or somthin)
    }


  }

  public void driveDynamicTrajectory(Pose2d targetPose2d) {
    if (!lastTarget.equals(targetPose2d)) {
      setDynamicTrajectory(targetPose2d);
    }
    lastTarget = targetPose2d;
    if (m_dynamicTrajectoryGenerator != null) {
      drivetrain.registerTelemetry(logger::telemeterize);
      ChassisSpeeds autoSpeeds = HoloDriveController.calculate(logger.adjustedPose,  
                          m_dynamicTrajectoryGenerator.getDriveTrajectory().sample(dynamicTimer.get()),
                          m_dynamicTrajectoryGenerator.getHolonomicRotationSequence().sample(dynamicTimer.get())
                          );

      if (!HoloDriveController.atReference()) {
        drivetrain.setControl(drive.withVelocityX(autoSpeeds.vxMetersPerSecond)
                                  .withVelocityY(autoSpeeds.vyMetersPerSecond)
                                  .withRotationalRate(autoSpeeds.omegaRadiansPerSecond));
        // T0D0 When at refrence for over x amount of time, brake/coast
      }
    } else {
      // throw new InvalidParameterException("Trajectory has not been generated/nhave you ran the set trajectory method?"); // commented to let code still run (make it a warning or somthin)
    }
  }


  public void resetBetterDynamics(Pose2d tolarances) {
    drivetrain.registerTelemetry(logger::telemeterize);
    
    xController.reset(logger.adjustedPose.getX(), logger.velocities.getX());
    yController.reset(logger.adjustedPose.getY(), logger.velocities.getY());
    thetaController.reset(logger.pose.getRotation().getDegrees(), 0); //T0D0 ADD THETA SPEED CALC TO LOGGER

    xController.setTolerance(tolarances.getX());
    yController.setTolerance(tolarances.getY());
    thetaController.setTolerance(tolarances.getRotation().getDegrees());

    poseCounter = 0;
  }

  public void betterDynamics(Pose2d targetPose2d) {
    drivetrain.registerTelemetry(logger::telemeterize);

    drivetrain.setControl(drive.withVelocityX(xController.calculate(logger.velocities.getX(), targetPose2d.getX()))
                                .withVelocityY(yController.calculate(logger.velocities.getY(), targetPose2d.getY()))
                                .withRotationalRate(thetaController.calculate(logger.velocities.getAngle().getDegrees(), targetPose2d.getRotation().getDegrees())));

    if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
      poseCounter ++;
    } else if (poseCounter >= 10) {  
      drivetrain.setControl(brake); //might be more efficent to have the velocitycontroll to be in an if statment as to not have to calc pid (depends on rio load)
    }
  }
}
