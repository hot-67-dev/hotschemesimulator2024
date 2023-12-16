// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.Waypoint;
import frc.robot.trajectory.CustomHolonomicDriveController;

public class RobotContainer {
  final double MaxSpeed = 2; // 6 meters per second desired top speed (lowered for mayas house so she doesnt get murdered)
  final double MaxAngularRate = 2 * Math.PI; //a rotation per second max angular velocity
  final double leftXdb = .015;
  final double leftYdb = .015;
  final double rightXdb = .02;

  // pid for autons
  private final PIDController xController = new PIDController(1, 0.0, 0.0);
  private final PIDController yController = new PIDController(1, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(1, 0, 0);
  // private final ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI)); // 1 r/s & .5 r/s^2

  private final CustomHolonomicDriveController HoloDriveController = new CustomHolonomicDriveController(xController, yController, thetaController);

  // trajectory planner setup, m_balls is waypoints in trajectory
  private List<Waypoint> m_balls;
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
    HoloDriveController.setTolerance(new Pose2d(.2, .2, new Rotation2d(15)));
    autonTimer.restart();
  }

  // to fix
  public void autonDrivePeriodic() {
    ChassisSpeeds wRizz = HoloDriveController.calculate(logger.getPose2d(), m_TrajectoryGenerator.getDriveTrajectory().sample(autonTimer.get()), m_TrajectoryGenerator.getHolonomicRotationSequence().sample(autonTimer.get()));
    drivetrain.setControl(drive.withVelocityX(wRizz.vxMetersPerSecond).withVelocityY(wRizz.vyMetersPerSecond).withRotationalRate(wRizz.omegaRadiansPerSecond));
  }
}
