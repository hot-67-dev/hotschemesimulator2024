// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  final double MaxSpeed = 2; // 6 meters per second desired top speed (lowered for mayas house so she doesnt get murdered)
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
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
    if(Math.abs(joystick.getLeftY()* joystick.getLeftY()) > 0.05){
      leftY = joystick.getLeftY();

      return leftY;
    } else {
      leftY = 0;

      return leftY;
    }

  }

  // set LX deadband
  private double deadbandLeftX(){
    if(Math.abs(joystick.getLeftX()* joystick.getLeftX()) > 0.05){
      leftX = joystick.getLeftX() ;

      return leftX;
    } else {
      leftX = 0;

      return leftX;
    }
  }

  // set RX deadband
  private double deadbandRightX(){
      if(Math.abs(joystick.getRightX() * joystick.getRightX()) > Math.abs(0.05)){
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

    // start sim code
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // constructor used to have command based process binding, changed to a test value
  public RobotContainer() {
    SmartDashboard.putBoolean("Drive Init", true);
  }

  // to fix when auton works
  public String getAutonomousCommand() {
    return "No autonomous command configured";
  }
}
