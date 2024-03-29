// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
  double leftX;
  double leftY;
  double rightX;

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  private double deadbandLeftY(){
    if(Math.abs(joystick.getLeftY()* joystick.getLeftY()) > 0.05){
      leftY = joystick.getLeftY();
      return leftY;
    }
    else{
      leftY = 0;
      return leftY;
    }

  }
  private double deadbandLeftX(){
       if(Math.abs(joystick.getLeftX()* joystick.getLeftX()) > 0.05){
      leftX = joystick.getLeftX() ;
      return leftX;
    }
    else{
      leftX = 0;
      return leftX;
    }
  }

    private double deadbandRightX(){
       if(Math.abs(joystick.getRightX() * joystick.getRightX()) > Math.abs(0.05)){
      rightX = joystick.getRightX();
      return rightX;
    }
    else{
      rightX = 0;
      return rightX;
    }
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-deadbandLeftY()* MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-deadbandLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-deadbandRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-deadbandLeftY(), -deadbandLeftX()))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
