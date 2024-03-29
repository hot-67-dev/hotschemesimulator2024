package frc.robot.hotpath.commands;

import frc.robot.hotpath.path.PathConstraints;
import frc.robot.hotpath.path.PathPlannerPath;
import frc.robot.hotpath.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** A command group that first pathfinds to a goal path and then follows the goal path. */
public class PathfindThenFollowPathHolonomic extends SequentialCommandGroup {
  /**
   * Constructs a new PathfindThenFollowPathHolonomic command group.
   *
   * @param goalPath the goal path to follow
   * @param pathfindingConstraints the path constraints for pathfinding
   * @param poseSupplier a supplier for the robot's current pose
   * @param currentRobotRelativeSpeeds a supplier for the robot's current robot relative speeds
   * @param robotRelativeOutput a consumer for the output speeds (robot relative)
   * @param config {@link frc.robot.hotpath.util.HolonomicPathFollowerConfig} for configuring the
   *     path following commands
   * @param rotationDelayDistance Distance to delay the target rotation of the robot. This will
   *     cause the robot to hold its current rotation until it reaches the given distance along the
   *     path.
   * @param requirements the subsystems required by this command (drive subsystem)
   */
  public PathfindThenFollowPathHolonomic(
      PathPlannerPath goalPath,
      PathConstraints pathfindingConstraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> currentRobotRelativeSpeeds,
      Consumer<ChassisSpeeds> robotRelativeOutput,
      HolonomicPathFollowerConfig config,
      double rotationDelayDistance,
      Subsystem... requirements) {
    addCommands(
        new PathfindHolonomic(
            goalPath,
            pathfindingConstraints,
            poseSupplier,
            currentRobotRelativeSpeeds,
            robotRelativeOutput,
            config,
            rotationDelayDistance,
            requirements),
        new FollowPathWithEvents(
            new FollowPathHolonomic(
                goalPath,
                poseSupplier,
                currentRobotRelativeSpeeds,
                robotRelativeOutput,
                config,
                requirements),
            goalPath,
            poseSupplier));
  }

  /**
   * Constructs a new PathfindThenFollowPathHolonomic command group.
   *
   * @param goalPath the goal path to follow
   * @param pathfindingConstraints the path constraints for pathfinding
   * @param poseSupplier a supplier for the robot's current pose
   * @param currentRobotRelativeSpeeds a supplier for the robot's current robot relative speeds
   * @param robotRelativeOutput a consumer for the output speeds (robot relative)
   * @param config {@link frc.robot.hotpath.util.HolonomicPathFollowerConfig} for configuring the
   *     path following commands
   * @param requirements the subsystems required by this command (drive subsystem)
   */
  public PathfindThenFollowPathHolonomic(
      PathPlannerPath goalPath,
      PathConstraints pathfindingConstraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> currentRobotRelativeSpeeds,
      Consumer<ChassisSpeeds> robotRelativeOutput,
      HolonomicPathFollowerConfig config,
      Subsystem... requirements) {
    this(
        goalPath,
        pathfindingConstraints,
        poseSupplier,
        currentRobotRelativeSpeeds,
        robotRelativeOutput,
        config,
        0.0,
        requirements);
  }
}
