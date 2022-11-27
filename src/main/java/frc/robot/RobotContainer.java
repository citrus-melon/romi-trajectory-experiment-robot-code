// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("troubleshooting");
  NetworkTableEntry leftReference = table.getEntry("left_reference");
  NetworkTableEntry leftMeasurement = table.getEntry("left_measurement");
  NetworkTableEntry rightReference = table.getEntry("right_reference");
  NetworkTableEntry rightMeasurement = table.getEntry("right_measurement");
  
  PathPlannerTrajectory newPath = PathPlanner.loadPath("New New Path", new PathConstraints(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION));
  
  public Command getFollowTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    RamseteController ramsete = new RamseteController();
    // disabled_Ramsete.setEnabled(false);
    
    PIDController leftController = new PIDController(Constants.KP, Constants.KI, Constants.KD);
    PIDController rightController = new PIDController(Constants.KP, Constants.KI, Constants.KD);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              m_romiDrivetrain.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj,
            m_romiDrivetrain::getPose, // Pose supplier
            ramsete,
            new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA),
            Constants.KINEMATICS, // DifferentialDriveKinematics
            m_romiDrivetrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            leftController, // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            rightController, // Right controller (usually the same values as left controller)
            (leftVolts, rightVolts) -> {
              m_romiDrivetrain.tankDriveVolts(leftVolts, rightVolts);
      
              leftMeasurement.setNumber(m_romiDrivetrain.getWheelSpeeds().leftMetersPerSecond);
              leftReference.setNumber(leftController.getSetpoint());
      
              rightMeasurement.setNumber(m_romiDrivetrain.getWheelSpeeds().rightMetersPerSecond);
              rightReference.setNumber(rightController.getSetpoint());

            }, // Voltage biconsumer
            m_romiDrivetrain // Requires this drive subsystem
        )
    );
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    PathPlannerServer.startServer(5811);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return getFollowTrajectoryCommand(newPath, true).andThen(() -> m_romiDrivetrain.stopDrive());
  }
}
