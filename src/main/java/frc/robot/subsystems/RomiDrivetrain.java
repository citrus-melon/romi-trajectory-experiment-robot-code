// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.Constants;

public class RomiDrivetrain extends SubsystemBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  
  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  private final RomiGyro m_gyro = new RomiGyro();
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.WHEEL_DIAMETER_METER) / Constants.COUNTS_PER_REVOLUTION);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.WHEEL_DIAMETER_METER) / Constants.COUNTS_PER_REVOLUTION);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(m_gyro.getAngleZ())));
  }

  // public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
  //   m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  // }

  public void stopDrive() {
    m_diffDrive.stopMotor();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // public double getLeftDistanceInch() {
  //   return m_leftEncoder.getDistance();
  // }

  // public double getRightDistanceInch() {
  //   return m_rightEncoder.getDistance();
  // }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, new Rotation2d(Math.toRadians(m_gyro.getAngleZ())));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      m_leftEncoder.getRate(),
      m_rightEncoder.getRate()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
