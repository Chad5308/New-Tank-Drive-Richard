// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup
  (
    new PWMVictorSPX(DriveConstants.kLeftMotor1Port),
    new PWMVictorSPX(DriveConstants.kLeftMotor2Port));
  )

   private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup
  (
    new PWMVictorSPX(DriveConstants.kRightMotor1Port),
    new PWMVictorSPX(DriveConstants.kRightMotor2Port));
  )
  }

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors)
  
  private final Encoder m_leftEncoder = new Encoder
  (
    DriveConstants.kLeftEncoderPorts[0]
    DriveConstants.kLeftEncoderPorts[1]
    DriveConstants.kLeftEncoderReversed
  );

   private final Encoder m_RightEncoder = new Encoder
  (
    DriveConstants.kRightEncoderPorts[0]
    DriveConstants.kRightEncoderPorts[1]
    DriveConstants.kRightEncoderReversed
  );

  public DriveSubsystem()
  {
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void arcadeDrive(double fwd, double rot)
  {
    m.arcadeDrive(fwd, rot);
  }

  public void resetEncoders()
  {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public class DriveDistance extends CommandBase 
  {
    private final DriveSubSystem m_drive;
    private final double m_distance;
    private final double m_speed;
  }

  public DriveDistance(double inches, double speed, DriveSubsystems drive)
  {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
