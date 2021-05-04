// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This is the swerve module class
 * Each module consists of a falcon to drive and a talon to turn
 * It gets the position that it should go to from the Drivetrain class
 */


 //imports
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  //a whole bunch of variables
  //mostly use these to get our robot to go to our desired real world speeds
  //drive stuff not done yet, turn is set to real world
  //also use to set PIDs for the modules
  private static final double kWheelRadius = 0.0381;
  public final double driveSpeed;
  public final double turnSpeed;
  private static final int kEncoderResolution = 4096;
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI;
  private final TalonFX m_driveMotor;
  private final TalonSRX m_turningMotor;
  private final PIDController m_drivePidController = new PIDController(1, 0, 0);
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(2.5, 0, 0, new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,kModuleMaxAngularAcceleration));
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /** Creates a new SwerveModule.
   * 
   * @param driveMotorChannel ID for the drive motor
   * @param turningMotorChannel ID for the turning motor
   * declare the modules here, set them to the motor ports and the sensors
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    driveSpeed=0;
    turnSpeed=0;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    m_turningMotor = new TalonSRX(turningMotorChannel);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * 
   * @return
   * returns the speed of the module and what angle it is at in radians
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
  }

  /**
   * 
   * @param desiredState
   * Goes to the state given from the drive train, divides by 2048 to deal with encoder stuff
   */
  public void setDesiredState (SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()*Math.PI/2048.0));

    double driveOutput = m_drivePidController.calculate(m_driveMotor.getSelectedSensorVelocity()/2048.0, state.speedMetersPerSecond);
    //double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    double turnOutput = -m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition()*Math.PI/2048.0, state.angle.getRadians());
    //double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    //m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);

  }

  //Returns the state that we want to drive train so that they can post it per module
  public String getDesiredState (SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()));

    final double driveOutput = m_drivePidController.calculate(m_driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);
    //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    final double turnOutput = -m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(), state.angle.getRadians());
    //final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    return "drive speed: "+driveOutput+" turn speed: "+turnOutput;
  }

  //get speed
  public double getDriveSpeed() {
    return m_driveMotor.getSelectedSensorVelocity();
  }

  //get azimuth speed
  public double getTurnSpeed() {
    return m_turningMotor.getSelectedSensorVelocity();
  }

  //go to a certain speed
  public void setTurnSpeed(double speed) {
    m_turningMotor.set(ControlMode.PercentOutput, speed);
  }

  //get angle
  public double getAzimuthPosition() {
    return m_turningMotor.getSelectedSensorPosition();
  }


  //stop all movement
  public void stop(){
    m_driveMotor.set(ControlMode.PercentOutput, 0);
    m_turningMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
