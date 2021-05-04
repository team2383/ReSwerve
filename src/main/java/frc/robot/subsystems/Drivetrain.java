// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Drivetrain subsystem, uses the Swerve Module class.
 * It translates inputs from the game pad into speeds for each of the swerve modules.
 */

package frc.robot.subsystems;

//imports, wow
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {

  //sets what we want as max real life speeds that we want. Might want to mess around with them a bit more.
  //kMaxSpeed needs to be updated, as given and real life values don't match
  public static final double kMaxSpeed = 0.5;//3.0;
  public static final double kMaxAngularSpeed = Math.PI;

  //sets positions of modules in meters, might want to change for constant to make updating easier
  private final Translation2d m_frontLeftLocation = new Translation2d(0.2683, 0.2683);
  private final Translation2d m_frontRightLocation = new Translation2d(0.2683, -0.2683);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.2683, 0.2683);
  private final Translation2d m_backRightLocation = new Translation2d(-0.2683, -0.2683);
  
  //initializes swerve modules with the motor ports
  public final SwerveModule m_frontLeft = new SwerveModule(20, 0);
  public final SwerveModule m_frontRight = new SwerveModule(21, 1);
  public final SwerveModule m_backLeft = new SwerveModule(22, 2);
  public final SwerveModule m_backRight = new SwerveModule(23, 3);

  //private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

  //Pass the swerve module positions to the kinematics class, so that it can figure out the necessary speeds for each module
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  //private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //m_gyro.reset();
    //
  }

  /**
   * drive boi
   * 
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldRelative
   * 
   * This is the drive command
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    //gets inputs and uses the math to get the wanted speeds
    var swerveModuleStates =
      m_kinematics.toSwerveModuleStates( new ChassisSpeeds(xSpeed,ySpeed,rot));
    //makes sure that the robot isn't going faster than our desired maximum speed
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    //Passes on to the swerve modules what we want them to do
    SmartDashboard.putString("front left", m_frontLeft.getDesiredState(swerveModuleStates[0]));
    SmartDashboard.putString("front right", m_frontRight.getDesiredState(swerveModuleStates[1]));
    SmartDashboard.putString("back left", m_backLeft.getDesiredState(swerveModuleStates[2]));
    SmartDashboard.putString("back right", m_backRight.getDesiredState(swerveModuleStates[3]));
    //Posts to smart dashboard what the wheels should do
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  //test command to make the wheels turn at the speed we want
  public void drive2(double speed){
    m_frontRight.setTurnSpeed(speed);
    m_frontLeft.setTurnSpeed(speed);
    m_backRight.setTurnSpeed(speed);
    m_backLeft.setTurnSpeed(speed);
  }

  //tries to keep track of robot position, can use for field orientation and for autonomous
  /*public void updateOdometry(){
    m_odometry.update(m_gyro.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());
  }*/

  //stops all the wheels
  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
