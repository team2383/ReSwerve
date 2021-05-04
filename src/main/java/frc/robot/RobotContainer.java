// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Link gamepad buttons to commands, and holds autonomous command
 */

package frc.robot;


//more imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... (yea)
  public final Drivetrain m_robotDrive = new Drivetrain();
  public final XboxController m_Controller = new XboxController(0);

  //something to normalize speed
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private static final double DEADBAND = 0.15;

  //sets up deadband and gives gamepad inputs to drive command
  //if we had field relative, would also pass that along to drive train to modify the outputs
  public void driveWithJoystick(boolean fieldRelative) {
      final var xSpeed = -m_xspeedLimiter.calculate(m_Controller.getY(GenericHID.Hand.kLeft))*frc.robot.subsystems.Drivetrain.kMaxSpeed;
      final var ySpeed = -m_yspeedLimiter.calculate(m_Controller.getX(GenericHID.Hand.kLeft))*frc.robot.subsystems.Drivetrain.kMaxSpeed;
      final var rot = -m_rotLimiter.calculate(m_Controller.getX(GenericHID.Hand.kRight))*frc.robot.subsystems.Drivetrain.kMaxAngularSpeed;
      SmartDashboard.putNumber("xSpeed", deadband(xSpeed));
      SmartDashboard.putNumber("ySpeed", deadband(ySpeed));
      SmartDashboard.putNumber("rot", deadband(rot));
      if(deadband(xSpeed)==0 && deadband(ySpeed)==0 && deadband(rot)==0){
        m_robotDrive.stop();
      }
      m_robotDrive.drive(deadband(xSpeed), deadband(ySpeed),  deadband(rot), fieldRelative);
      //m_robotDrive.drive2(deadband(xSpeed));
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    } else {
      return value;
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    return null;
  }
}
