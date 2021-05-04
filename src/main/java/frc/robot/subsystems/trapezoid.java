// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//also not used

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class trapezoid extends TrapezoidProfileSubsystem {
  /** Creates a new trapezoid. */
  public trapezoid() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        0);
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}
