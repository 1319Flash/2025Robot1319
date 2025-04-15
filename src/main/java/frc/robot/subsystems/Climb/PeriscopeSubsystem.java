// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PeriscopeSubsystem extends SubsystemBase {
  private final Solenoid periscopeSolenoid;

  public PeriscopeSubsystem() {
    periscopeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 10);
    periscopeSolenoid.set(false);
  }

  @Override
  public void periodic() {
  }

  public Command togglePeriscope(PeriscopeSubsystem periscopeSubsystem) {
      return Commands.runOnce(() -> periscopeSolenoid.toggle() , periscopeSubsystem);
  }
}
