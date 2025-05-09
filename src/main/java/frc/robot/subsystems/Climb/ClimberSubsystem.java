// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final DoubleSolenoid climberDoubleSolenoid;

  public ClimberSubsystem() {
    climberDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    climberDoubleSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
  }

  public Command toggleClimber(ClimberSubsystem climberSubsystem) {
      return Commands.runOnce(() -> climberDoubleSolenoid.toggle() , climberSubsystem);
  }
}
