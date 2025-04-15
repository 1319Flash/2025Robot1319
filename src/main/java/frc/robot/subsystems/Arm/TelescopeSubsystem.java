// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {
  private final TalonFX telescopeMotor;

  public TelescopeSubsystem() {
    telescopeMotor = new TalonFX(11 , "canivore");
  }

  @Override
  public void periodic() {
  }

  public Command telescope(
      TelescopeSubsystem telescopeSubsystem, DoubleSupplier move) {
    return Commands.run(
        () ->telescopeMotor.set(move.getAsDouble()), telescopeSubsystem);
  }

}
