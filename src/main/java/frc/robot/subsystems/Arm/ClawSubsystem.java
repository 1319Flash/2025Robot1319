// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX topClawMotor;
  private final TalonFX bottomClawMotor;

  public ClawSubsystem() {
    topClawMotor = new TalonFX(12 , "canivore");
    bottomClawMotor = new TalonFX(13 , "canivore");

    bottomClawMotor.setControl(new Follower(topClawMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
  }

  public Command runGripper(
      ClawSubsystem clawSubsystem , DoubleSupplier shoot , DoubleSupplier intake) {
    return Commands.run(
      () -> topClawMotor.set(shoot.getAsDouble() - intake.getAsDouble()) , clawSubsystem);    
  }
}
