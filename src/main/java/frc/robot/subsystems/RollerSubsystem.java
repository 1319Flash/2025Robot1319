// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RollerSubsystem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final DoubleSolenoid rampDoubleSolenoid;

  public RollerSubsystem() {
    rollerMotor = new TalonFX(9, "canivore");

    rampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    rampDoubleSolenoid.set(Value.kForward);
  }

  public void setMotorSpeed(){
    rollerMotor.set(0.2);
  }
  
  @Override
  public void periodic() {
  }

  public void stopMotor() {
    rollerMotor.set(0);
  }

  public Command runRoller(
    RollerSubsystem rollerSubsystem, DoubleSupplier forward , DoubleSupplier reverse) {
      return Commands.run(
        () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }

  public Command stopRoller(RollerSubsystem rollerSubsystem) {
    return new InstantCommand(() -> rollerSubsystem.stopMotor(), rollerSubsystem);
  }

  public Command extendRollerSolenoid(RollerSubsystem rollerSubsystem){
    return new InstantCommand(()-> rampDoubleSolenoid.set(Value.kForward), rollerSubsystem);
  }
 
  public Command retractRollerSolenoid(RollerSubsystem rollerSubsystem) {
    return new InstantCommand(() -> rampDoubleSolenoid.set(Value.kReverse), rollerSubsystem);
  }
}
