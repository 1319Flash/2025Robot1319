// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {
  private final DoubleSolenoid hookDoubleSolenoid;
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);


  public HookSubsystem() {
    hookDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    hookDoubleSolenoid.set(Value.kReverse);
  }

  public boolean getClimberState(){
    return hookDoubleSolenoid.get() != Value.kReverse;
  }

  public double getPressureValue() {
    return m_compressor.getPressure();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isClimberExtended", getClimberState());
    SmartDashboard.putNumber("getPressure", getPressureValue());
  }

  public Command toggleHook(HookSubsystem hookSubsystem) {
      return Commands.runOnce(() -> hookDoubleSolenoid.toggle() , hookSubsystem);
  }
}
