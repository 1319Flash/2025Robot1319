// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
  private final TalonFX shoulderMotor;
  private final MotionMagicVoltage motionMagic;
  private final DoubleSolenoid shoulderDoubleSolenoid; 
  
  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    shoulderMotor = new TalonFX(10, "canivore");
    motionMagic = new MotionMagicVoltage(0);
    shoulderDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 30;

    HardwareLimitSwitchConfigs HLSC = new HardwareLimitSwitchConfigs();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(.5)
    .withMotionMagicAcceleration(.5)
    .withMotionMagicJerk(25);

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0;
    slot0.kV = 0;
    slot0.kA = 0;
    slot0.kP = 20;
    slot0.kI = 0;
    slot0.kD = 0.5;

    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    shoulderMotor.setNeutralMode(NeutralModeValue.Brake);

    HLSC.ReverseLimitAutosetPositionEnable = true;
    HLSC.ReverseLimitAutosetPositionValue = 0.0;

    StatusCode status = shoulderMotor.getConfigurator().apply(cfg);
    if (!status.isOK()) {
      System.out.println("Configuration failed: " + status);
    }
    
  }

  @Override
  public void periodic() {
    
  }

  public Command setShoulderPosition(double rotations) {
    return runOnce(() -> shoulderMotor.setControl(motionMagic.withPosition(rotations).withSlot(0)));
  }

  public Command runShoulderWithSpeed(DoubleSupplier speedSupplier) {
    return run(() -> shoulderMotor.set(speedSupplier.getAsDouble()));
  }

  public Command extendShoulderSolenoid(ShoulderSubsystem shoulderSubsystem){
    return new InstantCommand(()-> shoulderDoubleSolenoid.set(Value.kForward), shoulderSubsystem);
  }

  public Command retreactShoulderSolenoid(ShoulderSubsystem shoulderSubsystem){
    return new InstantCommand(() -> shoulderDoubleSolenoid.set(Value.kReverse), shoulderSubsystem);
  }
}