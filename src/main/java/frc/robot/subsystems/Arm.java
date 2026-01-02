// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Testing branches Astin
// WHYYYYYY

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Arm extends SubsystemBase {

  private static Arm instance;

  private MotionMagicVoltage request = new MotionMagicVoltage(0);

  private double targetPoint;

  private boolean AutoZero = true;

  private Timer VelocityTimer = new Timer();
  
  TalonFX armMotor = new TalonFX(5, "CantDrive");
  
  public void TalonSetConfiguration() {
    TalonFXConfiguration TalonConfig = new TalonFXConfiguration();
    // TalonFXConfigurator TalonConfigurator = new TalonFXConfigurator(null);
    TalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
    TalonConfig.MotionMagic.MotionMagicAcceleration = 30;
    TalonConfig.MotionMagic.MotionMagicJerk = 1000;

    TalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    TalonConfig.CurrentLimits.StatorCurrentLimit = 20;

    TalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    TalonConfig.CurrentLimits.SupplyCurrentLimit = 20;

    TalonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    TalonConfig.CurrentLimits.SupplyCurrentLowerLimit = 0;

    TalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs slot0Config = TalonConfig.Slot0;
    slot0Config.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Config.kS = 0.3; // Add 0.25 V output to overcome static friction
    slot0Config.kV = 1.5; // A velocity target of 1 rps results in 0.12 V output
    slot0Config.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Config.kP = 15; // A position error of 2.5 rotations results in 12 V output
    slot0Config.kI = 0.0; // no output for integrated error
    slot0Config.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Config.kG = 0.2;

    TalonConfig.Slot0 = slot0Config;
    armMotor.getConfigurator().apply(TalonConfig);
  }

  public double getEncoderDistance() {
  return armMotor.getPosition().getValueAsDouble();
  }

  /** Creates a new ExampleSubsystem. */
  public Arm() {
    TalonSetConfiguration();
    // resetEncoders();
    
  }

  public static Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  public void resetEncoders() {
    // armMotor.setControl(new VoltageOut(-0.5));
    armMotor.setControl(new VoltageOut(0));
    armMotor.setPosition(0);
    targetPoint = 0;
  }

  public void runMotor() {
    // System.out.println(targetPoint); 
    armMotor.setControl(request.withPosition(targetPoint));

    Logger.recordOutput("Arm/motion/Pos", request.withPosition(targetPoint).getPositionMeasure());
    Logger.recordOutput("Arm/voltageMotionMagicCalculated", armMotor.getMotorVoltage().getValueAsDouble());
  }

  public void goToPosition(double point) {
    targetPoint = point;
  }

  public void resetTimer() {
    VelocityTimer.start();
    VelocityTimer.reset();
    AutoZero = true;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/TargetPoint", targetPoint);
    // armMotor.setControl(new VoltageOut(1));
    double armMotorVelocity = armMotor.getVelocity().getValueAsDouble();
    Logger.recordOutput("Arm/Motor/velocity", armMotorVelocity);

    
    Logger.recordOutput("Arm/Timer", VelocityTimer.get());
    Logger.recordOutput("Arm/AutoZero", AutoZero);

    if (AutoZero) {
      armMotor.setControl(new VoltageOut(-0.3));
      if (VelocityTimer.get() >=0.2) {
        if (Math.abs(armMotor.getVelocity().getValueAsDouble()) <= 0.2) {
          resetEncoders();
          AutoZero = false;
        }
      }
    }
    else {
      runMotor();
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
