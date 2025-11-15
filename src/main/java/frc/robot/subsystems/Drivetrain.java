// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  private static SparkMaxConfig config = new SparkMaxConfig();

  //left1 id 1
  final SparkMax leftMotor1 = new SparkMax(1, MotorType.kBrushless);
  //left2 id 4
  final SparkMax leftMotor2 = new SparkMax(4, MotorType.kBrushless);
  //right1 id 2
  final SparkMax rightMotor1 = new SparkMax(2, MotorType.kBrushless);
  //right2 id 3
  final SparkMax rightMotor2 = new SparkMax(3, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    config.inverted(false);
    leftMotor1.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor2.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    rightMotor1.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor2.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(double leftSpeed, double rightSpeed) {
    //0 to 12
    //clamped speed. may be changed
    MathUtil.clamp(leftSpeed, -leftSpeed * 0.1, leftSpeed * 0.1);
    MathUtil.clamp(rightSpeed, -rightSpeed * 0.1, rightSpeed * 0.1);

    leftMotor1.setVoltage(leftSpeed);
    leftMotor2.setVoltage(leftSpeed);

    rightMotor1.setVoltage(rightSpeed);
    rightMotor2.setVoltage(rightSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
