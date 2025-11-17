// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Astin doing a test on main

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleConsumer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  private static SparkMaxConfig config = new SparkMaxConfig();

  private static final double kCountsPerRevolution = 42.0; //check if really trustworthy
  private static final double kWheelDiameterCentimetre = 15.0; //very painfully calculated

  //left1 id 1
  final SparkMax leftMotor1 = new SparkMax(1, MotorType.kBrushless);
  final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  //left2 id 4
  final SparkMax leftMotor2 = new SparkMax(4, MotorType.kBrushless);
  final RelativeEncoder leftEncoder2 = leftMotor2.getEncoder();
  //right1 id 2
  final SparkMax rightMotor1 = new SparkMax(2, MotorType.kBrushless);
  final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  //right2 id 3
  final SparkMax rightMotor2 = new SparkMax(3, MotorType.kBrushless);
  final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    config.inverted(false);
    leftMotor1.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor2.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    rightMotor1.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor2.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getLeftDistance() {
    //shoot this will not work lmao
    return leftEncoder1.getPosition() * ((Math.PI * kWheelDiameterCentimetre) / kCountsPerRevolution);
  }

  public double getRightDistance() {
    return rightEncoder1.getPosition() * ((Math.PI * kWheelDiameterCentimetre) / kCountsPerRevolution);
  }

  public void resetEncoders() {
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);
  }

  public void runMotor(double leftSpeed, double rightSpeed) {
    //0 to 12
    //clamped speed. may be changed

    //limit speed to 12 * 0.1 (10% of 12)
    leftSpeed = MathUtil.clamp(leftSpeed, -1.2, 1.2);
    rightSpeed = MathUtil.clamp(rightSpeed, -1.2, 1.2);

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
