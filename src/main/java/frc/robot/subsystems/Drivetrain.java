// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Astin doing a test on main

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import java.util.function.DoubleConsumer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  private static SparkMaxConfig configLeft = new SparkMaxConfig();
  private static SparkMaxConfig configRight = new SparkMaxConfig();

  private static final int kCountsPerRevolution = 42;
  private static final double kWheelDiameterCentimetre = 15.0; //very painfully calculated

  // private final PIDController velocityPidController = new PIDController(1, 0, 0);

  //DifferentialDrive method, extreme pain!
  //create two PID controllers: one for left and one for right

  //First motor, Left 1, kBrushless ID 1, SparkMax
  final SparkMax leftMotor1 = new SparkMax(1, MotorType.kBrushless);
  final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  
  //Second motor, Left 2, kBrushless ID 4, SparkMax
  final SparkMax leftMotor2 = new SparkMax(4, MotorType.kBrushless);
  final RelativeEncoder leftEncoder2 = leftMotor2.getEncoder();
  
  //Third motor, Right 1, kBrushless ID 2, SparkMax
  final SparkMax rightMotor1 = new SparkMax(2, MotorType.kBrushless);
  final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  
  //Fourth motor, Right 2, kBrushless ID 3, SparkMax
  final SparkMax rightMotor2 = new SparkMax(3, MotorType.kBrushless);
  final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  //DifferentialDrive m_DifferentialDrive = new DifferentialDrive(leftMotor1::setVoltage, rightMotor1::setVoltage);

  PIDController leftVelocityPIDController = new PIDController(0.1, 0.0032, 0.3);
  PIDController rightVelocityPIDController = new PIDController(0.1, 0.0032, 0.3);

  public double getEncoderDistance() {
    return leftEncoder1.getPosition();
  }

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {

    configLeft.inverted(false);
    leftMotor1.configure(configLeft, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configLeft.follow(leftMotor1);
    leftMotor2.configure(configLeft, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    configRight.inverted(true);
    rightMotor1.configure(configRight, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configRight.follow(rightMotor1); //it works maybe but it's not neccesary because we set it later. If you change it can you do it hear and not at home.
    rightMotor2.configure(configRight, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //ALL MOTORS FOLLOW MOTOR1
    //ALL ROADS LEAD TO ROME
  }

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
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

  //updatePIDs method

  //arcadeDrive method

  public void runMotor(double xaxisSpeed, double zaxisRotate) {

    leftVelocityPIDController.setSetpoint(xaxisSpeed - zaxisRotate);
    rightVelocityPIDController.setSetpoint(xaxisSpeed + zaxisRotate);

    double leftSpeed = leftVelocityPIDController.calculate(leftEncoder1.getVelocity());
    double rightSpeed = rightVelocityPIDController.calculate(rightEncoder1.getVelocity());

    leftMotor1.setVoltage(leftSpeed);
    rightMotor1.setVoltage(rightSpeed);
    
    //m_DifferentialDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
