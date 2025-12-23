// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Astin doing a test on main

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ArmSubsystem extends SubsystemBase {

    private static ArmSubsystem instance;
    static TalonFX armMotor = new TalonFX(5, "CantDrive");
    private double currentPos = 0;
    //private VoltageOut request = new VoltageOut(1);
    private MotionMagicVoltage request = new MotionMagicVoltage(0);//.withSlot(0);
    private double setpoint = 0;
    public boolean isJoyous; //when it is controlled by joystick
    private double speed; //a value fed directly to the motor
    private double customPoint; //a user-defined custom point for travelling

    public static void setConfig() {
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    Slot0Configs slot0 = armConfig.Slot0;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.CurrentLimits.StatorCurrentLimit = 20;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 20;
    armConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    armConfig.CurrentLimits.SupplyCurrentLowerLimit = 10;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
    armConfig.MotionMagic.MotionMagicAcceleration = 30;
    armConfig.MotionMagic.MotionMagicJerk = 1000;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = 0.3;
    slot0.kV = 1.5;
    slot0.kA = 0;
    slot0.kP = 15;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kG = 0.2;

    armConfig.Slot0 = slot0;
    armMotor.getConfigurator().apply(armConfig);
  }

  public ArmSubsystem() {
    armMotor.setPosition(0);
    armMotor.setVoltage(0);
    setConfig();
  }
  public void resetEncoder() {
    armMotor.setPosition(0);
    customPoint = 0.5; // random
  }
  public void setTarget(double targetPos) {
    //setting the target for magicMotion to travel to
    setpoint = targetPos;
  }

  public void setVoltage(double volts) {
    // specifically for controlling the arm via joystick
    Logger.recordOutput("TankDrive/ArmSubsystem/Input/Volts", volts);
    if ((currentPos < 1 && volts < 0) || (currentPos > 18 && volts > 0)) {
      if (Math.abs(volts) < 0.2) {
        speed = volts*2;
        Logger.recordOutput("TankDrive/ArmSubsystem/Input/Odd", 1);
      } else {
        speed = 0;
      }
    } else {
      speed = volts;
      Logger.recordOutput("TankDrive/ArmSubsystem/Input/Odd", -1);
    }
    Logger.recordOutput("TankDrive/ArmSubsystem/Input/Speed", speed);
  }

  public double getEncoderDistance() {
    return armMotor.getPosition().getValueAsDouble();
  }
  
  public void setCustomPoint() {
    //for setting the custom point
    if (isJoyous) {
      //when controlled by joystick, press Y to save a position
      customPoint = getEncoderDistance();
    } else {
      //when not controlled by joystick, press Y to travel to saved position.
      setpoint = customPoint;
    }
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) instance = new ArmSubsystem();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //armMotor.setVoltage(1); works
    currentPos = getEncoderDistance();
    if (isJoyous) {
      //when controlled by joystick, use raw voltage value
      armMotor.setControl(new VoltageOut(speed));
    } else {
      //only travel to the set point when not in joystick mode
      armMotor.setControl(request.withPosition(setpoint));
    }
    Logger.recordOutput("TankDrive/ArmSubsystem/Voltage", armMotor.getMotorVoltage().getValueAsDouble());
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
