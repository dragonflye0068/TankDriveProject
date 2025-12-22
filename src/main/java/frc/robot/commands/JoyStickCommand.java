// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoyStickCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  double grandfathersLeftVoltage = 0;
  double grandfathersRightVoltage = 0;
  private final PIDController speedsterPID = new PIDController(.2,0,0);
  private final LoggedNetworkNumber myConstant = new LoggedNetworkNumber("A constant", 0.0);
  private double targetL;
  private double targetR;
  //USE 10% SPEED WHILE DRIVING ON TABLE
  //USE 15% SPEED WHILE DRIVING ON GROUND

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public JoyStickCommand(Drivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public void setSpeed(double lX, double lY, double rX, double rY) {
    targetL = -1*lY-lX;
    targetR = -1*lY+lX;
    if (Math.abs(targetL) < 0.1) {
      targetL = 0;
    }
    if (Math.abs(targetR) < 0.1) {
      targetR = 0;
    }
    Logger.recordOutput("TankDrive/JoyStickCommand/Speed/TargLeft", targetL);
    Logger.recordOutput("TankDrive/JoyStickCommand/Speed/TargRight", targetR);
    double newL = MathUtil.clamp(grandfathersLeftVoltage + speedsterPID.calculate(targetL), -0.7, 0.7);
    double newR = MathUtil.clamp(grandfathersRightVoltage + speedsterPID.calculate(targetR), -0.7, 0.7);
    m_subsystem.runMotor(newL, newR);
    grandfathersLeftVoltage = m_subsystem.leftSpee;
    grandfathersRightVoltage = m_subsystem.rightSpee;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
