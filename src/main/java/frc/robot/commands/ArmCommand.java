// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private double targetPos = 0;
  //USE 10% SPEED WHILE DRIVING ON TABLE
  //USE 15% SPEED WHILE DRIVING ON GROUND

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public ArmCommand(double targetPos, ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    this.targetPos = targetPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public void setTarget(double targetPos) {
    Logger.recordOutput("TankDrive/ArmCommand/Speed/Position", targetPos);
    m_subsystem.setTarget(targetPos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setTarget(targetPos);
    Logger.recordOutput("TankDrive/ArmCommand/Position", targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
