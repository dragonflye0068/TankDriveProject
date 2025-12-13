// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class TeleopCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  //USE 10% SPEED WHILE DRIVING ON TABLE
  //USE 15% SPEED WHILE DRIVING ON GROUND

  private CommandXboxController m_driverController;
  // private CommandPS4Controller m_driverController;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopCommand(Drivetrain subsystem, CommandXboxController m_drivercController) {
  // public TeleopCommand(Drivetrain subsystem, CommandPS4Controller m_drivercController) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.m_driverController = m_drivercController;

  }
    //TODO Auto-generated constructor stub

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double joystick_lefty = m_driverController.getLeftY() * m_subsystem.maxspeed/2;
    double joystick_rightx = m_driverController.getLeftX() * m_subsystem.maxspeed/2;

    if (Math.abs(joystick_rightx) < 0.03) {
      joystick_rightx = 0;
    }

    double speedleft = joystick_lefty + (2*MathUtil.clamp(joystick_rightx, 0, 1)*(joystick_lefty/Math.abs(joystick_lefty)));
    double speedright = joystick_lefty + (-2*MathUtil.clamp(joystick_rightx, -1, 0)*(joystick_lefty/Math.abs(joystick_lefty)));
    
    if (Math.abs(joystick_lefty)  > 0.03) {
      m_subsystem.runMotor(speedleft, speedright);
    } else {
      m_subsystem.runMotor(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.runMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
