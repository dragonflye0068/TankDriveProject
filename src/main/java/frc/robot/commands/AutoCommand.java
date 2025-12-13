// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  //USE 10% SPEED WHILE DRIVING ON TABLE
  //USE 15% SPEED WHILE DRIVING ON GROUND

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final PIDController pidController = new PIDController(1, 0, 0);

  private double distance;

  public AutoCommand(Drivetrain subsystem, double moveDistance) {
    m_subsystem = subsystem;
    distance = moveDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RemainingDistance = distance - Math.abs(m_subsystem.getEncoderDistance());
  
    // System.out.println("distance: " + distance);
    // System.out.println(m_subsystem.getEncoderDistance());

    double speed = pidController.calculate(RemainingDistance);
    
    // System.out.println(RemainingDistance);
    m_subsystem.runMotor(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.runMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance - Math.abs(m_subsystem.getEncoderDistance()) < 0.5;
  }
}
