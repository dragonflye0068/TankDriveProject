// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TeleopCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain m_Drivetrain;
  PIDController velocityPidController = new PIDController(0.02, 0.001, 0.3);

  double leftSpeed;
  double rightSpeed;

  //double speed;
  //double rotate;
  //USE 10% SPEED WHILE DRIVING ON TABLE
  //USE 15% SPEED WHILE DRIVING ON GROUND

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
                    /* DoubleSupplier speed, DoubleSupplier rotate */
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain.getInstance());
    m_Drivetrain = Drivetrain.getInstance();

    this.leftSpeed = leftSpeed.getAsDouble();
    this.rightSpeed = rightSpeed.getAsDouble();

    //this.speed = speed.getAsDouble();
    //this.rotate = rotate.getAsDouble();

    //get pid outputs for four 
    
    //velocityPidController.setSetpoint(leftSpeed.getAsDouble());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.runMotor(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.runMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
