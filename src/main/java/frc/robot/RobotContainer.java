// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_Drivetrain = Drivetrain.getInstance();
  public final ArmSubsystem m_arm = ArmSubsystem.getInstance();
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  private final AutoCommand m_autoCommand = new AutoCommand(m_Drivetrain, 20);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //m_Drivetrain.setDefaultCommand(new TeleopCommand(() -> m_driverController.getLeftY() - m_driverController.getRightX(), () -> m_driverController.getLeftY() - m_driverController.getRightX()));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(new ArmCommand(9.4, m_arm));
    m_driverController.b().onTrue(new ArmCommand(1, m_arm));
    m_driverController.x().onTrue(new ArmCommand(18, m_arm));
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    m_Drivetrain.leftEncoder1.setPosition(0);
    
    return m_autoCommand;
  }
}
