// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shooterAlign extends PIDCommand {
  /** Creates a new shooterAlign. */
  Shooter shooter;
  SwerveDrive swerve;
  public shooterAlign(Shooter shooter, SwerveDrive swerve) { 
    super(
        // The controller that the command will use
        // TODO: Set Proper Constant Values: Shooter PID Align
        new PIDController(.0095, 0, 0.001), 
        // This should return the measurement
        shooter::getXOffset,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          swerve.drive(0, 0, -.1 * output, true);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(shooter, swerve);
    getController().setTolerance(3);;
    this.shooter = shooter;
    this.swerve = swerve;
  }

  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

}
