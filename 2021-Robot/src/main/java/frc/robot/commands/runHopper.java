// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class runHopper extends CommandBase {
  private double lifterMult = Constants.MotorOutputMultiplier.lifter.multiplier;
  private double agitatorMult = Constants.MotorOutputMultiplier.agitator.multiplier;
  private Hopper hopper;

  /** Creates a new runHopper. */
  public runHopper(Hopper hopper) {
    this.hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Hopper.count % 2 == 1) {
      hopper.sethopperMotor(1 * lifterMult);
      hopper.setAgitatorMotor(1 * agitatorMult);
    } else {
      hopper.sethopperMotor(0);
      hopper.setAgitatorMotor(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
