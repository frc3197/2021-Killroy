// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechs.Button;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class runIntakeHopper extends CommandBase {
  Hopper hopper;
  Intake intake;
  double input;
  
  /** Creates a new runIntake. */
  public runIntakeHopper(Intake intake,Hopper hopper, double input) {
   this.intake = intake;
   this.hopper = hopper;
   this.input = input;
   addRequirements(intake,hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean beamBreak = BeamBreak.getBeamBreakState();
    SmartDashboard.putBoolean("beamBreak", beamBreak);
    intake.setIntake(input * Constants.MotorOutputMultiplier.intake.multiplier);
    if(beamBreak){
        hopper.sethopperMotor(1 * Constants.MotorOutputMultiplier.lifter.multiplier);
    }
      else{
        hopper.sethopperMotor(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
    hopper.sethopperMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}