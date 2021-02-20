// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  WPI_TalonFX hoodMotor;
  /** Creates a new Hood. */
  public Hood(int hoodMotorID) {
    hoodMotor = new WPI_TalonFX(hoodMotorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHood(double speed){
    hoodMotor.set(speed);
  }

}
