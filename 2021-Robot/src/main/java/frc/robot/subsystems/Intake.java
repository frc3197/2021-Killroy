// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  
  PhotonCamera camera = new PhotonCamera("intakeCam");
  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new WPI_TalonFX(Constants.intakeMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public PhotonCamera getCam(){
    return camera;
  }

  public void setIntake(double speed){

    intakeMotor.set(speed);

  }

  public double getYaw(){
    var result = camera.getLatestResult();
    double output = 0;
    if(result.hasTargets()){
      output = result.getBestTarget().getYaw();
    }
    return output;
  }
}
