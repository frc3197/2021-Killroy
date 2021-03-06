// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

/** Add your docs here. */
public class AutoConstants {
    public final static double maxSpeed = Units.feetToMeters(2);
    public final static double maxAcceleration = Units.feetToMeters(1);
    public final static double maxAngleSpeed = 5 * Math.PI;    
    public final static double maxAngleAcceleration = 2 * Math.PI;


    public static enum thetaPIDConstants{
        // .45 , 0 , .006
        kP(-.45),kI(0.0),kD(0.6);


        public final double constant;

        private thetaPIDConstants(double constant){
            this.constant = constant;
        }
    }

    public static enum xPIDConstants{
        kP(.31),kI(0.0),kD(0.006);


        public final double constant;

        private xPIDConstants(double constant){
            this.constant = constant;
        }
    }

    public static enum yPIDConstants{
        kP(-10),kI(0.0),kD(0.006);


        public final double constant;

        private yPIDConstants(double constant){
            this.constant = constant;
        }
    }




}
