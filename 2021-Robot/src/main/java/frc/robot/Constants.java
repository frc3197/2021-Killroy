// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum TalonID {
        // Module 1: Speed 0, Angle 1, Encoder 0 - Back Right
        // Module 2: Speed 2, Angle 3, Encoder 1 - Back Left
        // Module 3: Speed 4, Angle 5, Encoder 2 - Front Left
        // Module 4: Speed 6, Angle 7, Encoder 3 - Front Right
        kSwerveBLAngle(3, "BackLeftAngle"), kSwerveBLSpeed(2, "BackLeftSpeed"), kSwerveBRSpeed(0, "BackRightSpeed"),
        kSwerveBRAngle(1, "BackRightAngle"), kSwerveFRAngle(7, "FrontRightAngle"), kSwerveFLAngle(5, "FrontLeftAngle"),
        kSwerveFRSpeed(6, "FrontRightSpeed"), kSwerveFLSpeed(4, "FrontLeftSpeed"), kLifterMotor(8, "LifterMotor");

        public final int id;
        public final String name;

        private TalonID(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }

    public static enum CANDevices {
        kCANCoderBL(1, "CANCoderBackLeft"), kCANCoderBR(0, "CANCoderBackRight"), kCANCoderFL(2, "CANCoderFrontLeft"),
        kCANCoderFR(3, "CANCoderFrontRight");

        public final int id;
        public final String name;

        private CANDevices(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }

    public static final double L = 23.75;
    public static final double W = 24.75;
    public static final int SWERVE_MAX_VOLTS = 0;

    public static final int talonEncoderResolution = 2048;
    public static final double swerveWheelDiam = Units.inchesToMeters(4);
    public static final double swerveDriveMotorGR = 6.86;
    // This could be 6 or 6.2
	public static final double angleFeedForwardkV = 0;
    public static int intakeMotor = 10;

    public static final int beamBreakInput = 0;
    public static final int beamBreakOutput = 1;

    public static enum CANSparkMaxID {
        agitatorMotor("agitatorMotor",0),intakeMotor("intakeMotor",1);

        public final String name;
        public final int id;

        private CANSparkMaxID(String name, int id){
            this.name = name;
            this.id = id;
        }
    }


    public static enum PIDContants {
        swerveModule("swerveModules", .042, .003, 0, 0);


        public final String name;
        public final double p;
        public final double i;
        public final double d;
        public final double f;

        private PIDContants(String name, double p, double i, double d, double f) {
            this.name = name;
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        }
    }
    public static class DriveConstants{
        // Module 1: Speed 0, Angle 1, Encoder 0 - Back Right
// Module 2: Speed 2, Angle 3, Encoder 1 - Back Left
// Module 3: Speed 4, Angle 5, Encoder 2 - Front Left
// Module 4: Speed 6, Angle 7, Encoder 3 - Front Right    
//0.0122631851627                          
public final static SwerveModuleConstants frontRightConstants = new SwerveModuleConstants

(7, 6, 3, 
.07,  0.0122631851627   , .0325, 0.00,
 0.4, 0.0190285356420, 0, 0);

public final static SwerveModuleConstants backRightConstants = new SwerveModuleConstants
(1, 0, 0,
.07,0.0122631851627   , .0325, 0.00, 
 0.4, 0.0174386190803, 0, 0);

public final static SwerveModuleConstants frontLeftConstants = new SwerveModuleConstants
(5, 4, 2,
.07,  0.0122631851627   , .0325, 0.00,
 0.4, 0.0190626376654, 0, 0);

public final static SwerveModuleConstants backLeftConstants = new SwerveModuleConstants
(3, 2, 1, 
.07,  0.0122631851627 , .0325, 0.00,
 0.4, 0.0165160964636, 0, 0);


}
}