/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import com.kauailabs.navx.frc.AHRS;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Constants for the Control System of the Robot
     * 
     */
    public static final class ContainerConstants{
        public static final int JOYSTICK_USB = 0;
        
        public static final Joystick JOYSTICK = new Joystick(JOYSTICK_USB);
    }
    
    /**
     * Constants for the DriveTrain Subsystem
     * 
     */
    public static final class DriveConstants {
        public static final int DRIVE_PWM_RIGHT_BACK = 0;
        public static final int DRIVE_PWM_RIGHT_FRONT = 1;
        public static final int DRIVE_PWM_LEFT_BACK = 2;
        public static final int DRIVE_PWM_LEFT_FRONT = 3;
    }
    public static final class TurretConstants {
        public static final int TURRET_PWM_LEFT = 4;
        public static final int TURRET_PWM_RIGHT = 5;
        public static final int SHOOTER_CAN_LEFT = 1;
        public static final int SHOOTER_CAN_RIGHT = 2;
        public static final int TURRET_ROTATE_UP_BUTTON = 7;
        public static final int TURRET_ROTATE_DOWN_BUTTON = 9;
        // public static final int SUSAN_PWM = 6;
        public static final int TURRETGETPWM = 8;
        public static final int WINDOW_PWM_MOTOR = 9;
        public static final int ELEVATOR_PWM_MOTOR = 8;
    }
    public static final class EncoderConstants {
        // public static final int ENCODER_1_DIO1 = 2;
        // public static final int ENCODER_1_DIO2 = 3;
        // public static final int ENCODER_2_DIO1 = 0;
        // public static final int ENCODER_2_DIO2 = 1;
        public static final int SHOOTER_ENCODER_3_PORT = 0;// analog input port 0
        public static final double DISTANCE_PER_PULSE = 0.5*Math.PI/1024.;
    }
    
    public static final class ControlPanelArmConstants {

        /* Colors as read by the sensor in the lighting conditions of the shop */
        public static final Color BLUE = new Color(0.2,0.5,0.3);
        public static final Color GREEN = new Color(0.25,0.6,0.2);
        public static final Color RED = new Color(0.6,0.3,0.05);
        public static final Color YELLOW = new Color(0.42,0.5,0.05);

        /* Ideal Colors */
        public static final Color IDEAL_BLUE = new Color(0, 1, 1);
        public static final Color IDEAL_GREEN = new Color(0, 1, 0);
        public static final Color IDEAL_RED = new Color(1, 0, 0);
        public static final Color IDEAL_YELLOW = new Color(1, 1, 0);

        /* ARM PWMS */
        public static final int ARM_MOTOR_CAN = 1;

        /* i2C port */
        public static final I2C.Port I2C_PORT = I2C.Port.kOnboard;

        /* PCM ports */
        public static final int ARM_SOLENOID_FORWARD_CHANNEL = 0;
        public static final int ARM_SOLENOID_REVERSE_CHANNEL = 1;
        /*Button Mapping*/
        public static final int ARM_FWD_BUTTON = 5;
        public static final int ARM_REVERSE_BUTTON = 3;
        public static final int ROT_CONTROL_BUTTON = 12;
        public static final int POS_CONTROL_BUTTON = 11;
    }

    public static final class IntakeConstants {
        public static final int ROLLER_CHANNEL = 6;
        public static final int INTAKE_UP = 8;
        public static final int DROPPER_CHANNEL = 5;
        public static final int AGITATOR_CHANNEL = 4;
        public static final int FEEDER_CHANNEL = 00;

        public static final int BOTTOM_CHANNEL = 00;
        public static final int TOP_CHANNEL = 00;

        public static final int INTAKE_BUTTON = 12;
    }
    
    public static final class HangerConstants {
        public static final int CLIMBER_CAN = 2;
        public static final int CLIMB_UP_BUTTON = 6;
        public static final int CLIMB_DOWN_BUTTON = 4;
        /*PCM PORTS*/
        public static final int HANGER_SOLENOID_FORWARD_CHANNEL = 2;
        public static final int HANGER_SOLENOID_REVERSE_CHANNEL = 3;
    }

	public static final class NAVXConstants {
        public static final AHRS NAVX = new AHRS();
        double P = 0;
        double I = 0;
        double D = 0;
    }
    public static final int PCM_CAN = 3;
}