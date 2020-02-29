package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
public class Vision extends SubsystemBase {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tv = table.getEntry("tv");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    private static double vTurret = 1;
    private static double targetHeight = 9.4375;
    private static double turretHeight = 00;
    private static double gravity = 32.2;

    AnalogInput sonar = new AnalogInput(3);
    public Vision() {

    } 
    public double getDistanceEstimation() {
        return sonar.getValue()/13.888888;
    }
    
    
    //returns a value for the motor to run at in order to pivot to the target.
    public double pivotToTarget(double topSpeed, double slowSpeed, double threshold) {
        if (getTX() != 0.0) {
            if (getTX() > 1.0) {
                if (getTX() > threshold) {
                    return topSpeed;

                }else {
                    return slowSpeed;
                }
            }else if (getTX() < -1.0) {
                if (getTX() < -threshold) {
                    return -topSpeed;
                }else {
                    return -slowSpeed;
                }
            }
        }
        return 0.0;

    }
    public double getTX() {
        return tx.getDouble(0.0);
    }
    public double getTY() {
        return ty.getDouble(0.0);
    }

    public boolean getVisible() {
        return tv.getBoolean(false);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    @Override
    public void periodic() {
        
    }

}
