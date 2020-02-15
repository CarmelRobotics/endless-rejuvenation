package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class Vision extends SubsystemBase {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tv = table.getEntry("tv");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");

    public Vision() {

    }
    public double getDistanceEstimation(double angleOfCamera, double heightOfCamera) {
        return (98.25-heightOfCamera)/Math.tan(angleOfCamera+getTY());
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
