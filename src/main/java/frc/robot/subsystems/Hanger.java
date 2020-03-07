/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;

public class Hanger extends SubsystemBase {
  private SpeedController hanger;
  private DoubleSolenoid hookSolenoid;
  private Encoder reachEncoder;
  public Hanger() {
    hanger = new Talon(HangerConstants.CLIMBER_CAN); //placeholder speed controller
    hookSolenoid = new DoubleSolenoid(HangerConstants.HANGER_SOLENOID_FORWARD_CHANNEL, HangerConstants.HANGER_SOLENOID_REVERSE_CHANNEL);
    reachEncoder = new Encoder(0, 0);
  }

  @Override
  public void periodic() {
    
  }
  public void climb(double speed) {
    if (speed < 0.01 && speed > -0.01)
      hanger.stopMotor();
    else
      hanger.set(speed);
  }
  public void hook(DoubleSolenoid.Value v) {
    hookSolenoid.set(v);
  }
  public double getEncoder() {
    return reachEncoder.getDistance();
  }
}
