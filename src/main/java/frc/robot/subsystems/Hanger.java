/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {
  private TalonSRX hanger;
  private DoubleSolenoid banger;
  public Hanger() {
    hanger = new TalonSRX(HangerConstants.CLIMBER_CAN);
    banger = new DoubleSolenoid(HangerConstants.HANGER_SOLENOID_FORWARD_CHANNEL, HangerConstants.HANGER_SOLENOID_REVERSE_CHANNEL);
  }

  @Override
  public void periodic() {
    
  }
  public void climb(double speed) {
    
  }
  public void hook(DoubleSolenoid.Value v) {
    banger.set(v);
  }
}
