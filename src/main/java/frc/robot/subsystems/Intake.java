/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private VictorSP arm;
  // private DigitalOutput roller;
  private VictorSP agitator = new VictorSP(IntakeConstants.AGITATOR_PWM);
  private DigitalInput limit_switch_top = new DigitalInput(IntakeConstants.TOP_LIMITSWITCH_DIO);
  private DigitalInput limit_switch_bottom = new DigitalInput(IntakeConstants.BOTTOM_LIMITSWITCH_DIO);
  // private DigitalOutput feeder;
  // private DigitalInput bottomSwitch;
  // private DigitalInput topSwitch;
  public PWMVictorSPX roller;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    arm = new VictorSP(IntakeConstants.ARM_PWM);
    roller = new PWMVictorSPX(IntakeConstants.INTAKE_PWM);
    // roller.enablePWM(0.375);
    // roller.setPWMRate(250);
    // feeder.enablePWM(0.375);
    // feeder.setPWMRate(250);
    roller.setInverted(true);
  }
  public boolean getLimitSwitchTop() {
    return limit_switch_top.get();
  }
  public boolean getLimitSwitchBottom() {
    return limit_switch_bottom.get();
  }
  public void move(double speed) {
    System.out.println("moving arm at " + speed);
    arm.set(speed);
  }
  public void stopArm() {
    System.out.println("stopping arm");
    arm.stopMotor();
  }
  public void in(double speed) {
    roller.setSpeed(speed);
  }
  public void stopRoller() {
    roller.stopMotor();
  }
  public void agitate(double speed) {
    agitator.set(speed);
  }
  public void stopAgitate() {
    agitator.stopMotor();
  }
  public void feed(double speed) {
    // feeder.updateDutyCycle((speed*0.5+1.5)/4);
  }
  // public boolean getBottomSwitch() {
  //   return bottomSwitch.get();
  // }
  // public boolean getTopSwitch() {
  //   return topSwitch.get();
  // }
  @Override
  public void periodic() {
    
  }
}
