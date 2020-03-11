/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanelarm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelArm;
import edu.wpi.first.wpilibj.DriverStation;

public class ControlPanelPosCtrl extends CommandBase {
  
  private ControlPanelArm arm;
  private char desiredColor;
  private boolean colorReached;
  /**
   * Creates a new ControlPanelPositionControl.
   */
  public ControlPanelPosCtrl(ControlPanelArm cpa) {
    System.out.println("starting ControlPanelPosCtrl");
    addRequirements(cpa);
    arm = cpa;
    desiredColor = '?';
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RUN COMMAND... COOL..");
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
      colorReached = false;
    }else{
      colorReached = true;
    }
    System.out.println("GameData: " + gameData);
    switch (gameData.charAt(0)) {
      case 'R':
        desiredColor = 'B';
        break;
      case 'G':
        desiredColor = 'Y';
        break;
      case 'B':
        desiredColor = 'R';
        break;
      case 'Y':
        desiredColor = 'G';
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(desiredColor == '?'){
      return;
    }
    if(arm.detectColor() == desiredColor){
      colorReached = true;
      arm.stopSpin();
      System.out.println("color read is desired!");
    }
    else{
      arm.spin(0.18);
      System.out.println("color read is not desiredColor. spinning");
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopSpin();
    System.out.println("stopping");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return colorReached;
  }
}
