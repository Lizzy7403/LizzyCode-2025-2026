// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralHandler extends Command {

  private final Arm arm;

  private final double position;
  private final double speed;

  /** Creates a new Up. */
  public coralHandler(Arm arm, double position,double speed) {
    this.arm = arm;
    this.position = position;
    this.speed = speed;
    addRequirements(this.arm);    

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    arm.setPosition(position,speed);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if(interrupted){
      arm.stop();
  }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(arm.getArmEncoderPosition()-position)<10){
            
      return true;

  }
  return false;
 // This command completes immediately after setting the speed
  }
}
