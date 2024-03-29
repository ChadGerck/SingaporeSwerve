// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  /** Creates a new Wait. */
  private double time;
  public Wait(double sec) {
    time = sec;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  double t_s = 0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t_s = System.currentTimeMillis()/1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()/1000.0-t_s > time);
  }
}
