// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logger.Logable;

public class ExampleSubsystem extends SubsystemBase implements Logable {

  // Singleton pattern
  private static ExampleSubsystem exampleInstance = null;
  public static ExampleSubsystem getInstance() {
    if(exampleInstance == null){
      exampleInstance = new ExampleSubsystem();
    }
    return exampleInstance;
  }

  public class PeriodicIo extends LogData {
    public double test = 0;
  }

  PeriodicIo io;

  /** Creates a new ExampleSubsystem. */
  private ExampleSubsystem() {
    io = new PeriodicIo();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public LogData getLogger() {
    return io;
  }
}
