// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public abstract class OI {

    //Sets up both controllers
    static CommandXboxController driverJoystick = new CommandXboxController(0); 
    static CommandXboxController operatorJoystick = new CommandXboxController(1);

    static public double getDriverJoystickLeftX() {
        return driverJoystick.getLeftX();
    }

    static public double getDriverJoystickLeftY() {
        return driverJoystick.getLeftY();
    }

    static public double getDriverJoystickRightX() {
        return driverJoystick.getRightX();
    }










}
