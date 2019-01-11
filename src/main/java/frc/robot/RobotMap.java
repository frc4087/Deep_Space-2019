/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public enum RobotMap {
  
  //Drivetrain Mappings
	LEFT_MOTOR(5),
	RIGHT_MOTOR(1),
	LEFT_SLAVE(6),
	RIGHT_SLAVE(2),
	//Winch Mappings
	WINCH_MASTER(3),
	WINCH_SLAVE(7),
	//Wrist Mappings
	WRIST(9),
	//Control Mappings
	DRIVE_JOYSTICK(0),
	CONTROL_JOYSTICK(1);
	
	public final int value;

	RobotMap(int value) {
		this.value = value;
  }

}
