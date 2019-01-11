/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drivebase extends Subsystem {

  private TalonSRX leftMotor;
  private TalonSRX rightMotor;
  private TalonSRX leftSlave;
  private TalonSRX rightSlave;

  public Drivebase() {

    leftMotor = new TalonSRX(RobotMap.LEFT_MOTOR.value);
    rightMotor = new TalonSRX(RobotMap.RIGHT_MOTOR.value);
    leftSlave = new TalonSRX(RobotMap.LEFT_SLAVE.value);
    rightSlave = new TalonSRX(RobotMap.RIGHT_SLAVE.value);

    Robot.initTalon(leftMotor);
    Robot.initTalon(rightMotor);
    Robot.initTalon(leftSlave);
    Robot.initTalon(rightSlave);

    leftSlave.follow(leftMotor);
    rightSlave.follow(rightMotor);

  }

  public void tankDrive(ControlMode mode, double leftValue, double rightValue) {

    leftMotor.set(mode, -leftValue);
    rightMotor.set(mode, rightValue);

  }

  @Override
  public void initDefaultCommand() {

    setDefaultCommand(new TankDrive());

  }
}
