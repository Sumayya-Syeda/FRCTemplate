package org.usfirst.frc.team2854.robot.commands;

import org.usfirst.frc.team2854.robot.OI;
import org.usfirst.frc.team2854.robot.Robot;
import org.usfirst.frc.team2854.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2854.robot.subsystems.SubsystemNames;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickCommand extends Command {
	DriveTrain drive;

	public JoystickCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		drive = (DriveTrain) (Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
		double throttle = OI.joystick.getRawAxis(3);
		throttle = Math.abs(throttle) < .1 ? 0 : throttle;



		double turn = OI.joystick.getRawAxis(0);
		turn = Math.abs(turn) < .05 ? 0 : turn;
		if (OI.joystick.getRawButton(3)) {
			turn /= 2d;
		}
		// turn *= -(1+Robot.getSensors().getNavX().getVelocityX());
		drive.drive(ControlMode.PercentOutput, sig(throttle - cubeRoot(turn)), sig(throttle + cubeRoot(turn)));
	}
	/*ask how next two methods are calculated*/

	public double cubeRoot(double val) {
		if(val >= 0) {
			return Math.pow(val,  3/2d);
		} else {
			return -Math.pow(-val, 3/2d);
		}
	}

	public double sig(double val) {
		return 2/(1 + Math.pow(Math.E, -7 * Math.pow(val, 3))) - 1;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		drive.drive(ControlMode.PercentOutput, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
