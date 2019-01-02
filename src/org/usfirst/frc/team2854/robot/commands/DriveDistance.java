package org.usfirst.frc.team2854.robot.commands;

import org.usfirst.frc.team2854.robot.Robot;
import org.usfirst.frc.team2854.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2854.robot.subsystems.SubsystemNames;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistance extends Command {
	DriveTrain drive;
	private double distance;
	private boolean isFinished;
	private boolean inErrorZone;
	private int count;
    public DriveDistance(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
    	this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	drive = (DriveTrain) (Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
    	
   
    	isFinished = false;
    	inErrorZone = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double error = drive.turnController.getError();
    	inErrorZone = Math.abs(error)  < 2;
    	if(inErrorZone) {
    		count++;
    		isFinished = count >= 5; //if in error zone for 5 roborio ticks --> finished
    	}else {
    		count = 0;
    	}
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
