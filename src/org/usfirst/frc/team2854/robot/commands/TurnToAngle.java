package org.usfirst.frc.team2854.robot.commands;

import org.usfirst.frc.team2854.robot.Robot;
import org.usfirst.frc.team2854.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2854.robot.subsystems.SubsystemNames;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command {
	DriveTrain drive;
	private double angle;
	private boolean isFinished;
	private boolean inErrorZone;
	private int count;
	
    public TurnToAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	drive = (DriveTrain) (Robot.getSubsystem(SubsystemNames.DRIVE_TRAIN));
    	
    	drive.rotateDegrees(angle);
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
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	drive.turnController.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
