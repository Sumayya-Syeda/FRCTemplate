package org.usfirst.frc.team2854.robot.commands;

import org.usfirst.frc.team2854.robot.Robot;
import org.usfirst.frc.team2854.robot.RobotMap;
import org.usfirst.frc.team2854.robot.subsystems.Elevator;
import org.usfirst.frc.team2854.robot.subsystems.SubsystemNames;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorSetPoint extends Command {
	Elevator elevator;
	
	
	Encoder encoder;
	
	private int P, I, D = 1;
	private double integral, error, derivative, previous_error, setpoint, rcw = 0;
	
    public ElevatorSetPoint() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.getSubsystem(SubsystemNames.ELEVATOR));
    	encoder =  new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	elevator = (Elevator)( Robot.getSubsystem(SubsystemNames.ELEVATOR));
    }
    public void PID() {
		error = setpoint - encoder.get();
		this.integral += (error *.02);
		derivative = (error - this.previous_error) / 0.02;
		this.rcw = P*error + I * this.integral + D *derivative;
	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PID();
    	elevator.set(ControlMode.PercentOutput, rcw);
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
    
    public void setSetpoint(int setPoint) {
		this.setpoint = setPoint;
		
	}
	
	

}
