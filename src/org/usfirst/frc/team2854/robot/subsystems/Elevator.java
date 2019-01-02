package org.usfirst.frc.team2854.robot.subsystems;

import org.usfirst.frc.team2854.robot.RobotMap;
import org.usfirst.frc.team2854.robot.commands.JoystickCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem implements PIDOutput{
	private TalonSRX elevatorTalon;
	private Encoder encoder;
	public final PIDController pidControl;
	
	
	private int P, I, D = 1;
	private double integral, error, derivative, previous_error, setpoint, rcw = 0;
	;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public Elevator() {
		
		elevatorTalon = new TalonSRX(RobotMap.elevatorTalon1);
		encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		pidControl = new PIDController(P, I, D, encoder, this);
		pidControl.setInputRange(0 , 1000);
		pidControl.setAbsoluteTolerance(2.0f);
		pidControl.setOutputRange(0, 500);
		pidControl.setContinuous(false);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new JoystickCommand());
    }
    public void setPosition(double height) {
    	encoder.reset();
    	pidControl.reset();
    	pidControl.setPID(P, I, D);
    	pidControl.setSetpoint(height);
    	pidControl.enable();
    }

	public void set(ControlMode mode, double speed) {
		elevatorTalon.set(ControlMode.PercentOutput, speed);
		
	
    
}
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		set(ControlMode.PercentOutput, output);
		
	}
	
		
	
}

