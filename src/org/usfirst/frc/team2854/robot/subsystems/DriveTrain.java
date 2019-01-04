package org.usfirst.frc.team2854.robot.subsystems;



import org.usfirst.frc.team2854.robot.PIDTemplate;
import org.usfirst.frc.team2854.robot.RobotMap;
import org.usfirst.frc.team2854.robot.commands.JoystickCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem implements PIDOutput {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private TalonSRX leftTalon1;
	private TalonSRX leftTalon2;
	private TalonSRX rightTalon1;
	private TalonSRX rightTalon2;

	private final AHRS ahrs;

	public final PIDController turnController;
	
	private static Value SLOW, FAST, UNKNOWN;
	private static Value gear;
	
/*would be a good idea to store all these constants in another file*/
	private final double P = 0;
	private final double I = 0;
	private final double D = 0;

	
	private final double Pd = 0;
	private final double Id = 0;
	private final double Dd = 0;
	private final double Fd = 0;
	private final double targetSpeed = 0;

	private boolean side = false;
	
	public DriveTrain() {
		leftTalon1 = new TalonSRX(RobotMap.leftTalon1);
		leftTalon2 = new TalonSRX(RobotMap.leftTalon2);
		rightTalon1 = new TalonSRX(RobotMap.rightTalon1);
		rightTalon2 = new TalonSRX(RobotMap.rightTalon2);

		ahrs = new AHRS(SPI.Port.kMXP);
		
		leftTalon2.follow(leftTalon1);
		rightTalon2.follow(rightTalon2);
		
		turnController = new PIDController(P, I, D, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f); // range of angles to accept
		turnController.setOutputRange(-0.45, 0.45); // range of power values to output
		turnController.setAbsoluteTolerance(2.0f); // pidcontroller will stop within 2 degrees of set point
		turnController.setContinuous();
		
		PIDTemplate.configTalon(leftTalon1, !side);
		PIDTemplate.configTalon(rightTalon1, !side);
		
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new JoystickCommand());
	}

	public void rotateDegrees(double angle) {
		ahrs.reset();
		turnController.reset();
		turnController.setPID(P, I, D);
		turnController.setSetpoint(angle);
		turnController.enable();
	
	}
	
	
	public void drive(ControlMode mode, double left, double right) {
		leftTalon1.set(mode, left);
		rightTalon1.set(mode, right);
	}

	public void setGearState() {
		SLOW = Value.kForward;
		FAST = Value.kReverse;
		UNKNOWN = Value.kOff;
	}

	public void applyShift(String gearState) {
		if(gearState.equals("SLOW")) {
			gear = SLOW;
			double P_Drive_LOW = 0;
			double I_Drive_LOW = 0;
			double D_Drive_LOW =  0;
			double F_Drive_LOW = 0; 
			double targetSpeed_Drive_LOW = 0;
			PIDTemplate.updatePID(leftTalon1, P_Drive_LOW, I_Drive_LOW, D_Drive_LOW, F_Drive_LOW,targetSpeed_Drive_LOW) ;
			PIDTemplate.updatePID(rightTalon1, P_Drive_LOW, I_Drive_LOW, D_Drive_LOW, F_Drive_LOW,targetSpeed_Drive_LOW) ;
		}else if(gearState.equals("FAST")) {
			gear = FAST;
			double P_Drive_HIGH = 0.35;
			double I_Drive_HIGH = 1.0E-4;
			double D_Drive_HIGH = 0.11;
			double F_Drive_HIGH = 0;
			double targetSpeed_Drive_FAST = 0;
			
			PIDTemplate.updatePID(leftTalon1, P_Drive_HIGH, I_Drive_HIGH, D_Drive_HIGH, F_Drive_HIGH, targetSpeed_Drive_FAST);
			PIDTemplate.updatePID(rightTalon1, P_Drive_HIGH, I_Drive_HIGH, D_Drive_HIGH, F_Drive_HIGH, targetSpeed_Drive_FAST);
		}
	}
	
	public void shiftSlow() {
		applyShift("SLOW");
	}
	public void shiftFast() {
		applyShift("FAST");
	}
	
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		drive(ControlMode.PercentOutput, -output, output); // spins robot in circles

	}
	
	
}
