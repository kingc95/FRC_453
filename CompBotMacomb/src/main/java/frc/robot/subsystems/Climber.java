// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Climber extends SubsystemBase {


<<<<<<< Updated upstream
private WPI_TalonSRX climberMotor = new WPI_TalonSRX(ClimberConstants.kClimberMotorPort);
private DigitalInput lowerLimit = new DigitalInput(2);
private Encoder climbEncoder = new Encoder(3, 4, ClimberConstants.kReverseEncoder, EncodingType.k1X);
private DigitalOutput goalMet = new DigitalOutput(6);
=======
public WPI_TalonSRX climberMotor = new WPI_TalonSRX(ClimberConstants.kClimberMotorPort);
public DigitalInput lowerLimit = new DigitalInput(2);
public Encoder climbEncoder = new Encoder(3, 4, ClimberConstants.kReverseEncoder, EncodingType.k1X);
public DigitalOutput goalMet = new DigitalOutput(6);
>>>>>>> Stashed changes

    public Climber() {
        climberMotor.configFactoryDefault();
        climberMotor.setInverted(false);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climbEncoder.setDistancePerPulse(ClimberConstants.kDistPerPulse);
 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber Speed", climberMotor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Limit Switch", lowerLimit.get());
        SmartDashboard.putNumber("Climber Dist", climbEncoder.getDistance());
        SmartDashboard.putNumber("Climber Dist Raw", climbEncoder.getRaw());
        if(lowerLimit.get() == false){
            climberMotor.set(ControlMode.PercentOutput, 0);
            climbEncoder.reset();
            goalMet.set(true);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void zeroClimber(){
        while(lowerLimit.get() == true){
            climberMotor.set(ControlMode.PercentOutput, ClimberConstants.climberSpeed);
        }
        climberMotor.set(ControlMode.PercentOutput, 0);
    }

<<<<<<< Updated upstream
    public void lowBarClimb(){
        goalMet.set(true);
        zeroClimber();
        while(climbEncoder.getRaw() < ClimberConstants.kLowBarTicks){
            climberMotor.set(ControlMode.PercentOutput, -ClimberConstants.climberSpeed);
        }
        climberMotor.set(ControlMode.PercentOutput, 0);
        goalMet.set(false);
    }

    public void medBarClimb(){
        goalMet.set(true);
        zeroClimber();
        while(climbEncoder.getRaw() < ClimberConstants.kMedBarTicks){
            climberMotor.set(ControlMode.PercentOutput, -ClimberConstants.climberSpeed);
        }
        climberMotor.set(ControlMode.PercentOutput, 0);
        goalMet.set(false);
    }

=======
>>>>>>> Stashed changes
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void extendClimb(){
        if(lowerLimit.get() == false){
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
        else{
            climberMotor.set(ControlMode.PercentOutput, ClimberConstants.climberSpeed);
        }
    }

    public void retractClimb(){
        climberMotor.set(ControlMode.PercentOutput, -ClimberConstants.climberSpeed);
    }

    public void stopClimb(){
        climberMotor.set(ControlMode.PercentOutput, 0);
    }
}

