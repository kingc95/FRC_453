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


import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Intake extends SubsystemBase {

public static final double intakeSpeed = 0.5;
private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.kIntakeMotorPort);
private final ColorSensorV3 m_colorSensor = new ColorSensorV3(IntakeConstants.kI2cPort);
private final ColorMatch m_colorMatcher = new ColorMatch();
private boolean allyColorMatch = false;

    public Intake() {
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        m_colorMatcher.addColorMatch(IntakeConstants.kBlueTarget);
        m_colorMatcher.addColorMatch(IntakeConstants.kRedTarget);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Alliance alliance = DriverStation.getAlliance();
        Color detectedColor = m_colorSensor.getColor();
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == IntakeConstants.kBlueTarget) {
        colorString = "Blue";
        } else if (match.color == IntakeConstants.kRedTarget) {
        colorString = "Red";
        } else {
        colorString = "Unknown";
        }

        if((alliance == Alliance.Red && colorString.equals("Red")) || (alliance == Alliance.Red && colorString.equals("Unknown"))){
            allyColorMatch = true;
        }
        else if((alliance == Alliance.Blue && colorString.equals("Blue")) || (alliance == Alliance.Blue && colorString.equals("Unknown"))){
            allyColorMatch = true;
        }
        else{
            allyColorMatch = false;
        }

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
        SmartDashboard.putNumber("Intake Speed", intakeMotor.getMotorOutputPercent());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void intakeBall(){
        /* if(allyColorMatch){
            intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
        }
        else{
            intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.intakeSpeed);
        } */
        intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
    }

    public void outtakeBall(){
        intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.intakeSpeed);
    }

    public void stop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}

