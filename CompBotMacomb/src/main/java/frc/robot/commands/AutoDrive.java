package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;


public class AutoDrive extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LimeLight m_LimeLight;
    private final Drivetrain m_Drivetrain;
  
    /**
     * Creates a new AutoDrive.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoDrive(LimeLight subsystem1, Drivetrain subsystem2) {
      m_LimeLight = subsystem1;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem1);

      m_Drivetrain = subsystem2;

      addRequirements(subsystem2);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_LimeLight.m_ValidTarget == true){
            if((m_LimeLight.tx < -DriveConstants.k_AutoCorrectTurn) || (m_LimeLight.tx > DriveConstants.k_AutoCorrectTurn)){
                if(m_LimeLight.tx > 0){
                    if((m_LimeLight.ta < DriveConstants.k_AutoCorrectDist - 0.1) || (m_LimeLight.ta > DriveConstants.k_AutoCorrectDist + 0.1)){
                        if(m_LimeLight.ta > DriveConstants.k_AutoCorrectDist){
                            m_Drivetrain.drive(0, DriveConstants.k_AutoCorrectSpeed, -DriveConstants.k_AutoCorrectSpeed, false, false);
                        }
                        else{
                            m_Drivetrain.drive(0, -DriveConstants.k_AutoCorrectSpeed, -DriveConstants.k_AutoCorrectSpeed, false, false);
                        }
                    }
                    else{
                        m_Drivetrain.drive(0, 0, -DriveConstants.k_AutoCorrectSpeed, false, false);
                    }
                }
                else{
                    if((m_LimeLight.ta < DriveConstants.k_AutoCorrectDist - 0.1) || (m_LimeLight.ta > DriveConstants.k_AutoCorrectDist + 0.1)){
                        if(m_LimeLight.ta > DriveConstants.k_AutoCorrectDist){
                            m_Drivetrain.drive(0, DriveConstants.k_AutoCorrectSpeed, DriveConstants.k_AutoCorrectSpeed, false, false);
                        }
                        else{
                            m_Drivetrain.drive(0, -DriveConstants.k_AutoCorrectSpeed, DriveConstants.k_AutoCorrectSpeed, false, false);
                        }
                    }
                    else{
                        m_Drivetrain.drive(0, 0, DriveConstants.k_AutoCorrectSpeed, false, false);
                    }
                }
            }
            else{
                m_Drivetrain.drive(0, 0, 0, false, false);
            }
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
