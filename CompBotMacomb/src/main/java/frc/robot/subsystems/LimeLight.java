package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    public boolean m_ValidTarget = false;
    public double tx;
    public double ty;
    public double tv;
    public double ta;

    public LimeLight(){
        //setup for calibration
    }

    @Override
    public void periodic() {
        updateLimeLightTracking();
    }

    public void updateLimeLightTracking() {
        final double TARGET = 13.0;

         tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
         tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
         ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_ValidTarget = false;
          return;
        }
        else{
            m_ValidTarget = true;
        }

        SmartDashboard.putNumber("Lime TV", tv);
        SmartDashboard.putNumber("Lime TX", tx);
        SmartDashboard.putNumber("Lime TY", ty);
        SmartDashboard.putNumber("Lime TA", ta);
        SmartDashboard.putBoolean("Target Aquired", m_ValidTarget);
    }
}
