package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static final class DriveConstants {
        public static final int kLeftFrontPort = 1;
        public static final int kLeftRearPort = 2;
        public static final int kRightFrontPort = 3;
        public static final int kRightRearPort = 4;

        public static final double k_AutoCorrectSpeed = 0.25;
        public static final double k_AutoCorrectDist = 1.0;
        public static final double k_AutoCorrectTurn = 0.1;

        public static final double kTrackWidth = 0.68;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.68;
        // Distance between centers of front and back wheels on robot

        public static final int kGearRatio = 16;

        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
            (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR * kGearRatio);
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPFrontLeftVel = 0.5;
        public static final double kPRearLeftVel = 0.5;
        public static final double kPFrontRightVel = 0.5;
        public static final double kPRearRightVel = 0.5;

        public static final double kNormSpeedMult = 0.66;
        public static final double kTurboSpeedMult = 1.0;

        //Button Mapping
        public static final int kTurboButtonMap = 3;
        public static final int kIntakeInMap = 9;
        public static final int kIntakeOutMap = 11;
        public static final int kLifterUpMap = 10;
        public static final int kLifterDownMap = 12;
        public static final int kClimberUpMap = 7;
        public static final int kClimberDownMap = 8;
    }

    public static final class IntakeConstants{
        public static final int kIntakeMotorPort = 6;
        public static final I2C.Port kI2cPort = I2C.Port.kOnboard;
        public static final double intakeSpeed = 1.0;
        public static final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
        public static final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    }

    public static final class ClimberConstants{
        public static final int kClimberMotorPort = 9;
        public static final double climberSpeed = 0.5;
        public static final int countsPerRev = 2048;

        public static double kDistPerPulse = (1.0/countsPerRev);
        public static boolean kReverseEncoder = true;
        public static int kLowBarTicks = 16342;
        public static int kMedBarTicks = 31717;
    }

    public static final class LifterConstants{
        public static final int kLifterMotorPort = 7;
        public static final double lifterSpeed = 1.0;
    }

    public static final class ShooterConstants{
        public static final int kShooterMotorPort = 8;
        /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
        public static final int kLowGoalMap = 2;
        public static final int kLowGoalVal = 5500;
        public static final int kHighGoalMap = 1;
        public static final int kHighGoalVal = 25000;
        public static final int kSliderFireMap = 4;
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
   public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static int k_DriverAutoButton = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.5;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

