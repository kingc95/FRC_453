package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.Shooter;

public class Drivetrain extends SubsystemBase {
    public static final double normalSpeedMult = DriveConstants.kNormSpeedMult;
    public static final double turboSpeedMult = DriveConstants.kTurboSpeedMult;
    public long start = System.currentTimeMillis();

    private PigeonIMU pigeonIMU = new PigeonIMU(5);
    private WPI_TalonFX leftFrontDrive = new WPI_TalonFX(DriveConstants.kLeftFrontPort);
    private WPI_TalonFX leftRearDrive = new WPI_TalonFX(DriveConstants.kLeftRearPort);
    private WPI_TalonFX rightFrontDrive = new WPI_TalonFX(DriveConstants.kRightFrontPort);
    private WPI_TalonFX rightRearDrive = new WPI_TalonFX(DriveConstants.kRightRearPort);
    private MecanumDrive m_drive;
    private Shooter m_shooter = new Shooter();
    private Lifter m_lifter = new Lifter();
    private MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()));

    public Drivetrain() {

        m_drive = new MecanumDrive(leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive);
        addChild("m_drive", m_drive);
        m_drive.setSafetyEnabled(true);
        m_drive.setExpiration(0.1);
        m_drive.setMaxOutput(1.0);
        CameraServer.startAutomaticCapture();

        leftFrontDrive.configFactoryDefault();
        leftFrontDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftFrontDrive.setInverted(true);
        leftFrontDrive.setNeutralMode(NeutralMode.Brake);

        leftRearDrive.configFactoryDefault();
        leftRearDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftRearDrive.setInverted(true);
        leftRearDrive.setNeutralMode(NeutralMode.Brake);

        rightFrontDrive.configFactoryDefault();
        rightFrontDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFrontDrive.setInverted(false);
        rightFrontDrive.setNeutralMode(NeutralMode.Brake);

        rightRearDrive.configFactoryDefault();
        rightRearDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightRearDrive.setInverted(false);
        rightRearDrive.setNeutralMode(NeutralMode.Brake);

        pigeonIMU.configFactoryDefault();
        pigeonIMU.setYaw(0, 50);
        pigeonIMU.setAccumZAngle(0, 50);
        System.out.println("============================");
        System.out.println("Yaw and accumulated Z zero'ed");
        System.out.println("============================");
        System.out.println();
        

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(Rotation2d.fromDegrees(
                pigeonIMU.getFusedHeading()),
                new MecanumDriveWheelSpeeds(
                        leftFrontDrive.getSelectedSensorVelocity(),
                        leftRearDrive.getSelectedSensorVelocity(),
                        rightFrontDrive.getSelectedSensorVelocity(),
                        rightRearDrive.getSelectedSensorVelocity()));

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()));
    }

    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean turboEn) {
        if (fieldRelative) {
            if (turboEn) {
                m_drive.driveCartesian(ySpeed * turboSpeedMult, xSpeed * turboSpeedMult, rot * (turboSpeedMult / 2),
                        -pigeonIMU.getYaw());
            } else {
                m_drive.driveCartesian(ySpeed * normalSpeedMult, xSpeed * normalSpeedMult, rot * (normalSpeedMult / 2),
                        -pigeonIMU.getYaw());
            }
        } else {
            if (turboEn) {
                m_drive.driveCartesian(ySpeed * turboSpeedMult, xSpeed * turboSpeedMult, rot * (turboSpeedMult / 2));
            } else {
                m_drive.driveCartesian(ySpeed * normalSpeedMult, xSpeed * normalSpeedMult, rot * (normalSpeedMult / 2));
            }
        }
    }


    public void tarmacDrive(long startTime){
        //TODO: Add Intake, Lifter and Shooter code for low goal auton
        while(System.currentTimeMillis() < startTime + 10000){
            if(System.currentTimeMillis() < startTime + 3000){
                m_shooter.shootLow();
                m_lifter.lifterUp();
            }
            else{
                m_shooter.stopShoot();
            }
            if(System.currentTimeMillis() > startTime + 3000 && System.currentTimeMillis() < startTime + 5000){
                m_drive.driveCartesian(0.4, 0, 0);
            }
            else{
                m_drive.driveCartesian(0.0, 0, 0);
            }
        }
        
    }

    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        leftFrontDrive.setVoltage(volts.frontLeftVoltage);
        leftRearDrive.setVoltage(volts.rearLeftVoltage);
        rightFrontDrive.setVoltage(volts.frontRightVoltage);
        rightRearDrive.setVoltage(volts.rearRightVoltage);
    }

    public void resetEncoders() {
        leftFrontDrive.setSelectedSensorPosition(0);
        leftRearDrive.setSelectedSensorPosition(0);
        rightFrontDrive.setSelectedSensorPosition(0);
        rightRearDrive.setSelectedSensorPosition(0);
    }

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                leftFrontDrive.getSelectedSensorVelocity(),
                leftRearDrive.getSelectedSensorVelocity(),
                rightFrontDrive.getSelectedSensorVelocity(),
                rightRearDrive.getSelectedSensorVelocity());
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        pigeonIMU.setYaw(0);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return pigeonIMU.getFusedHeading();
    }

}
