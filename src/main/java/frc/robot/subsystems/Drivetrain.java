/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private final WPI_TalonFX leftMaster =  new WPI_TalonFX(Constants.DrivetrainPorts.leftMasterPort);
  private final WPI_TalonFX leftFollower =  new WPI_TalonFX(Constants.DrivetrainPorts.leftMasterPort);
  private final WPI_TalonFX rightMaster =  new WPI_TalonFX(Constants.DrivetrainPorts.leftMasterPort);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.DrivetrainPorts.leftMasterPort);

  private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMaster, leftFollower);
  private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMaster, rightFollower);

  final TalonFXInvertType kFxInvertType = TalonFXInvertType.Clockwise;
  final NeutralMode kBrakeDurNeutral = NeutralMode.Brake;

  private final AnalogGyro m_gyro = new AnalogGyro(Constants.gyroPort); //-> To be replaced with Pigeon IMU


  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup,m_rightGroup);

  //Not needed as of now
  //private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  //private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3); //USE ROBOT CHARACTERIZATION TOOL


  public Drivetrain() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    //Programming the motors as TalonFXs
    //Left side
    leftMaster.configAllSettings(leftConfig);
    leftMaster.setInverted(false);
    leftMaster.setNeutralMode(kBrakeDurNeutral);

    leftFollower.configAllSettings(leftConfig);
    leftFollower.setInverted(false);
    leftFollower.setNeutralMode(kBrakeDurNeutral);
    leftFollower.follow(leftMaster);

    //Right side
    rightMaster.configAllSettings(rightConfig);
    rightMaster.setInverted(false);
    rightMaster.setNeutralMode(kBrakeDurNeutral);

    
    rightFollower.configAllSettings(rightConfig);
    rightFollower.setInverted(false);
    rightFollower.setNeutralMode(kBrakeDurNeutral);
    rightFollower.follow(rightMaster);
  }

  public void arcadeDrive(double fwd, double rot){
    drive.arcadeDrive(fwd, rot);
  }

  public double getLeftEncoderPos(){
    return leftMaster.getSelectedSensorPosition();
  }

  public double getRightEncoderPos(){
    return rightMaster.getSelectedSensorPosition();
  }

  public double getLeftEncoderDistance(){
    return getLeftEncoderPos() * Constants.DrivetrainNumericalConstants.distancePerPulseFactor;
  }

  public double getRightEncoderDistance(){
    return getRightEncoderDistance() * Constants.DrivetrainNumericalConstants.distancePerPulseFactor;
  }

  public void resetDrivetrainEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getGyroAngle(){
    return m_gyro.getAngle();
  }

  public void resetGryo(){
    m_gyro.reset();
  }

  public void driveStraight(double speed, double setPoint){
    double angleError = (setPoint - getGyroAngle()) * Constants.PIDConstants.gyrokP;
    angleError = Math.copySign(angleError, speed);
    drive.arcadeDrive(speed, angleError);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
