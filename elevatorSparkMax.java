/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// BASED OFF THE REV SPARKMAX closed loop example.
// This uses the SmartDashboard for feedback and for selecting the levels. 

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Robot extends TimedRobot {

////////////////////////////////////////////////////////
  private final SendableChooser<String> el_chooser = new SendableChooser<>();
  private String elevatorAction = "none";

  private String level0 = "Level 0"; // made these strings ONLY because the Dashboard chooser wanted it.
  private String level01 = "Level 1";
  private String level02 = "Level 2";
  private String level03 = "Level 3";
  private String level04 = "Level 4";

/////////////////////////////////////////////////////////////////////////////
  // VARIABLES FOR LEVELS AND LIMITS ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////

  // elevator encoder positions... 
  private double elevatorPosition0 = 0;
  private double elevatorPosition1 = 20;
  private double elevatorPosition2 = 30;
  private double elevatorPosition3 = 40;
  private double elevatorPosition4 = 50;

  // top boundry limit
  private int forwardLimit = 60;
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

  private SparkMax elMotor;
  private SparkMaxConfig elMotorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder elEncoder;

  public Robot() {
    /** Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use. */
    elMotor = new SparkMax(25, MotorType.kBrushless);
    closedLoopController = elMotor.getClosedLoopController();
    elEncoder = elMotor.getEncoder();

    /** Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.*/
    elMotorConfig = new SparkMaxConfig();

    elMotorConfig.softLimit
    .forwardSoftLimit(forwardLimit)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(0)
    .reverseSoftLimitEnabled(true);

    /* Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors. */
    elMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /** Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder. */
    elMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.01)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 0.5, ClosedLoopSlot.kSlot1);  //// MAX SPEED?????

   // Set the idle mode to brake to stop immediately when reaching a limit ***********<<<<<======
    //motorConfig.idleMode(IdleMode.kBrake);

    /** Apply the configuration to the SPARK MAX.
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.*/
    elMotor.configure(elMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    //SmartDashboard.setDefaultNumber(key:"Elevator Position: ", elevatorPosition0);

    el_chooser.setDefaultOption("Level 0: ", level0);
    el_chooser.addOption("Level 1: ", level01);
    el_chooser.addOption("Level 2: ", level02);
    el_chooser.addOption("Level 3: ", level03);
    el_chooser.addOption("Level 4: ", level04);

    SmartDashboard.putData("Elevator Choices", el_chooser);

  } ////////////////////// END OF ROBOT CONSTRUCTOR ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void teleopPeriodic() {
    // Get the selected action from the SmartDashboard
    elevatorAction  = el_chooser.getSelected();
    double targetPosition = 0;
    if (elevatorAction == level0) {
      targetPosition = elevatorPosition0;
    } else if (elevatorAction == level01) {
      targetPosition = elevatorPosition1;
    } else if (elevatorAction == level02) {
      targetPosition = elevatorPosition2;
    } else if (elevatorAction == level03) {
      targetPosition = elevatorPosition3;
    } else if (elevatorAction == level04) {
      targetPosition = elevatorPosition4;
    }

      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);


    if (SmartDashboard.getBoolean("Control Mode", false)) {
      /*
       * Get the target velocity from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
      closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    } else {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
     // double targetPosition = SmartDashboard.getNumber("Target Position", 0);
      //closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  } // end of teleopPeriodic /////////////////////////////////////////////////////////////////

  @Override
  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putNumber("Actual Position", elEncoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", elEncoder.getVelocity());

    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!/////////////////////
    // TURNED THIS OFF SO WE DONT ACCIDENTALLY RESET THE ENCODER WHILE TESTING !!!!
    ///////////!!!!!!!!!!!vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv!/////////////////////

    // if (SmartDashboard.getBoolean("Reset Encoder", false)) {
    //   SmartDashboard.putBoolean("Reset Encoder", false);
    //   // Reset the encoder position to 0
    //   elEncoder.setPosition(0);        // TURNED THIS OFF SO WE DONT ACCIDENTALLY RESET THE ENCODER AT THE WRONG LEVEL  !!!!
    // }/////////////////////////////////////////////////////////////////////////// 
  }
}
