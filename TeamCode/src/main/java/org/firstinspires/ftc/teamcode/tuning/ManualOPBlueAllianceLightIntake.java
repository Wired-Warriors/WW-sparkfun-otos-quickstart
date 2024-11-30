package org.firstinspires.ftc.teamcode.tuning;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ManualOPBlueAllianceLightIntake (Blocks to Java)")
@Disabled
public class ManualOPBlueAllianceLightIntake extends LinearOpMode {

  private DcMotor leftFront;
  private DcMotor leftBack;
  private DcMotor rightFront;
  private DcMotor rightBack;
  private DcMotor ArmLift;
  private DcMotor ArmExtender;
  private DcMotor ArmHangerLeft;
  private DcMotor ArmHangerRight;
  private CRServo Intake;
  private Servo Wrist;
  private TouchSensor TouchMagneticSensor;
  private LED LED_Intake;
  private ColorSensor ColorSensor_ColorSensor;
  private DistanceSensor ColorSensor_DistanceSensor;

  int gainColorSensor;
  // TODO: Enter the type for variable named colorReject
  String colorReject;
  ElapsedTime PID_Timer;
  String colorSample;
  int integral;
  int currentPos;
  int errorPosLast;
  int errorPos;
  int myColorData;
  double distanceColorSensor;
  double motorPower;
  int countColorCompare;
  boolean statusColorCompare;
  double errorRateMAX;
  double gainP;
  double motorPowerMAX;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int targetPos;
    double positionAngle;
    double maxWheelPower;
    double COUNTS_PER_DEGREE;
    int COUNTS_PER_MOTOR_REV;
    int GEAR_REDUCTION;
    int COUNTS_PER_GEAR_REV;
    int currentPos_Extender;
    ElapsedTime intakeTimer;
    int targetPos_Extender;
    int targetPos_Wrist;
    float axial;
    float lateral;
    float yaw;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double max;
    double currentPos_ArmLiftDeg;
    double velocityArmLift;
    int kP_ArmLift;
    int kI_ArmLift;
    int kD_ArmLift;
    double kF_ArmLift;

    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    ArmLift = hardwareMap.get(DcMotor.class, "Arm Lift");
    ArmExtender = hardwareMap.get(DcMotor.class, "Arm Extender");
    ArmHangerLeft = hardwareMap.get(DcMotor.class, "Arm Hanger Left");
    ArmHangerRight = hardwareMap.get(DcMotor.class, "Arm Hanger Right");
    Intake = hardwareMap.get(CRServo.class, "Intake");
    Wrist = hardwareMap.get(Servo.class, "Wrist");
    TouchMagneticSensor = hardwareMap.get(TouchSensor.class, "TouchMagnetic Sensor");
    LED_Intake = hardwareMap.get(LED.class, "LED_Intake");
    ColorSensor_ColorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
    ColorSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Color Sensor");

    // Put initialization blocks here.
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setDirection(DcMotor.Direction.REVERSE);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    leftFront.setDirection(DcMotor.Direction.REVERSE);
    rightFront.setDirection(DcMotor.Direction.FORWARD);
    ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ArmLift.setDirection(DcMotor.Direction.REVERSE);
    ((DcMotorEx) ArmLift).setVelocity(10);
    ArmExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ArmExtender.setDirection(DcMotor.Direction.REVERSE);
    ArmHangerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ArmHangerLeft.setDirection(DcMotor.Direction.FORWARD);
    ArmHangerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ArmHangerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmHangerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ArmHangerRight.setDirection(DcMotor.Direction.FORWARD);
    ArmHangerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ArmHangerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Intake.setDirection(CRServo.Direction.REVERSE);
    Wrist.setDirection(Servo.Direction.FORWARD);
    maxWheelPower = 0.5;
    COUNTS_PER_MOTOR_REV = 28;
    GEAR_REDUCTION = 144;
    COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360;
    currentPos_Extender = ArmExtender.getCurrentPosition();
    intakeTimer = new ElapsedTime();
    VerifyColor_Initializations();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      // Target position, in absolute position - encoder ticks
      targetPos = 0;
      targetPos_Extender = 0;
      // Position controller proportional gain
      gainP = 0.002;
      // Maximum arm movement speed
      errorRateMAX = 0.4;
      // Calculate absolute position considering initial position as zero
      currentPos = ArmLift.getCurrentPosition();
      targetPos = currentPos;
      currentPos_Extender = ArmExtender.getCurrentPosition();
      targetPos_Extender = currentPos_Extender;
      targetPos_Wrist = 0;
      ((DcMotorEx) ArmLift).setMotorEnable();
      ((DcMotorEx) ArmExtender).setMotorEnable();
      // Lift arm off of hard stop
      targetPos = (int) (COUNTS_PER_DEGREE * 30);
      targetPos_Extender = 0;
      while (opModeIsActive()) {
        VerifyColor_Runtime();
        VerifyColor_Telemetry();
        // ////////  ROBOT MONTION CONTROL
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // Note: pushing stick forward gives negative value
        axial = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = ((axial - lateral) + yaw) * maxWheelPower;
        rightFrontPower = ((axial + lateral) - yaw) * maxWheelPower;
        leftBackPower = (axial + lateral + yaw) * maxWheelPower;
        rightBackPower = ((axial - lateral) - yaw) * maxWheelPower;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
          leftFrontPower = leftFrontPower / max;
          rightFrontPower = rightFrontPower / max;
          leftBackPower = leftBackPower / max;
          rightBackPower = rightBackPower / max;
        }
        // Send calculated power to wheels.
        leftBack.setPower(leftFrontPower);
        rightBack.setPower(rightFrontPower);
        leftFront.setPower(leftBackPower);
        rightFront.setPower(rightBackPower);
        // Power Half Button For Drive
        if (gamepad1.right_bumper) {
          maxWheelPower = 0.25;
        } else {
          maxWheelPower = 0.5;
        }
        // Arm and Intake Control
        // Reset
        if (gamepad2.start) {
          targetPos = (int) (COUNTS_PER_DEGREE * 4);
          targetPos_Extender = 0;
          Intake.setPower(0);
          Wrist.setPosition(0.85);
        }
        // Top Basket Deposit
        if (gamepad2.dpad_up) {
          targetPos = (int) (COUNTS_PER_DEGREE * 100);
          targetPos_Extender = 1550;
          Wrist.setPosition(0.4);
        }
        // Bottom Basket Deposit
        if (gamepad2.dpad_down) {
          targetPos = (int) (COUNTS_PER_DEGREE * 85);
          targetPos_Extender = 700;
          Wrist.setPosition(0.4);
        }
        // Specimen Prepare to Hang
        if (gamepad2.dpad_left) {
          targetPos = (int) (COUNTS_PER_DEGREE * 68);
          targetPos_Extender = 650;
          Wrist.setPosition(0.65);
        }
        if (gamepad2.dpad_right) {
          Wrist.setPosition(0.275);
        }
        // Far Pickup
        if (gamepad2.b) {
          Intake.setPower(1);
          targetPos = (int) (COUNTS_PER_DEGREE * 30);
          targetPos_Extender = 1550;
          Wrist.setPosition(0.65);
        }
        // Far Extend
        if (gamepad2.y) {
          Intake.setPower(0);
          targetPos = (int) (COUNTS_PER_DEGREE * 40);
          targetPos_Extender = 1550;
          Wrist.setPosition(0.65);
        }
        // Near Extend
        if (gamepad2.x) {
          Intake.setPower(0);
          targetPos = (int) (COUNTS_PER_DEGREE * 35);
          targetPos_Extender = 700;
          Wrist.setPosition(0.65);
        }
        // Near Pickup
        if (gamepad2.a) {
          Intake.setPower(1);
          targetPos = (int) (COUNTS_PER_DEGREE * 23);
          targetPos_Extender = 700;
          Wrist.setPosition(0.65);
        }
        // If sample is in the intake, turn off the intake servo
        if (distanceColorSensor < 1.75) {
          Intake.setPower(0);
        }
        // Calculating the Lift Arm Angle for compensation for gravity in PID.  26 ticks = -27.5deg, 839 ticks = 53.7deg, so [deg]=0.1[ticks]-30.1
        currentPos_ArmLiftDeg = 0.1 * ArmLift.getCurrentPosition() - 30.1;
        // ///////////////////////Move the lift arm and arm extender
        // Gain scheduling for moving in different directions due to the heavy weight
        velocityArmLift = ((DcMotorEx) ArmLift).getVelocity(AngleUnit.DEGREES);
        positionAngle = 90;
        if (velocityArmLift > 0) {
          kP_ArmLift = 10;
          kI_ArmLift = 2;
          kD_ArmLift = 0;
          kF_ArmLift = Math.cos(positionAngle / 180 * Math.PI) * 10;
        } else if (velocityArmLift < 0) {
          kP_ArmLift = 10;
          kI_ArmLift = 2;
          kD_ArmLift = 0;
          kF_ArmLift = Math.cos(positionAngle / 180 * Math.PI) * 1000;
        } else {
          kP_ArmLift = 10;
          kI_ArmLift = 2;
          kD_ArmLift = 0;
          kF_ArmLift = Math.cos(positionAngle / 180 * Math.PI) * 10;
        }
        ArmExtender.setTargetPosition(targetPos_Extender);
        ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmExtender.setPower(1);
        ProportionalController(targetPos, gainP, errorRateMAX);
        // Shut off intake if sample is present AND we're not in the process of rejecting a sample
        // Manual Controls
        if (gamepad1.dpad_up) {
          Wrist.setPosition(0.275);
        }
        if (gamepad2.left_bumper) {
          Intake.setPower(1);
        }
        if (gamepad2.right_bumper) {
          Intake.setPower(-1);
        }
        // Hanging
        if (gamepad1.a) {
          ((DcMotorEx) ArmHangerLeft).setMotorEnable();
          ArmHangerLeft.setTargetPosition(180);
          ArmHangerLeft.setPower(1);
          ArmHangerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          if (ArmHangerLeft.getCurrentPosition() >= 180) {
            ((DcMotorEx) ArmHangerLeft).setMotorDisable();
          }
        }
        if (gamepad1.a) {
          ((DcMotorEx) ArmHangerRight).setMotorEnable();
          ArmHangerRight.setTargetPosition(180);
          ArmHangerRight.setPower(1);
          ArmHangerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          if (ArmHangerRight.getCurrentPosition() >= 180) {
            ((DcMotorEx) ArmHangerRight).setMotorDisable();
          }
        }
        // Run the position control function
        // Update telemerty output
        telemetry.addData("Extender Pos", ArmExtender.getCurrentPosition());
        telemetry.addData("Touch Sensor is Pressed", TouchMagneticSensor.isPressed());
        telemetry.addData("currentPos = ", currentPos);
        telemetry.addData("targetPos = ", targetPos);
        telemetry.addData("errorPos = ", errorPos);
        telemetry.addData("Arm Lift Motor Power = ", motorPower);
        telemetry.addData("Left 1", leftFront.getPower());
        telemetry.addData("Left 2", leftBack.getPower());
        telemetry.addData("Right 1", rightFront.getPower());
        telemetry.addData("Right 2", rightBack.getPower());
        telemetry.addData("Intake Timer", intakeTimer);
        telemetry.addData("V = ", velocityArmLift);
        telemetry.addData("Encoder = ", ArmLift.getCurrentPosition());
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void ProportionalController(double targetPos, double P, double RateMAX) {
    // Propostional Arm Position Controller
    // Update current position
    currentPos = ArmLift.getCurrentPosition();
    // Calculate the error that the controller will work to minimize.  Means that it will try to make currentPos equal to targetPos.
    errorPos = (int) (targetPos - currentPos);
    // This is the actual proportional control, with the rate of change limited to errorRateMAX.
    // Note that it can be negative or positive depending on which direction the motor needs to turn (if the current position is larger than target or less than target).
    motorPower = Math.min(Math.max(errorPos * gainP, -errorRateMAX), errorRateMAX);
    if (Math.abs(motorPower) > 1) {
      // Normalize motor power to be less than 1.0
      motorPowerMAX = Math.abs(motorPower);
      motorPower = motorPower / motorPowerMAX;
    }
    // Move the motor / arm in the proper direction, then do it all again in the loop.
    ArmLift.setPower(motorPower);
  }

  /**
   * Describe this function...
   */
  private void PID_Controller(double targetPos, double positionAngle, double P, double i, int D, int Fg, double RateMAX) {
    double feedforward;
    double derivative;
    double gainI;
    double gainD;

    PID_Timer = new ElapsedTime();
    integral = 0;
    errorPosLast = 0;
    gainI = 0;
    while (currentPos != targetPos) {
      feedforward = Math.cos(positionAngle / 180 * Math.PI) * Fg;
      // Propostional Arm Position Controller
      // Update current position
      currentPos = ArmLift.getCurrentPosition();
      // Calculate the error that the controller will work to minimize.  Means that it will try to make currentPos equal to targetPos.
      errorPos = (int) (targetPos - currentPos);
      integral = (int) (integral + errorPos * PID_Timer.seconds());
      derivative = (errorPos - errorPosLast) / PID_Timer.seconds();
      // This is the actual proportional control, with the rate of change limited to errorRateMAX.
      // Note that it can be negative or positive depending on which direction the motor needs to turn (if the current position is larger than target or less than target).
      motorPower = Math.min(Math.max(errorPos * gainP + integral * gainI, -errorRateMAX), errorRateMAX);
      if (Math.abs(motorPower) > 1) {
        // Normalize motor power to be less than 1.0
        motorPowerMAX = Math.abs(motorPower);
        motorPower = motorPower / motorPowerMAX;
      }
      // Move the motor / arm in the proper direction, then do it all again in the loop.
      ArmLift.setPower(motorPower);
      errorPosLast = errorPos;
      PID_Timer.reset();
    }
  }

  /**
   * Describe this function...
   */
  private void VerifyColor_Initializations() {
    String colorAlliance;

    LED_Intake.off();
    // //////// Set Current Alliance Color (to reject wrong colors
    // Enable this block to set alliance color to blue, otherwise it will be set to red.
    colorAlliance = "BLUE";
    // Used to reduce confusion. If our alliance color is blue, we reject red samples, if
    // it's red we reject blue samples. The only code chance is to "colorAlliance" above.
    colorReject = colorAlliance.equals("BLUE") ? "RED" : "BLUE";
    // //////// Initialize color sensor
    gainColorSensor = 35;
    countColorCompare = 0;
    statusColorCompare = false;
    colorSample = "NO SAMPLE PRESENT";
    // //////// Initialize LEDs
    waitForStart();
  }

  /**
   * Describe this function...
   */
  private void VerifyColor_Runtime() {
    NormalizedRGBA myNormalizedColors;

    // Put these blocks into the main While loop called while opModeIsActive
    // Sample color detection and verification to alliance color
    // Read the data from the color sensor
    myNormalizedColors = ((NormalizedColorSensor) ColorSensor_ColorSensor).getNormalizedColors();
    myColorData = myNormalizedColors.toColor();
    // Call the VerifySampleColor function to verify the color is blue or reed, compare to colorAlliance, and reject the wrong alliance color sample
    colorSample = VerifySampleColor(gainColorSensor, colorReject, myNormalizedColors);
  }

  /**
   * Describe this function...
   */
  private void VerifyColor_Telemetry() {
    // Put these blocks into the main While loop with other telemetry (called while opModeIsActive)
    // /////// UPDATE TELEMETRY
    telemetry.addData("Sample Color: ", colorSample);
    telemetry.addData("Distance to Sample (in): ", Double.parseDouble(JavaUtil.formatNumber(ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH), 3)));
  }

  /**
   * VerifySampleColor
   * Reads the color sensor data, uses the included calibration to determine the color of the
   * sample based on the Hue, and compares that color to the color that should be rejected
   * (the one that is not our current alliance color). Yellow samples are always accepted.
   * If rejected, a message is displayed and an LED is lit so the drive can reverse
   * the intake. You could add code to automatically reverse the intake and eject the
   * sample in the same section where the LED is turned on to make it more automatic.
   * The function uses the distance sensor in the color sensor to determine if the intake
   * is empty or if it has a sample present. If a sample is present, the distance to the
   * sample will be lower than a threshold (distanceColorSensor) that you must calibrate.
   */
  private String VerifySampleColor(float gainColorSensor,
      // TODO: Enter the type for argument named colorReject
      String colorReject,
      NormalizedRGBA myNormalizedColors) {
    float hueColorSensor;
    float saturationColorSensor;
    float valueColorSensor;

    // Set up and read the color data when the function is called (reduces runtime)
    ((NormalizedColorSensor) ColorSensor_ColorSensor).setGain(gainColorSensor);
    hueColorSensor = JavaUtil.rgbToHue(Color.red(myColorData), Color.green(myColorData), Color.blue(myColorData));
    saturationColorSensor = JavaUtil.rgbToSaturation(Color.red(myColorData), Color.green(myColorData), Color.blue(myColorData));
    valueColorSensor = JavaUtil.rgbToValue(Color.red(myColorData), Color.green(myColorData), Color.blue(myColorData));
    distanceColorSensor = ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH);
    // Run the comparison and verification only if there is a sample present in the intake by using the distance sensor.
    // If a sample is in the intake, the distance will be smaller than if there isn't one there.  THE DISTANCE IS IN INCHES!!!
    if (distanceColorSensor < 1.75) {
      // Determine if the sample is red, yellow or blue based on the hue value.
      if (hueColorSensor < 35) {
        // If hue is less than the value, then it calls it red
        colorSample = "RED";
      } else if (hueColorSensor < 100) {
        // Since the hue wasn't less than the red value, it checks if it's less than the yellow value.  If so, then it's yellow.
        colorSample = "YELLOW";
      } else if (hueColorSensor > 200) {
        // Since the hue wasn't less than either value, it checks if it's greater than the blue value.  If so, then it's blue.
        colorSample = "BLUE";
      } else {
        // If the hue is not less than any of the values, then we don't know what color it is.
        colorSample = "UNKNOWN";
      }
      // Add a delay counter to reduce toggling in colorSample <> colorReject comparison: we want this to be reliable, not rest if the lighting shifts.
      // This is where we compare the color to the alliance color we want to reject.  If they are the same, then statusColorCompare is "true," if not then it's "false"
      statusColorCompare = colorReject == colorSample;
      if (statusColorCompare == true) {
        // If the sample is not our alliance color, we put a messge on the display, light the LED, and rumble the controller
        telemetry.addLine("!!!!!! WRONG SAMPLE COLOR - REJECTING SAMPLE !!!!!!");
        countColorCompare = countColorCompare + 1;
        LED_Intake.on();
        Intake.setPower(-1);
      }
    } else {
      LED_Intake.off();
      colorSample = "NO SAMPLE PRESENT";
    }
    return colorSample;
  }
}
