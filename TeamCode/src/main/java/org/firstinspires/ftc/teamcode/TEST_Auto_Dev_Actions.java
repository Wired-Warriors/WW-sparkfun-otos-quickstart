/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.source.tree.CompilationUnitTree;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This is our development OpMode for AUTO, the first one we're experimenting with in java for 23168
 * We will integrate to following in this mode:
 *    - Limelight3a (limelight)
 *    - Sparkfun OTOS (sensor_otos)
 *    - Mecanum drive (motors left_front, left_back, right_front, right_back)
 *    - Road Runner using a localizer for the OTOS developed by @j5155 on the FTC Discord https://github.com/jdhs-ftc
 *
 * All of the drive configuration is done via MecanumDrive.java, you do not have to manage that here.
 *
 * THIS MODE IS CONFIGURED FOR PANCAKE, NOT WAFFLES
 *
 */
@Autonomous(name="TEST_AUTO_Development Actions", group="AUTO")
//@Disabled
public class TEST_Auto_Dev_Actions extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private SparkFunOTOS otos;
    private DcMotor ArmLift;
    private DcMotor ArmExtender;
    private DcMotor ArmHangerLeft;
    private DcMotor ArmHangerRight;
    private CRServo Intake;
    private Servo Wrist;
    private LED LED_Intake;
    private ColorSensor ColorSensor_ColorSensor;
    private DistanceSensor ColorSensor_DistanceSensor;

    /////////////////////////////////////////////////////////////////////////
    // Declare variables
    //TODO ********** Set the Alliance Color  **************
    public String colorAlliance = "BLUE"; //Enter either BLUE or RED alliance, this will chance all further color-related settings
    public double txLimelight;
    public double tyLimelight;
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
    double powerIntake = 0;


    //TODO *********** Set the starting pose for the robot based on the alliance start position,
    // X and Y in INCHES from the center of the field, heading in RADIANS (or convert DEGREES to
    // RADIANS by multiplying the value in DEGREES by Math.PI/180
    Pose2d beginPose = new Pose2d(-14, -62.69, 90*Math.PI/180);

    @Override
    public void runOpMode() {

        int targetPos;
        double maxWheelPower;
        double COUNTS_PER_DEGREE;
        int COUNTS_PER_MOTOR_REV;
        int GEAR_REDUCTION;
        int COUNTS_PER_GEAR_REV;
        int currentPos_Extender;
        ElapsedTime intakeTimer;
        int targetPos_Extender;
        int targetPos_Wrist;
        int targetPos_Hanger;
        double velocityArmLift;

        //Set Hardware Map
        ArmLift = hardwareMap.get(DcMotor.class, "Arm Lift");
        ArmExtender = hardwareMap.get(DcMotor.class, "Arm Extender");
        ArmHangerLeft = hardwareMap.get(DcMotor.class, "Arm Hanger Left");
        ArmHangerRight = hardwareMap.get(DcMotor.class, "Arm Hanger Right");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        LED_Intake = hardwareMap.get(LED.class, "LED_Intake");
        ColorSensor_ColorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        ColorSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Color Sensor");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        initDevices(); // Initialize all motors, servos, sensors

        //Settings for the lift arm
        COUNTS_PER_MOTOR_REV = 28;
        GEAR_REDUCTION = 144;
        COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
        COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV / 360;
        currentPos_Extender = ArmExtender.getCurrentPosition();
        intakeTimer = new ElapsedTime();
//        VerifyColor_Initializations(); // Initialize Color Sensor

        targetPos = 0;           // Target arm position, in absolute position - encoder ticks
        targetPos_Extender = 0;  // Target arm position, in absolute position - encoder ticks
        gainP = 0.002;           // Position controller proportional gain
        errorRateMAX = 0.4;      // Maximum arm movement speed
        currentPos = ArmLift.getCurrentPosition();         // Calculate absolute position considering initial position as zero
        targetPos = currentPos;
        currentPos_Extender = ArmExtender.getCurrentPosition();
        targetPos_Extender = currentPos_Extender;
        targetPos_Hanger = 0;
        targetPos_Wrist = 0;
        ((DcMotorEx) ArmLift).setMotorEnable();
        ((DcMotorEx) ArmExtender).setMotorEnable();

        // TODO: Set initial limelight pipeline for alliance color: 0=red, 1=blue, 2=yellow
        limelight.pipelineSwitch(1);

        //Instantiate the roadrunner mecanum drive (via the OTOS localizer)
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ////////////////////////////////////////////////////////////////////////////////////
        // Wait for the game to start (driver presses START)
        ////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        if(isStopRequested()) return;

        //Build the actions for our AUTO mode
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(new setArmPostionAction(ArmLift, COUNTS_PER_DEGREE, 35))
                        .stopAndAdd(new setArmExtensionAction(ArmExtender,700))
                        .waitSeconds(10)
                        .stopAndAdd(new setIntakePowerAction(Intake,1))
                        .waitSeconds(5)
//                        .splineTo(new Vector2d(-51, -38), 90*Math.PI/180)
//                        .waitSeconds(2)
//                        .splineTo(new Vector2d(-54, -54), -135*Math.PI/180)
//                        .waitSeconds(2)
//                        .splineTo(new Vector2d(-60.5, -38), 90*Math.PI/180)
//                        .waitSeconds(2)
//                        .splineTo(new Vector2d(-54, -54), -135*Math.PI/180)
//                        .waitSeconds(2)
//                        .splineTo(new Vector2d(-51,-38),90*Math.PI/180)
//                        .splineTo(new Vector2d(-60.5,-7),90*Math.PI/180)
//                        .lineToY(-55)
                        .build());


//        // Lift arm off of hard stop
//        targetPos = (int) (COUNTS_PER_DEGREE * 30);
//        targetPos_Extender = 0;
        distanceColorSensor = ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH);  //Distance to the sample in the intake, used to switch off intake

        ProportionalController(targetPos, gainP, errorRateMAX);  //Lift arm off of the stop

        SparkFunOTOS.Pose2D pos = otos.getPosition(); //Read OTOS Pose for telemetry

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.addLine();
        telemetry.addData("OTOS Data", "X: (%.1f), Y: (%.1f), H: (%.2f)", pos.x,pos.y,pos.h);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
    }

    ///////////////////////////////////////////////////
    //PUBLIC CLASSES FOR ROADRUNNER ACTION DEFINITIONS
    //////////////////////////////////////////////////

    //Set the target arm position (assumes P-Controller is running in the background)
    // Do we really need this?  Can we just set a value for targetPos in the action builder instead?
    public class setArmPostionAction implements Action {
        DcMotor ArmLift;
        double targetPos;
        double COUNTS_PER_DEGREE;

        public setArmPostionAction(DcMotor ArmLift,double COUNTS_PER_DEGREE, double targetPos) {
            this.ArmLift = ArmLift;
            this.targetPos = targetPos;
            this.COUNTS_PER_DEGREE = COUNTS_PER_DEGREE;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPos = (int) (COUNTS_PER_DEGREE * 100);

            return false;
        }
    }

    // Set the arm extension to a given position
    public class setArmExtensionAction implements Action {
        DcMotor ArmExtender;
        double targetPos_Extender;

        public setArmExtensionAction(DcMotor ArmExtender, double targetPos_Extender) {
            this.ArmExtender = ArmExtender;
            this.targetPos_Extender = targetPos_Extender;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ArmExtender.setTargetPosition((int) targetPos_Extender);
            ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtender.setPower(1);
            return false;
        }
    }

    // Set the hangers to a given position
    public class setHangerPositionAction implements Action {
        DcMotor ArmHangerLeft;
        DcMotor ArmHangerRight;
        double targetPos_Hanger;

        public setHangerPositionAction(DcMotor ArmHangerLeft, DcMotor ArmHangerRight, double targetPos_Hanger) {
            this.ArmHangerLeft = ArmHangerLeft;
            this.ArmHangerRight = ArmHangerRight;
            this.targetPos_Hanger = targetPos_Hanger;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ((DcMotorEx) ArmHangerLeft).setMotorEnable();
            ArmHangerLeft.setTargetPosition((int) targetPos_Hanger);
            ((DcMotorEx) ArmHangerRight).setMotorEnable();
            ArmHangerRight.setTargetPosition((int) targetPos_Hanger);
            ArmHangerLeft.setPower(1);
            ArmHangerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmHangerRight.setPower(1);
            ArmHangerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    // Set the wrist to a given position
    public class setWristPositionAction implements Action {
        Servo Wrist;
        double targetPos_Wrist;

        public setWristPositionAction(Servo Wrist, double targetPos_Wrist) {
            this.Wrist = Wrist;
            this.targetPos_Wrist = targetPos_Wrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Wrist.setPosition(targetPos_Wrist);
            return false;
        }
    }

    // Set the intake power
    public class setIntakePowerAction implements Action {
        CRServo Intake;
        double powerIntake;

        public setIntakePowerAction(CRServo Intake, double powerIntake) {
            this.Intake = Intake;
            this.powerIntake = powerIntake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Intake.setPower(powerIntake);
            return false;
        }
    }

    //////////////////////////////////////////////////
    // PRIVATE VOIDS REFERENCED IN THE PUBLIC VOID
    //////////////////////////////////////////////////

    // Initializes all devices: sensors and actuators, to clean up the code
    private void initDevices(){
        //INIT LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // Set a default defined pipeline
        limelight.start(); // This tells Limelight to start looking!

        // Initialize actuators
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
    }


//    private void VerifyColor_Initializations() {
//        String colorAlliance;
//
//        LED_Intake.off();
//        // //////// Set Current Alliance Color (to reject wrong colors
//        // Enable this block to set alliance color to blue, otherwise it will be set to red.
//        colorAlliance = "BLUE";
//        // Used to reduce confusion. If our alliance color is blue, we reject red samples, if
//        // it's red we reject blue samples. The only code chance is to "colorAlliance" above.
//        colorReject = colorAlliance.equals("BLUE") ? "RED" : "BLUE";
//        // //////// Initialize color sensor
//        gainColorSensor = 35;
//        countColorCompare = 0;
//        statusColorCompare = false;
//        colorSample = "NO SAMPLE PRESENT";
//        // //////// Initialize LEDs
//        waitForStart();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void VerifyColor_Runtime() {
//        NormalizedRGBA myNormalizedColors;
//
//        // Put these blocks into the main While loop called while opModeIsActive
//        // Sample color detection and verification to alliance color
//        // Read the data from the color sensor
//        myNormalizedColors = ((NormalizedColorSensor) ColorSensor_ColorSensor).getNormalizedColors();
//        myColorData = myNormalizedColors.toColor();
//        // Call the VerifySampleColor function to verify the color is blue or reed, compare to colorAlliance, and reject the wrong alliance color sample
//        colorSample = VerifySampleColor(gainColorSensor, colorReject, myNormalizedColors);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void VerifyColor_Telemetry() {
//        // Put these blocks into the main While loop with other telemetry (called while opModeIsActive)
//        // /////// UPDATE TELEMETRY
//        telemetry.addData("Sample Color: ", colorSample);
//        telemetry.addData("Distance to Sample (in): ", Double.parseDouble(JavaUtil.formatNumber(ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH), 3)));
//    }
//
//    /**
//     * VerifySampleColor
//     * Reads the color sensor data, uses the included calibration to determine the color of the
//     * sample based on the Hue, and compares that color to the color that should be rejected
//     * (the one that is not our current alliance color). Yellow samples are always accepted.
//     * If rejected, a message is displayed and an LED is lit so the drive can reverse
//     * the intake. You could add code to automatically reverse the intake and eject the
//     * sample in the same section where the LED is turned on to make it more automatic.
//     * The function uses the distance sensor in the color sensor to determine if the intake
//     * is empty or if it has a sample present. If a sample is present, the distance to the
//     * sample will be lower than a threshold (distanceColorSensor) that you must calibrate.
//     */
//    private String VerifySampleColor(float gainColorSensor,
//                                     String colorReject,
//                                     NormalizedRGBA myNormalizedColors) {
//        float hueColorSensor;
//        float saturationColorSensor;
//        float valueColorSensor;
//
//        // Set up and read the color data when the function is called (reduces runtime)
//        ((NormalizedColorSensor) ColorSensor_ColorSensor).setGain(gainColorSensor);
//        hueColorSensor = JavaUtil.rgbToHue(Color.red(myColorData), Color.green(myColorData), Color.blue(myColorData));
//        saturationColorSensor = JavaUtil.rgbToSaturation(Color.red(myColorData), Color.green(myColorData), Color.blue(myColorData));
//        valueColorSensor = JavaUtil.rgbToValue(Color.red(myColorData), Color.green(myColorData), Color.blue(myColorData));
//        distanceColorSensor = ColorSensor_DistanceSensor.getDistance(DistanceUnit.INCH);
//        // Run the comparison and verification only if there is a sample present in the intake by using the distance sensor.
//        // If a sample is in the intake, the distance will be smaller than if there isn't one there.  THE DISTANCE IS IN INCHES!!!
//        if (distanceColorSensor < 1.75) {
//            // Determine if the sample is red, yellow or blue based on the hue value.
//            if (hueColorSensor < 35) {
//                // If hue is less than the value, then it calls it red
//                colorSample = "RED";
//            } else if (hueColorSensor < 100) {
//                // Since the hue wasn't less than the red value, it checks if it's less than the yellow value.  If so, then it's yellow.
//                colorSample = "YELLOW";
//            } else if (hueColorSensor > 200) {
//                // Since the hue wasn't less than either value, it checks if it's greater than the blue value.  If so, then it's blue.
//                colorSample = "BLUE";
//            } else {
//                // If the hue is not less than any of the values, then we don't know what color it is.
//                colorSample = "UNKNOWN";
//            }
//            // Add a delay counter to reduce toggling in colorSample <> colorReject comparison: we want this to be reliable, not rest if the lighting shifts.
//            // This is where we compare the color to the alliance color we want to reject.  If they are the same, then statusColorCompare is "true," if not then it's "false"
//            statusColorCompare = colorReject == colorSample;
//            if (statusColorCompare == true) {
//                // If the sample is not our alliance color, we put a messge on the display, light the LED, and rumble the controller
//                telemetry.addLine("!!!!!! WRONG SAMPLE COLOR - REJECTING SAMPLE !!!!!!");
//                countColorCompare = countColorCompare + 1;
//                LED_Intake.on();
//                Intake.setPower(-1);
//            }
//        } else {
//            LED_Intake.off();
//            colorSample = "NO SAMPLE PRESENT";
//        }
//        return colorSample;
//    }
    private void ProportionalController(double targetPos, double P, double RateMAX) {
        // Proportional Arm Position Controller
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
}
