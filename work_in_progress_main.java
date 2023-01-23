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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.lang.Math;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SkyNet", group="Linear Opmode")
public class Testing extends LinearOpMode {

    // Declare BIG BOI variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo servoZero = null;
    private Servo servoOne = null;
    private Servo servoString = null;
    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 1.4763; // in
    public static double GEAR_RATIO = 5; // output (wheel) speed / input (encoder) speed
    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4;

    // Sets all motor and servo powers to zero
    double leftFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightFrontPower = 0.0;
    double rightBackPower = 0.0;
    double servoZeroPower = 0.0;
    double servoStringPower = 0.0;
    double servoOnePower = 0.0;

    // createPower automatically sets the motor powers to current variables
    public void createPower(){
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. The strings used as parameters must match
        // the names assigned to the respective parts in the driver hub
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        servoZero = hardwareMap.get(Servo.class, "servo0");
        servoOne = hardwareMap.get(Servo.class, "servo1");
        servoString = hardwareMap.get(Servo.class, "servocont");

        // Because the motors all turn the same direction, one of the sides
        // must be reversed in order for them to all turn the same way
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Assigns variables to controller input
            double rightStickX = gamepad1.right_stick_x;
            double rightStickY  = gamepad1.right_stick_y;
            boolean upKey = gamepad1.dpad_up;
            boolean downKey = gamepad1.dpad_down;
            boolean leftKey = gamepad1.dpad_left;
            boolean rightKey = gamepad1.dpad_right;
            double leftTurn = gamepad1.left_trigger;
            double rightTurn = gamepad1.right_trigger;
            
            // Calculates the angle of the joystick, creates a variable that limits
            // the speed, and sets the steepness of the turn (cutoff) 
            double speedLimit = 0.75;
            //double angle = Math.toDegrees(Math.atan2(rightStickX, -rightStickY));
            double angle = (Math.atan2(rightStickY, rightStickX))-(Math.PI/2)
            double magnitude = speedLimit*Math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY);
            double cutoff = 0.5;
            double leftFrontpower = Math.sqrt(Math.pow(rightStickX, 2) + Math.pow(rightStickY, 2)) * (Math.sin(angle - Math.PI / 4)); 
            double rightBackPower = leftFrontPower;
            double rightFrontPower = Math.sqrt(Math.pow(rightStickX, 2) + Math.pow(rightStickY, 2)) * (Math.sin(angle + Math.PI / 4));
            double leftBackPower = rightFrontPower;
            // Takes the angle of the joystick, figures out which quadrant
            // it is in, and then assigns power accordingly
            /*if (angle%90 == 0){
                leftFrontPower = magnitude*1;
                rightFrontPower = magnitude*1;
                leftBackPower = magnitude*1;
                rightBackPower = magnitude*1;
            }
            else if (angle%90 == 1){
                leftFrontPower = magnitude*1;
                rightFrontPower = magnitude*-1;
                leftBackPower = magnitude*-1;
                rightBackPower = magnitude*1;
            }
            else if (angle%90 == 2){
                leftFrontPower = magnitude*-1;
                rightFrontPower = magnitude*-1;
                leftBackPower = magnitude*-1;
                rightBackPower = magnitude*-1;
            }
            else if (angle%90 == 3){
                leftFrontPower = magnitude*-1;
                rightFrontPower = magnitude*1;
                leftBackPower = magnitude*1;
                rightBackPower = magnitude*-1;
            }
            else if (angle%90 == 1){
                leftFrontPower = magnitude*1;
                rightFrontPower = magnitude*1;
                leftBackPower = magnitude*1;
                rightBackPower = magnitude*1;
            }
            else if (0 < angle && angle < 90){
                leftFrontPower = magnitude*1;
                rightFrontPower = (45-(angle%90))/(45);
                leftBackPower = (45-(angle%90))/(45);
                rightBackPower = magnitude*1;
            }
            else if (90 < angle && angle < 180){
                leftFrontPower = -(45-(angle%90))/(45);
                rightFrontPower = magnitude*-1;
                leftBackPower = magnitude*-1;
                rightBackPower = -(45-(angle%90))/(45);
            }
            else if (180 < angle && angle < 270){
                leftFrontPower = magnitude*-1;
                rightFrontPower = -(45-(angle%90))/(45);
                leftBackPower = -(45-(angle%90))/(45);
                rightBackPower = magnitude*-1;
            }
            else if (270 < angle && angle < 360){
                leftFrontPower = (45-(angle%90))/(45);
                rightFrontPower = magnitude*1;
                leftBackPower = magnitude*1;
                rightBackPower = (45-(angle%90))/(45);
            }
            else if (magnitude == 0){
                leftFrontPower = 0;
                rightFrontPower = 0;
                leftBackPower = 0;
                rightBackPower = 0;
            }
            */

            createPower();

            
            if (upKey){
                leftFrontPower = -0.5;
                rightFrontPower = -0.5;
                leftBackPower = -0.5;
                rightBackPower = -0.5;
                createPower();
            }
            if (downKey){
                leftFrontPower = 0.5;
                rightFrontPower = 0.5;
                leftBackPower = 0.5;
                rightBackPower = 0.5;
                createPower();
            }
            if (rightKey){
                leftFrontPower = -0.5;
                rightFrontPower = 0.5;
                leftBackPower = 0.5;
                rightBackPower = -0.5;
                createPower();
            }
            if (leftKey){
                leftFrontPower = 0.5;
                rightFrontPower = -0.5;
                leftBackPower = -0.5;
                rightBackPower = 0.5;
                createPower();
            }
            if  (!upKey && !downKey && !leftKey && !rightKey && rightStickX == 0 && rightStickY == 0){
                leftFrontPower = 0;
                leftBackPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
            }
            if (leftTurn > 0){
                leftFrontDrive.setPower(leftTurn * cutoff);
                leftBackDrive.setPower(leftTurn * cutoff);
                rightFrontDrive.setPower(-leftTurn * cutoff) ;
                rightBackDrive.setPower(-leftTurn * cutoff);
            }
            else if (rightTurn > 0){
                leftFrontDrive.setPower(-rightTurn * cutoff);
                leftBackDrive.setPower(-rightTurn * cutoff);
                rightFrontDrive.setPower(rightTurn * cutoff);
                rightBackDrive.setPower(rightTurn * cutoff);
            }else{
                leftFrontDrive.setPower(leftFrontPower * cutoff);
                rightFrontDrive.setPower(rightFrontPower * cutoff);
                leftBackDrive.setPower(leftBackPower * cutoff);
                rightBackDrive.setPower(rightBackPower * cutoff);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update();

        }
    }
}