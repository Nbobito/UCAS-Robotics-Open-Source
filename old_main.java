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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import java.util.Arrays;
import java.util.List;
import java.lang.Math;
//import androidx.annotation.NonNull;




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


  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftFrontDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightBackDrive = null;
  private Servo servoZero = null;
  private Servo servoOne = null;
  private Servo contServo = null;
  public static double SQUARE_LENGTH = 3 //ft
  public static double TICKS_PER_REV = 0;
  public static double WHEEL_RADIUS = 1.4763; // in
  public static double GEAR_RATIO = 5; // output (wheel) speed / input (encoder) speed
  public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
  public static double FORWARD_OFFSET = 4;
  private Encoder encoder0, encoder1, encoder2;


  encoder0 = new Encoder(hardwareMap.get(DcMotorEx.class, "encoder0"));
  encoder1 = new Encoder(hardwareMap.get(DcMotorEx.class, "encoder1"));
  encoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, "encoder2"));


  double leftFrontPower = 0.0;
  double leftBackPower = 0.0;
  double rightFrontPower = 0.0;
  double rightBackPower = 0.0;
  double contServoPower = 0.0;
  double servoZeroPower = 0.0;
  double servoOnePower = 0.0;


  public void createPower(){
      leftFrontDrive.setPower(leftFrontPower);
      rightFrontDrive.setPower(rightFrontPower);
      leftBackDrive.setPower(leftBackPower);
      rightBackDrive.setPower(rightBackPower);
  }
  //The Rev Hardware CLient might throw errors about these functions.
  /*public static double encoderTicksToInches(double ticks) {
      return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }
  public List<Double> getWheelPositions() {
      return Arrays.asList(
              encoderTicksToInches(leftEncoder.getCurrentPosition()),
              encoderTicksToInches(rightEncoder.getCurrentPosition()),
              encoderTicksToInches(frontEncoder.getCurrentPosition())
      );
  }
  @NonNull
  @Override
  public List<Double> getWheelPositions() {
      return Arrays.asList(
              encoderTicksToInches(leftEncoder.getCurrentPosition()),
              encoderTicksToInches(rightEncoder.getCurrentPosition()),
              encoderTicksToInches(frontEncoder.getCurrentPosition())
      );
  }


  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
      return Arrays.asList(
              encoderTicksToInches(leftEncoder.getRawVelocity()),
              encoderTicksToInches(rightEncoder.getRawVelocity()),
              encoderTicksToInches(frontEncoder.getRawVelocity())
      );
  }
}*/


  @Override
  public void runOpMode() {
      telemetry.addData("Status", "Initialized");
      telemetry.update();


      // Initialize the hardware variables. Note that the strings used here as parameters
      // to 'get' must correspond to the names assigned during the robot configuration
      // step (using the FTC Robot Controller app on the phone).
      leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
      rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
      leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
      rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
      servoZero = hardwareMap.get(Servo.class, "servo0");
      servoOne = hardwareMap.get(Servo.class, "servo1");
      contServo = hardwareMap.get(Servo.class, "contservo");


      // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
      // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
      // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


      // Wait for the game to start (driver presses PLAY)
      leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
      leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
      rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
      rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);


      waitForStart();
      runtime.reset();


      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {


          // Setup a variable for each drive wheel to save power level for telemetry


          // Choose to drive using either Tank Mode, or POV Mode
          // Comment out the method that's not used.  The default below is POV.


          // POV Mode uses left stick to go forward, and right stick to turn.
          // - This uses basic math to combine motions and is easier to drive straight.
          double rightStickX = gamepad1.right_stick_x;
          double rightStickY = gamepad1.right_stick_y;


          double angle = Math.toDegrees(Math.atan2(rightStickX, -rightStickY));
          if (angle < 0){
          angle += 360;
          }
          double magnitude = 0.75 * Math.sqrt(rightStickX * rightStickX + rightStickY * rightStickY);
          if (angle / 45 == 0) {
              leftFrontPower = magnitude;
              rightFrontPower = magnitude;
              leftBackPower = magnitude;
              rightBackPower = magnitude;
          }
          if (0 < angle && angle < 90) {
              leftFrontPower = magnitude * 1;
              rightFrontPower = 0;
              leftBackPower = 0;
              rightBackPower = magnitude * 1;
          } else if (90 < angle && angle < 180) {
              leftFrontPower = 0;
              rightFrontPower = magnitude * -1;
              leftBackPower = magnitude * -1;
              rightBackPower = 0;
          } else if (180 < angle && angle < 270) {
              leftFrontPower = magnitude * -1;
              rightFrontPower = 0;
              leftBackPower = 0;
              rightBackPower = magnitude * -1;
          } else if (270 < angle && angle < 360) {
              leftFrontPower = 0;
              rightFrontPower = magnitude * 1;
              leftBackPower = magnitude * 1;
              rightBackPower = 0;
          } else if (angle / 45 == 5) {
              leftFrontPower = magnitude * -1;
              rightFrontPower = magnitude * 0;
              leftBackPower = magnitude * 0;
              rightBackPower = magnitude * -1;
          } else if (angle == 0) {
              leftFrontPower = magnitude;
              rightFrontPower = magnitude;
              leftBackPower = magnitude;
              rightBackPower = magnitude;
          } else if (magnitude == 0) {
              leftFrontPower = 0;
              rightFrontPower = 0;
              leftBackPower = 0;
              rightBackPower = 0;
          }
          if (gamepad2.dpad_down) {
              servoZero.setPosition(1);
              servoOne.setPosition(1);
          } else if (gamepad2.dpad_up) {
              servoZero.setPosition(0);
              servoOne.setPosition(0);
          } else {
              servoZero.setPosition(0.5);
              servoOne.setPosition(0.5);
          }
          //I am just guessing on how this will look like (Dominic)
          if (servoZero.getPosition() == 0.5 && //and the robot arm is extended all the way (Don't know how to do code this) {
              while contServo.getPosition != 1 {
                  contServo.setPosition(1);
              }
          /*}else if (servoZero.getPosition == 0.5){
              contServo.setPosition(-1);
          }*/
          boolean upKey = gamepad1.dpad_up;
          boolean downKey = gamepad1.dpad_down;
          boolean leftKey = gamepad1.dpad_left;
          boolean rightKey = gamepad1.dpad_right;
          double leftTurn = gamepad1.left_trigger;
          double rightTurn = gamepad1.right_trigger;
          double cutoff = 0.5;


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
