package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "Auto")
public class NewAuto extends LinearOpMode {

    DcMotorEx m1, m2, m3, m4, arm, rotor;
    Servo handServo;
    BNO055IMU imu;
    DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    ColorSensor colorSensor;

    public void runOpMode(){
        m1 = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        m2 = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        m3 = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        m4 = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotor = hardwareMap.get(DcMotorEx.class, "rotor_motor");
        rotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo handServo = hardwareMap.get(Servo.class, "hand_servo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();

        // turn left
        m1.setPower(-1);
        m2.setPower(-1);
        m3.setPower(1);
        m4.setPower(1);
        sleep(850);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        // go back
        m1.setPower(-1);
        m2.setPower(-1);
        m3.setPower(-1);
        m4.setPower(-1);
        sleep(750);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        rotor.setPower(-1);
        sleep(1500);
        rotor.setPower(0);

        // turn left
        m1.setPower(0);
        m2.setPower(-1);
        m3.setPower(1);
        m4.setPower(0);
        sleep(300);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        // go straight
        m1.setPower(1);
        m2.setPower(1);
        m3.setPower(1);
        m4.setPower(1);
        sleep(1300);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        // turn right
        m1.setPower(1);
        m2.setPower(1);
        m3.setPower(-1);
        m4.setPower(-1);
        sleep(959);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        // go straight
        m1.setPower(1);
        m2.setPower(1);
        m3.setPower(1);
        m4.setPower(1);
        sleep(450);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        // arm
        arm.setPower(1);
        sleep(1000);
        arm.setPower(0);
        sleep(1000);

        // claw
        handServo.setPosition(0);
        sleep(1000);

        // arm
        arm.setPower(-1);
        sleep(1000);
        arm.setPower(0);
        sleep(1000);

        // turn left
        m1.setPower(-1);
        m2.setPower(-1);
        m3.setPower(1);
        m4.setPower(1);
        sleep(800);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);

        // go straight
        m1.setPower(1);
        m2.setPower(1);
        m3.setPower(1);
        m4.setPower(1);
        sleep(2000);

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(500);
    }
}
