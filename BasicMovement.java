package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BasicMovement")
public class BasicMovement extends LinearOpMode {

    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_front;
    private DcMotor left_back;

    @Override
    public void runOpMode() {
        float vertical;
        float horizontal;
        float pivot;

        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");


        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;

            while (opModeIsActive()) {
                right_front.setPower(-pivot + (vertical - horizontal));
                right_back.setPower(-pivot + vertical + horizontal);
                left_front.setPower(pivot + vertical + horizontal);
                left_back.setPower(pivot + (vertical - horizontal));

                telemetry.update();
            }
        }
    }
}
