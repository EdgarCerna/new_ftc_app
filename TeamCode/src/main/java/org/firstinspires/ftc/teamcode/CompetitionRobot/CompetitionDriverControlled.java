package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="CompetitionDriverControlled", group="TeleOp")

public class CompetitionDriverControlled extends OpMode {

    //DECLARE OpMode MEMBERS
    CompetitionHardware robot = new CompetitionHardware();

    // GLOBAL VARIABLES
    static boolean modeToggle = true;

    //RUN CODE ONCE ON init()
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    //RUN ONCE ON start()
    @Override
    public void start() {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //LOOP ON start()
    @Override
    public void loop() {
        // gamepad1 VARIABLES
        double leftTrigger = -gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;
        double rightStickX = gamepad1.right_stick_x;
        double rightPowerT = leftTrigger + rightTrigger - rightStickX;
        double leftPowerT = leftTrigger + rightTrigger + rightStickX;
        // gamepad2 VARIABLES
        double armPower = gamepad2.right_trigger - gamepad2.left_trigger;

        // SET MODE NUMBER TO EITHER 0 OR 1
        if (gamepad1.b) {
            modeToggle = false;
        }
        else if (gamepad1.a) {
            modeToggle = true;
        }

        // ARM LIFT CODE
        robot.armMotor.setPower(armPower);

        // ROBOT LIFT CODE
        if (gamepad1.right_bumper)
            robot.liftMotor.setPower(1);
        else if (gamepad1.left_bumper)
            robot.liftMotor.setPower(-1);
        else
            robot.liftMotor.setPower(0);

        // DRIVING BASED ON modeNumber
        if (modeToggle) {
            telemetry.addData("DriveMode", "Standard");
            telemetry.update();
            // STRAFE CODE
            if (gamepad1.dpad_right) {
                robot.frontLeftDrive.setPower(1);
                robot.frontRightDrive.setPower(-1);
                robot.rearLeftDrive.setPower(-1);
                robot.rearRightDrive.setPower(1);
            }
            else if (gamepad1.dpad_left) {
                robot.frontLeftDrive.setPower(-1);
                robot.frontRightDrive.setPower(1);
                robot.rearLeftDrive.setPower(1);
                robot.rearRightDrive.setPower(-1);
            }
            else {
                robot.frontLeftDrive.setPower(leftPowerT);
                robot.rearLeftDrive.setPower(leftPowerT);
                robot.frontRightDrive.setPower(rightPowerT);
                robot.rearRightDrive.setPower(rightPowerT);
            }
        }
        else if (!modeToggle) {
            telemetry.addData("DriveMode", "Arm");
            telemetry.update();
            // STRAFE CODE
            if (gamepad1.dpad_right) {
                robot.frontLeftDrive.setPower(1);
                robot.frontRightDrive.setPower(1);
                robot.rearLeftDrive.setPower(1);
                robot.rearRightDrive.setPower(1);
            }
            else if (gamepad1.dpad_left) {
                robot.frontLeftDrive.setPower(-1);
                robot.frontRightDrive.setPower(-1);
                robot.rearLeftDrive.setPower(-1);
                robot.rearRightDrive.setPower(-1);
            }
            else {
                robot.frontLeftDrive.setPower(-rightPowerT);
                robot.rearLeftDrive.setPower(leftPowerT);
                robot.frontRightDrive.setPower(rightPowerT);
                robot.rearRightDrive.setPower(-leftPowerT);
            }
        }
    }

    //RUN ONCE ON stop()
    @Override
    public void stop() {
        robot.frontLeftDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.liftMotor.setPower(0);
    }
}
