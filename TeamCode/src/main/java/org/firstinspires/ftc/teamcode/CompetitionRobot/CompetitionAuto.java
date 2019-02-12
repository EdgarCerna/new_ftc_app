package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Competition Autonomous", group="Autonomous")
@Disabled
public class CompetitionAuto extends LinearOpMode {

    private GoldAlignDetector detector;
    CompetitionHardware robot = new CompetitionHardware();

    //GLOBAL VARIABLES
    static final double DRIVE_SPEED = 0.3;
    boolean newCommand = true;

    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        //
        //AUTONOMOUS START
        //

        int cnt = 0;

        moveLiftMotor(10690, 1);
        while(robot.liftMotor.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "liftMotor Raising");
            telemetry.update();
        }
        robot.liftMotor.setPower(0);

        moveDriveEncoder(500, 500, .5);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward");
            telemetry.update();
        }
        setDriveMotors(0);

        moveLiftMotor(-10690, 1);
        while(robot.liftMotor.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "liftMotor Lowering");
            telemetry.update();
        }
        robot.liftMotor.setPower(0);

        stopResetAllEncoders();
        while(robot.frontRightDrive.getCurrentPosition() < 2500)
        {
            telemetry.addData("EncoderCnt", robot.frontRightDrive.getCurrentPosition());
            telemetry.update();
            driveToGold();
        }

    }

    private void driveToGold(){
        //VARIABLES
        final int target = 320;
        //int centerCnt = 0;
        double x = detector.getXPosition();
        double e, eOld, de, ie = 0;
        double kp = 0.1, kd = 0, ki = 0;
        double leftPower, rightPower, turnSpeed;
        int cnt = 0;

        e = x - target;
        eOld = e;
        if (newCommand == true)
        {
            de = 0;
            newCommand = false;
        }
        else
        {
            de = e - eOld;  //difference
        }
        ie += e;            //integration

        turnSpeed = kp * e + kd * de + ki * ie;
        if (turnSpeed > 0.3)
        {
            turnSpeed = 0.3;
        }
        else if (turnSpeed < -0.3)
        {
            turnSpeed = -0.3;
        }
        if (turnSpeed > 0) {
            leftPower = DRIVE_SPEED + turnSpeed;
            rightPower = DRIVE_SPEED;
        }
        else
        {
            leftPower = DRIVE_SPEED;
            rightPower = DRIVE_SPEED - turnSpeed;
        }

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeftDrive.setPower(leftPower);
        robot.rearLeftDrive.setPower(leftPower);
        robot.frontRightDrive.setPower(rightPower);
        robot.rearRightDrive.setPower(rightPower);

//        telemetry.addData("leftPow", leftPower);
//        telemetry.addData("rightPow", rightPower);
//        telemetry.addData("e", e);
//        telemetry.update();
    }

    private void moveLiftMotor(int ticks, double speed){
        int lmPos = robot.liftMotor.getCurrentPosition() + ticks;
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(lmPos);
        robot.liftMotor.setPower(speed);
    }

    private void moveDriveEncoder(int ticksLeft, int ticksRight, double speed){
        int lfPose = robot.frontLeftDrive.getCurrentPosition() + ticksLeft;
        int lrPose = robot.rearLeftDrive.getCurrentPosition() + ticksLeft;
        int rfPos = robot.frontRightDrive.getCurrentPosition() + ticksRight;
        int rrPos = robot.rearRightDrive.getCurrentPosition() + ticksRight;

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setTargetPosition(lfPose);
        robot.rearLeftDrive.setTargetPosition(lrPose);
        robot.frontRightDrive.setTargetPosition(rfPos);
        robot.rearRightDrive.setTargetPosition(rrPos);

        robot.frontLeftDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);
    }

    private boolean driveMotorsBusy(){
        return robot.frontLeftDrive.isBusy() && robot.rearLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.rearRightDrive.isBusy();
    }

    private void stopResetAllEncoders(){
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setDriveMotors(double speed){
        robot.frontLeftDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);
    }
}
