package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Locale;

abstract public class Auto_Routines extends LinearOpMode {
    CompetitionHardware robot = new CompetitionHardware();
    public GoldAlignDetector detector;

    //GLOBAL VARIABLES
    final double DRIVE_SPEED = 0.15;
    boolean newCommand = true;
    double eOld = 0;
    char goldPos = 'U';

    public void Auto_Init() {
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
    }

    public void moveLiftMotor(int ticks, double speed){
        int lmPos = robot.liftMotor.getCurrentPosition() + ticks;
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(lmPos);
        robot.liftMotor.setPower(speed);
    }
    
    public void raiseLiftMotor() {
        moveLiftMotor(12000, 1);
        while(robot.liftMotor.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "liftMotor Raising");
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
    }
    
    public void lowerLiftMotor() {
        moveLiftMotor(-12000, 1);
        while(robot.liftMotor.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "liftMotor Lowering");
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
    }

    public void moveDriveEncoder(int ticksLeft, int ticksRight, double speed){
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

        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Using Encoders");
            telemetry.update();
        }
        setDriveMotors(0);
    }

    public boolean driveMotorsBusy(){
        return robot.frontLeftDrive.isBusy() && robot.rearLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.rearRightDrive.isBusy();
    }

    public void stopResetDriveEncoders(){
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDriveMotors(double speed){
        robot.frontLeftDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void composeImuTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.gravity  = robot.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return robot.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(robot.gravity.xAccel*robot.gravity.xAccel
                                        + robot.gravity.yAccel*robot.gravity.yAccel
                                        + robot.gravity.zAccel*robot.gravity.zAccel));
                    }
                });
    }

    public void turnUntilAngle(int angle) {
        do {
            //VARIABLES
            final int target = angle;
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double x = robot.angles.firstAngle;
            double e, de, ie = 0;
            double kp = 0.04, kd = 0, ki = 0;
            double leftPower, rightPower, turnSpeed;
            int cnt = 0;

            e = x - target;
            if (newCommand == true) {
                de = 0;
                newCommand = false;
            } else {
                de = e - eOld;  //difference
            }
            ie += e;            //integration

            eOld = e;
            turnSpeed = kp * e + kd * de + ki * ie;

            if (turnSpeed > 0.3) {
                turnSpeed = 0.3;
            } else if (turnSpeed < -0.3) {
                turnSpeed = -0.3;
            }


            leftPower = turnSpeed;
            rightPower = -turnSpeed;

            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontLeftDrive.setPower(leftPower);
            robot.rearLeftDrive.setPower(leftPower);
            robot.frontRightDrive.setPower(rightPower);
            robot.rearRightDrive.setPower(rightPower);

            telemetry.addData("Angle", x);
            telemetry.update();
        } while (robot.angles.firstAngle > angle + 1 || robot.angles.firstAngle < angle - 1);
    }

    public void strafe(int encoderCount, double speed) {
        stopResetDriveEncoders();
        int pos = robot.frontLeftDrive.getCurrentPosition() + encoderCount;

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setTargetPosition(pos);
        robot.rearLeftDrive.setTargetPosition(-pos);
        robot.frontRightDrive.setTargetPosition(-pos);
        robot.rearRightDrive.setTargetPosition(pos);

        robot.frontLeftDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);

        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Strafing");
            telemetry.update();
        }
        setDriveMotors(0);
    }

    public void deployArm(int encoderCount) {
        stopResetDriveEncoders();
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (robot.frontLeftDrive.getCurrentPosition() < encoderCount) {
            robot.frontLeftDrive.setPower(.3);
            robot.rearLeftDrive.setPower(-.3);
            robot.frontRightDrive.setPower(-.3);
            robot.rearRightDrive.setPower(.3);

            if (robot.frontLeftDrive.getCurrentPosition() < 1000)
                if (robot.frontLeftDrive.getCurrentPosition() < 500)
                    robot.armMotor.setPower(-0.6);
                else
                    robot.armMotor.setPower(-0.4);
            else
                robot.armMotor.setPower(-0.3);
        }
        setDriveMotors(0);
        robot.armMotor.setPower(0);
    }

    public void hoverArm(int time, double speed) {
        robot.armMotor.setPower(speed);
        sleep(time);
        robot.armMotor.setPower(0);
    }

    // NEEDS TO BE FINISHED
    public void noRightTM() {
        // PID DRIVE TO GOLD
        stopResetDriveEncoders();
        while (robot.frontLeftDrive.getCurrentPosition() < 2500 && robot.frontRightDrive.getCurrentPosition() < 2500) {
            pidDriveTowardGold();
        }

        // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
        stopResetDriveEncoders();
        moveDriveEncoder(2500, 2500, .6);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward To TM Drop Off");
            telemetry.addData("goldPos", goldPos);
            telemetry.update();
        }
        setDriveMotors(0);

        // TURN TO DEPOT BASED ON goldPos
        if (goldPos == 'R') {
            stopResetDriveEncoders();
            moveDriveEncoder(-2500, -2500, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);
        }
        else if (goldPos == 'L') {
            // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
            stopResetDriveEncoders();
            moveDriveEncoder(300, 300, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);

            // TURN RIGHT TO FACE DEPOT
            moveDriveEncoder(1800, -1800, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Turning To TM Drop Off");
                telemetry.update();
            }

            // DRIVE FORWARD INTO DEPOT
            stopResetDriveEncoders();
            moveDriveEncoder(1600, 1600, .4);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }
            setDriveMotors(0);

            //TURNS SERVO TO DROP TEAM MARKER
            deployMarker();
            sleep(300);
        }
        else if (goldPos == 'C') {
            //TURNS SERVO TO DROP TEAM MARKER
            deployMarker();
            sleep(300);
        }


        // TURN TO DEPOT BASED ON goldPos
        if (goldPos == 'R') {

        }
        else if (goldPos == 'L') {
            moveDriveEncoder(-1600, -1600, .4);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }

            // POTENTIALLY UNNEEDED TURN
//            stopResetDriveEncoders();
//            moveDriveEncoder(-1800, 1800, .5);
//            while(driveMotorsBusy() && !isStopRequested()){
//                telemetry.addData("Status", "Turning To TM Drop Off");
//                telemetry.update();
//            }

            // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
            stopResetDriveEncoders();
            moveDriveEncoder(-2500, -2500, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backwards away from TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);
        }
        else if (goldPos == 'C') {
            // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
            stopResetDriveEncoders();
            moveDriveEncoder(-2500, -2500, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backwards away from TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);
        }
    }

    // NEEDS TO BE FINISHED
    public void newTMRoutine() {
        // PID DRIVE TO GOLD
        stopResetDriveEncoders();
        while (robot.frontLeftDrive.getCurrentPosition() < 2500 && robot.frontRightDrive.getCurrentPosition() < 2500) {
            pidDriveTowardGold();
        }

        // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
        stopResetDriveEncoders();
        moveDriveEncoder(2500, 2500, .6);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward To TM Drop Off");
            telemetry.addData("goldPos", goldPos);
            telemetry.update();
        }
        setDriveMotors(0);

        // TURN TO DEPOT BASED ON goldPos
        if (goldPos == 'R') {
            // TURN LEFT TO FACE DEPOT
            moveDriveEncoder(-2000, 2000, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Turning To TM Drop Off");
                telemetry.update();
            }

            // DRIVE FORWARD INTO DEPOT
            stopResetDriveEncoders();
            moveDriveEncoder(2100, 2100, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }
            setDriveMotors(0);
        }
        else if (goldPos == 'L') {
            // TURN RIGHT TO FACE DEPOT
            moveDriveEncoder(1800, -1800, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Turning To TM Drop Off");
                telemetry.update();
            }

            // DRIVE FORWARD INTO DEPOT
            stopResetDriveEncoders();
            moveDriveEncoder(1600, 1600, .4);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }
            setDriveMotors(0);
        }

        //TURNS SERVO TO DROP TEAM MARKER
        deployMarker();
        sleep(300);


        // TURN TO DEPOT BASED ON goldPos
        if (goldPos == 'R') {
//            moveDriveEncoder(-2100, -2100, .5);
//            while(driveMotorsBusy() && !isStopRequested()){
//                telemetry.addData("Status", "Driving Forward To TM Drop Off");
//                telemetry.update();
//            }
//
//            // DRIVE FORWARD INTO DEPOT
//            stopResetDriveEncoders();
//            moveDriveEncoder(2000, -2000, .5);
//            while(driveMotorsBusy() && !isStopRequested()){
//                telemetry.addData("Status", "Turning To TM Drop Off");
//                telemetry.update();
//            }
//
//            // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
//            stopResetDriveEncoders();
//            moveDriveEncoder(-2500, -2500, .6);
//            while(driveMotorsBusy() && !isStopRequested()){
//                telemetry.addData("Status", "Driving Backwards away from TM Drop Off");
//                telemetry.addData("goldPos", goldPos);
//                telemetry.update();
//            }
//            setDriveMotors(0);
        }
        else if (goldPos == 'L') {
            moveDriveEncoder(-1600, -1600, .4);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }

            // POTENTIALLY UNNEEDED TURN
//            stopResetDriveEncoders();
//            moveDriveEncoder(-1800, 1800, .5);
//            while(driveMotorsBusy() && !isStopRequested()){
//                telemetry.addData("Status", "Turning To TM Drop Off");
//                telemetry.update();
//            }

            // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
            stopResetDriveEncoders();
            moveDriveEncoder(-2500, -2500, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backwards away from TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);
        }
        else if (goldPos == 'C') {
            // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
            stopResetDriveEncoders();
            moveDriveEncoder(-2500, -2500, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backwards away from TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);
        }
    }

    public void tmRoutine() {

        // PID DRIVE TO GOLD
        stopResetDriveEncoders();
        while (robot.frontLeftDrive.getCurrentPosition() < 2500 && robot.frontRightDrive.getCurrentPosition() < 2500) {
            pidDriveTowardGold();
        }

        // DRIVE FORWARD 2000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
        stopResetDriveEncoders();
        moveDriveEncoder(2500, 2500, .4);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward To TM Drop Off");
            telemetry.addData("goldPos", goldPos);
            telemetry.update();
        }
        setDriveMotors(0);

        // TURN TO DEPOT BASED ON goldPos
        if (goldPos == 'R') {
            // TURN LEFT TO FACE DEPOT
            moveDriveEncoder(-2000, 2000, .3);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Turning To TM Drop Off");
                telemetry.update();
            }

            // DRIVE FORWARD INTO DEPOT
            stopResetDriveEncoders();
            moveDriveEncoder(2100, 2100, .3);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }
            setDriveMotors(0);
        }
        else if (goldPos == 'L') {
            // TURN RIGHT TO FACE DEPOT
            moveDriveEncoder(1800, -1800, .3);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Turning To TM Drop Off");
                telemetry.update();
            }

            // DRIVE FORWARD INTO DEPOT
            stopResetDriveEncoders();
            moveDriveEncoder(1600, 1600, .2);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward To TM Drop Off");
                telemetry.update();
            }
            setDriveMotors(0);
        }

        //TURNS SERVO TO DROP TEAM MARKER
        deployMarker();
    }

    // UN-TESTED
    // MAY NEED TO DRIVE FORWARD SLIGHTLY MORE TO ACTUALLY HIT GOD MARKER AND THEN TO PARK ON THE CRATER
    public void craterRoutine() {

        // PID DRIVE TO GOLD
        stopResetDriveEncoders();
        while (robot.frontLeftDrive.getCurrentPosition() < 2500 && robot.frontRightDrive.getCurrentPosition() < 2500) {
            pidDriveTowardGold();
        }

    }

    public void deployMarker() {
        robot.tmServo.setPosition(1);
        sleep(1000);
    }

    public void pidDriveTowardGold(){
        //VARIABLES
        final int target = 320;
        double x = detector.getXPosition();
        double e, de, ie = 0;
        double kp = 0.002, kd = 0, ki = 0;
        double leftPower, rightPower, turnSpeed;
        int cnt = 0;

        e = x - target;
        if (newCommand == true) {
            de = 0;
            newCommand = false;

            if (e > 80) {
                goldPos = 'R';
            }
            if (e < -80) {
                goldPos = 'L';
            }
            if (e >= -80 && e <= 80) {
                goldPos = 'C';
            }

            telemetry.addData("goldPos", goldPos);
            telemetry.update();
        }
        else
        {
            de = e - eOld;  //difference
        }
        ie += e;            //integration

        eOld = e;
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
            rightPower = DRIVE_SPEED - turnSpeed / 2;
        }
        else
        {
            leftPower = DRIVE_SPEED + turnSpeed / 2;
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
//        telemetry.addData("de", de);
//        telemetry.addData("goldPos", goldPos);
//        telemetry.update();
    }
}
