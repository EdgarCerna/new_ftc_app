package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "State Autonomous", group = "Autonomous")
//@Disabled
public class StateAuto extends Auto_Routines {
    @Override
    public void runOpMode() {
        // Initialize Everything...
        Auto_Init();

        // Raise Lift Motor To Release Latch
        raiseLiftMotor();

        // Drive Robot Forward 500 Ticks To Free From Latch
        moveDriveEncoder(500, 500, .2);
        while(driveMotorsBusy() && !isStopRequested()){
            telemetry.addData("Status", "Driving Forward");
            telemetry.update();
        }
        setDriveMotors(0);

        // PID DRIVE TO GOLD
        stopResetDriveEncoders();
        while (robot.frontLeftDrive.getCurrentPosition() < 2500 && robot.frontRightDrive.getCurrentPosition() < 2500) {
            pidDriveTowardGold();
        }

        // TURN TO DEPOT BASED ON goldPos
        if (goldPos == 'L') {
            // Drive Robot Forward 1200 Ticks To Bump Gold
            moveDriveEncoder(1200, 1200, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Drive Robot Backward 1000 Ticks Away From Gold
            moveDriveEncoder(-1200, -1200, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Turn Towards Wall To Avoid Minerals
            turnUntilAngle(90);

            // Drive Robot Forward 4000 Ticks Towards Wall
            moveDriveEncoder(4000, 4000, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Turn Right To Line Up With Wall
            turnUntilAngle(45);

            // Drive Robot Forward 1100 Ticks To Bump Wall
            moveDriveEncoder(1100, 1100, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            deployArm(5500);

            deployMarker();

            strafe(-4800, 0.4);
        }
        else if (goldPos == 'C') {
            // DRIVE BACKWARD 1000 ENCODER TICKS TO MOVE AWAY FROM MINERALS
            stopResetDriveEncoders();
            moveDriveEncoder(-1200, -1200, .6);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backwards away from TM Drop Off");
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
            }
            setDriveMotors(0);

            // Turn Towards Wall To Avoid Minerals
            turnUntilAngle(90);

            // Drive Robot Forward 4000 Ticks Towards Wall
            moveDriveEncoder(4500, 4500, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Turn Right To Line Up With Wall
            turnUntilAngle(45);

            // Drive Robot Forward 1500 Ticks Towards Wall
            moveDriveEncoder(1500, 1500, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            deployArm(5300);

            deployMarker();

            strafe(-4700, 0.4);
        }
        else if (goldPos == 'R') {
            // Drive Robot Forward 1000 Ticks To Bump Gold
            moveDriveEncoder(1000, 1000, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Drive Robot Backward 1000 Ticks Away From Gold
            moveDriveEncoder(-1000, -1000, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Backward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Turn Towards Wall To Avoid Minerals
            turnUntilAngle(90);

            // Drive Robot Forward 4000 Ticks Towards Wall
            moveDriveEncoder(5000, 5000, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            // Turn Right To Line Up With Wall
            turnUntilAngle(45);

            // Drive Robot Forward 1600 Ticks Towards Wall
            moveDriveEncoder(1600, 1600, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            deployArm(5400);

            deployMarker();

            strafe(-4800, 0.4);
        }
    }
}
