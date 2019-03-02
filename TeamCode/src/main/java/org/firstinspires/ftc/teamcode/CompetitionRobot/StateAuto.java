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

            // Drive Robot Forward 500 Ticks To Bump Wall
            moveDriveEncoder(500, 500, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            deployArm(3000);

            deployMarker();

            // Drive Robot Forward 500 Ticks To Bump Wall
            moveDriveEncoder(200, 200, .5);
            while(driveMotorsBusy() && !isStopRequested()){
                telemetry.addData("Status", "Driving Forward");
                telemetry.update();
            }
            setDriveMotors(0);

            strafe(-3000);

        }
        else if (goldPos == 'C') {

        }
        else if (goldPos == 'R') {

        }

        // Reset Lift Motor To Original Size
        // lowerLiftMotor();
    }
}
