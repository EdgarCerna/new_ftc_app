package org.firstinspires.ftc.teamcode.CompetitionRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@Autonomous(name = "DeployArm", group = "Autonomous")
//@Disabled
public class testDeployArm extends Auto_Routines {
    @Override
    public void runOpMode() {
        // Initialize Everything...
        Auto_Init();

        hoverArm(100, 0.3);
        while(robot.armMotor.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "Lifting Arm");
            telemetry.addData("Encoder:", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.armMotor.setPower(0);
        //deployArm(3500);
    }
}
