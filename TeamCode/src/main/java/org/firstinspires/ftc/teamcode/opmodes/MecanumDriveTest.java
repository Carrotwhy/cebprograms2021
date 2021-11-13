package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@TeleOp
public class MecanumDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        Slide mySlide = new Slide(robot);
        robot.registerSubsystem(mecanumDrive);
        robot.registerSubsystem(mySlide);
        int slidecountup = 0;
        int slidecountdown = 0;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            mecanumDrive.setDrivePower(new Pose2d (-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            if (gamepad1.left_bumper && slidecountup < 3) {
                mySlide.setPower(0.2);
                slidecountup = slidecountup + 1;
            }
            else {
                if (slidecountdown < 3) {
                    mySlide.setPower(-gamepad1.left_trigger);
                    slidecountdown = slidecountdown + 1;
                }

            }
        }
    }
}