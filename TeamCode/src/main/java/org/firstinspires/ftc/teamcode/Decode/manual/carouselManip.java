package org.firstinspires.ftc.teamcode.Decode.manual;

import static org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs.mod;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Decode.Carousel;

@Configurable
@TeleOp
public class carouselManip extends OpMode{
    private DcMotor carouselMotor;
    private Servo lift;
    private Carousel carousel;
    private boolean canMoveCarousel = false,
            carouselMoving = false;
    private int mode = 0;
    @Override
    public void init(){
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel");
        lift = hardwareMap.get(Servo.class, "lift");
        carousel.assignMotor(carouselMotor);
        carousel.assignLift(lift);
    }

    @Override
    public void loop(){
        switch(mode){
            case 0:
                carouselMotor.setPower(gamepad1.right_stick_x);
                break;

            case 1:
                if(gamepad1.circle){
                    carousel.approachPosition();
                }

                if(gamepad1.leftBumperWasPressed()){
                    carousel.incrementPosition(-1 + mod(carousel.getPosition(), 1));
                }

                if(gamepad1.rightBumperWasPressed()){
                    carousel.incrementPosition(1 - mod(carousel.getPosition(), 1));
                }
                break;

            case 2:
                if(gamepad1.circle){
                    carousel.approachPosition();
                }

                if(gamepad1.leftBumperWasPressed()){
                    carousel.incrementPosition(-1);
                }

                if(gamepad1.rightBumperWasPressed()){
                    carousel.incrementPosition(1);
                }

                //Circle pressed and carousel not moving
                if(gamepad2.circle && !carouselMoving){
                    carousel.liftUp();
                    canMoveCarousel = false;
                }
                //Circle not pressed (doesn't matter if carousel is moving or not)
                else{
                    canMoveCarousel = carousel.liftDown();
                }

                //Try carousel movement
                if(canMoveCarousel) {
                    //Unwinding of carousel
                    if (Math.abs(carousel.getPosition()) >= 5) {
                        carousel.setPosition(carousel.getPosition() % 3);
                    }

                    //Moves and checks movement of carousel
                    carouselMoving = !carousel.approachPosition();
                }
                break;
        }

        if(gamepad1.xWasPressed()) mode = (mode + 1) % 3;
        if(gamepad1.squareWasPressed()) carousel.resetEncoding();

        telemetry.addData("carouselPos", carousel.getPosition());
        telemetry.addData("carouselTicks", carouselMotor.getCurrentPosition());
        telemetry.addData("canMoveCarousel?", canMoveCarousel);
        telemetry.addData("carouselMoving?", carouselMoving);
    }
}
