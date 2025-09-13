package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class scorer {
    //SparkMax scoringMotor = new SparkMax(13, MotorType.kBrushless);
    Spark scoringMotor = new Spark(1);

    public scorer(){}
    
    public void runMotorForwards(){
        scoringMotor.set(0.9);
    }

    public void runMotorBackwards(){
        scoringMotor.set(-0.9);
    }

    public void stopMotor(){
        scoringMotor.stopMotor();
    }
}
