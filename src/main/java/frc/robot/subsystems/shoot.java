// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax; 









import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class shoot extends SubsystemBase {
  /** Creates a new Shooting. */
  SparkMax ShootingMotor;
  public shoot() {

    ShootingMotor = new SparkMax(Constants.OperatorConstants.kShootingMotorId, MotorType.kBrushless);
  }
  public void shoot(JoystickButton shoot){
    if(shoot.getAsBoolean()){
      ShootingMotor.set(1.0);
    }else{
      ShootingMotor.set(0.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}






































