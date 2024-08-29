package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants;



public SimulateDrivePid(){

    void Simulate(){
        var leftfollowermotorsim=m_leftFollowerMotor.getSimState();


     

    }

    void Simulate2(){
        var leftleadermotorsim=m_leftLeaderMotor.getSimState();
    }

    void Simulate3(){
        var rightleadermotorsim=m_rightLeaderMotor.getSimState();
    }

    void Simulate4(){
        var rightfollowermotorsim=m_rightFollowerMotor.getSimState();
    }
}

//1 motor sim object for each method, and 1 class for each method

//https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/simulation-intro.html