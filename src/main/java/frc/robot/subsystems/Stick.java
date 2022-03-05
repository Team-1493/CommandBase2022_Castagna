// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stick extends SubsystemBase{
  private Joystick mystick;
  private boolean turbo=false;
  private double sfTurbo=1,sfSlow=0.85;
  private double scaleFactor=sfSlow;
  private double omega=0;
  public int stickNum=1;

  int xID=0;
  int yID=1;
  int omID=2;
  public Stick(int port){
    System.out.println("Initializing");
    mapStick(port);
  }

  // get the squared magnitude squared of the strafe stick
  public double getMagSquared() {
    double mag2=mystick.getMagnitude();
  return mag2*mag2;
  }


  public double[] getStickState(){
    double angle = mystick.getDirectionDegrees();
    double mag2=getMagSquared()*scaleFactor;
    double vx=-mag2*Math.sin(angle*Math.PI/180);
    double vy= -mag2*Math.cos(angle*Math.PI/180);
    omega =mystick.getRawAxis(omID);
    // flag=2 indicates we are sending a rotational rate 
    return new double[] {vx,vy,omega,2};  
  }

// maps the stick axes for different contollers
public void mapStick(int port){
  mystick=new Joystick(port);
  String stickName = mystick.getName();
  System.out.println("**** NAME = "+stickName);
  if (stickName.equals("HORIPAD S")) {
    xID=0;
    yID=1;
    omID=2;
    stickNum=1;
  }

  if  (stickName.equals("Wireless Xbox Controller") || stickName.contains("F310")){
    xID=0;
    yID=1;
    omID=2; 
    stickNum=0;
  }

  if  (stickName.contains("Bluetooth XINPUT")) {
    xID=0;
    yID=1;
    omID=4; 
    stickNum=0;
  }

  mystick.setXChannel(xID);
  mystick.setYChannel(yID);
}

public int getPOV(){
  int val=mystick.getPOV(0);
  if (val==270)val=45;
  if(val==90)val=-45;
  return val; 
}

public void turboToggle(){
  turbo=!turbo;
  if (turbo) scaleFactor=sfTurbo;
  else scaleFactor=sfSlow;
}

// need this to provide the Joystick instance to the button bindings
public Joystick getStick(){
  return this.mystick;
}
}