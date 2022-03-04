// Rotate - Profiled pidcontroller with trap profile controls heading angle.
//          PID output is the rotaitonal velocity omega, which becomes the input
//          to chassis speeds and module kinematics calculations
//
// Velocity - Speeds vx and vy are used in kinematics calcs to
//            determine module velocity setpoint. Module velocity controlled by
//            controller's onboard velocity mode.  Currently no separate 
//            FeedForward object is used (only kF set on the controller),
//            or voltage compensation or other constraints.

// Turn   -   This is the module angle calculated by kinematics and optimized
//            to determine smalled angle change needed.  Currently controlled by 
//            controller's onboard postiion mode with kP and kD.
//            and no constraints or profile.  
//            Should be changed to SmartMotion with a trap profile and external
//            feedforward object providing the feedforward component.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.Pigeon;
import frc.robot.Utilities.ModuleGenerator;
import frc.robot.Utilities.Util;

//import static frc.robot.Constants.Constants_Swerve.*;



public class SwerveDriveSystem  extends SubsystemBase {
  private ModuleGenerator generator = new ModuleGenerator();

  private SwerveModuleMDK[] modules = generator.generateModuleMDK();

  // Rotate (omega) Constants
    public static double kP_rotate=4.0;
    public static double kD_rotate=.1;
    private double kS_rotate=0.0;
    private double AllowErr_rotate=0.0;
    private double TrapMaxVel_rotate=30;
    private double TrapMaxAcc_rotate=20;
    private double rotateDPS=225;
    private double rotateRP20msec=rotateDPS*Math.PI/(50.0*180.0);


    // Robot Dimensions for MK4 Swerve
    private  double  maxVelocityFPS = 14.2;  //max speed in feet/sec
    private double maxVelocityMPS = 0.3048*maxVelocityFPS; // 4.328     

    public static  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(0.4064, -0.4064), 
      new Translation2d(0.4064, +0.4064), 
      new Translation2d(-0.4064, -0.4064), 
      new Translation2d(-0.4064, +0.4064));
      

  private final Pigeon gyro = new Pigeon(20);
  public SwerveModuleState[] moduleStatesOptimized = new SwerveModuleState[4];
  public double heading=gyro.getHeadingRadians();

  public SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics,new Rotation2d(heading));
  private Tables datatable;  
  private double[] encPositionRad = new double[4];   // encoder position of swerve motors
  private double[] speedSet = new double[4];  // speed setppiont for driver motors in mps
  private double[] rotationSetPrev={0,0,0,0};
  private double[] turnSet = new double[4]; // position setpoint for swerve motors     
  private String[] moduleNames={"FL","FR","BL","BR"};
  private double headingset=0;
  private double previousTurnMode = 2;

  private boolean inDeadband=false;
  private double Pi=Math.PI;
  private double twoPi=2*Math.PI;
 


// PID controller to control rotation of the robot. It will follow a trapezoidal profiel,
// the setpot is robot heading, sensor value is the gyro angle, and the output feeds into the 
// omega (rotational velocity in rad/sec) of the chassis speeds object.
  private TrapezoidProfile.Constraints trapProf =
    new TrapezoidProfile.Constraints(TrapMaxVel_rotate,TrapMaxAcc_rotate);
    
  private ProfiledPIDController pidRotate =
    new ProfiledPIDController(kP_rotate, 0, kD_rotate,trapProf);

 // Constrcutor 
  public SwerveDriveSystem(Tables m_datatable){
    datatable=m_datatable;
    datatable.putNumber("heading", 0);
    pidRotate.enableContinuousInput(-Pi, Pi);
    pidRotate.setTolerance(AllowErr_rotate);

    SmartDashboard.putNumber("kP_Rotate",kP_rotate);                    
    SmartDashboard.putNumber("kD_Rotate",kD_rotate);                   
    SmartDashboard.putNumber("kD_Rotate",kD_rotate);                    
    SmartDashboard.putNumber("kS_Rotate",kS_rotate);                    
    SmartDashboard.putNumber("AllErr_Rotate",AllowErr_rotate);                    
    SmartDashboard.putNumber("TrapMaxVel_Rotate",TrapMaxVel_rotate);
    SmartDashboard.putNumber("TrapMaxAcc_Rotate",TrapMaxAcc_rotate);
    SmartDashboard.putNumber("rotateDPS",rotateDPS);
    SmartDashboard.putNumber("rotateRP20msec",rotateRP20msec);  
    SmartDashboard.putNumber("Max Vel FPS",maxVelocityFPS);
    SmartDashboard.putNumber("Max Drive RPM",modules[0].MPStoRPM(maxVelocityMPS));


  }      
      
//
//  Calculate the chassis speeds input from joystick input
//  Do the field centric calculation, use the kinematics oject to 
// determine module states - with field centric and normalized speeds
//
// StickState Array:  vx,vy,omega or heading setpoint,flag
//  if flag=1, omega in rad/sec is supplied
// if flag = 2, an omega in rad/20ms is supplied from joystick and new heading setpoint is calculated
// if flag=3, a new heading setpoint is supplied
// for cases 2 and 3, a pid controller is used to calculate the necessary omega from 
// the current heading and heading setpoint
  public void setMotors(double[] stickState ) {
    inDeadband=false;
    if(stickState[3]==2){
      if ((Math.abs(stickState[0])+Math.abs(stickState[1]) <0.03) && Math.abs(stickState[2])<0.13) {
        stickState[0]=0;
        stickState[1]=0;
        stickState[2]=0;
        inDeadband=true;
      }
      if (Math.abs(stickState[2])<0.13) stickState[2]=0;
    }
  double vx=-stickState[0]*maxVelocityMPS;
  double vy=stickState[1]*maxVelocityMPS;
  double omega=0;

  datatable.putNumber("heading", Util.toDegrees(heading));
  datatable.putNumber("headingSet",  Util.toDegrees(headingset));
 
// omega directly supplied - could be used by camera follower
if (stickState[3]==1) omega=stickState[2];

  // omega from teleop stick input, use it to determnine new heading setpoint. 
  if (stickState[3]==2){
    headingset=headingset-stickState[2]*rotateRP20msec;
    // scale to +/- Pi radians
    if(headingset>Pi) headingset = headingset - twoPi;
    if(headingset<-Pi)headingset=headingset+ twoPi;
  }
  // Heading setpoint directly supplied, need to reset the rotation pid
  if(stickState[3]==3 && previousTurnMode !=3)
  {
    headingset=stickState[3];    
    pidRotate.reset(heading, 0); 
    pidRotate.enableContinuousInput(-Pi, Pi);
    headingset=stickState[2];
  }
// calculate the required rotational rate in radians/sec using the current
// heading from the gyro and the desired heading setpoint`
  if (stickState[3]==2||stickState[3]==3){  
    double pidout=pidRotate.calculate(heading,headingset);
    pidout=pidout+Math.signum(pidout)*kS_rotate;
    if (!pidRotate.atSetpoint()) omega=pidout;
    else omega=0;
  }

  // Rotate the calculated angle 90 degrees CCW by supplying vy,-vx instead of vx,vy
  // This makes 0 degrees straight up on the stick
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    vy, vx, omega,  new Rotation2d(heading));

// Convert to speeds module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
//  printModuleStates();
  previousTurnMode=stickState[3];
}



public void setModuleStates(SwerveModuleState[] moduleStates){
  // normalize wheel speed so no wheel exceeds max allowable velocity
  SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,maxVelocityMPS);
  int i = 0;
  while(i<4){
    // get the current turn encoder position in radians
    encPositionRad[i]=modules[i].getTurnPosition_Rad();
    // optimize module state to minimize the turn rotation needed
    moduleStatesOptimized[i]=Util.optimize(moduleStates[i],encPositionRad[i]);
    // calculate the drive motor's setpoint in rpm 
    speedSet[i]=moduleStatesOptimized[i].speedMetersPerSecond;
    // calculate the turn motor's rotation setpoint radians
    turnSet[i]=moduleStatesOptimized[i].angle.getRadians();
    // set the drive and turn motors
    /*
    if(inDeadband){
      rotationsSet[i]=rotationSetPrev[i];
    }  
    else rotationSetPrev[i]=rotationsSet[i];

    */
     modules[i].setMotors(speedSet[i], turnSet[i]);
    i++;  
  }
}


  public void headingBumpCCW(){
    headingset=headingset+0.06;
  }

  public void headingBumpCW(){
    headingset=headingset-0.06;
  }

// Write encoder values
  public void writeEncoders() {
    int i = 0;
    while(i<4){
      modules[i].writeEncoders();
      i++;
    }
  }

// a bunch of getters - so that the everything except the SwerveModules class can be 
// independant of the type of motors being used
  

  public void resetEncoders() {
    int i = 0;
    while(i<4){
      modules[i].resetEncoders();
      i++;
    }
  }


  public void resetGyro() {
    gyro.resetAngle();
    headingset=0;
    pidRotate.reset(0,0);
    
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public void updateConstants() {
    kP_rotate= SmartDashboard.getNumber("kP_Rotate",kP_rotate);
    kD_rotate= SmartDashboard.getNumber("kD_Rotate",kD_rotate);
    kS_rotate= SmartDashboard.getNumber("kS_Rotate",kS_rotate);
    AllowErr_rotate= SmartDashboard.getNumber("AllErr_Rotate",AllowErr_rotate);
    TrapMaxVel_rotate= SmartDashboard.getNumber("TrapMaxVel_Rotate",TrapMaxVel_rotate);
    TrapMaxAcc_rotate= SmartDashboard.getNumber("TrapMaxAcc_Rotate",TrapMaxAcc_rotate);
    rotateDPS= SmartDashboard.getNumber("rotateDPS",rotateDPS);
    maxVelocityFPS= SmartDashboard.getNumber("Max Vel FPS",maxVelocityFPS);
      
    rotateRP20msec= rotateDPS*Math.PI/(50.0*180.0);
    trapProf=new TrapezoidProfile.Constraints(TrapMaxVel_rotate,TrapMaxAcc_rotate);
    pidRotate =new ProfiledPIDController(kP_rotate, 0,kD_rotate,trapProf);
    pidRotate.setTolerance(AllowErr_rotate);

    
    maxVelocityMPS = 0.3048*maxVelocityFPS; 
    SmartDashboard.putNumber("Max Drive RPM",modules[0].MPStoRPM(maxVelocityMPS));
/*
    int i=0;
    while(i<4){
      modules[i].updateConstants();
      i++;
    }
*/
  }



  public double getDriveVelocityMagnitude() {
    return Math.abs(modules[0].getDriveVelocity());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, new Rotation2d(heading));
  }


  @Override
  public void periodic() {
    heading=gyro.getHeadingRadians();
    try{
    m_odometry.update(
        new Rotation2d(heading),
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
        SmartDashboard.putNumber("pose-angle", m_odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("pose-x", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("pose-y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("module state 0 mps", modules[0].getState().speedMetersPerSecond);
      }
        catch(Exception e){
          
        }
  }



// print module values to smartdashboard
private void printModuleStates(){
int i=0;
while(i<4){
  datatable.putNumber(moduleNames[i]+" Opt Angle", 
  moduleStatesOptimized[i].angle.getDegrees());
  datatable.putNumber(moduleNames[i]+" Tpos_Uncorr",Util.toDegrees(encPositionRad[i]));
  datatable.putNumber(moduleNames[i]+" Tpos",Util.toDegrees(encPositionRad[i]));
  datatable.putNumber(moduleNames[i]+" TAbspos",modules[i].getTurnAbsPosition());
  datatable.putNumber(moduleNames[i]+" Tvel",modules[i].getTurnVelocity());
  datatable.putNumber(moduleNames[i]+" SP rot",turnSet[i]/twoPi); 
  datatable.putNumber(moduleNames[i]+" Dpos",modules[i].getDrivePosition());            
  datatable.putNumber(moduleNames[i]+" Dvel",modules[i].getDriveVelocity()); 
  datatable.putNumber(moduleNames[i]+" SP RPM", modules[i].MPStoRPM(speedSet[i]));
 
  i++;
}

}


}