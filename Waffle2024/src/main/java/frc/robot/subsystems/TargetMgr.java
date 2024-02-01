package frc.robot.subsystems;

import java.util.ArrayList;

import frc.robot.objects.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetMgr {
    static ArrayList<TagTarget> targets=new ArrayList<>();

    static final public  int NO_TAGS=0;
    static final public  int FIELD_TAGS=1;

    static final public  int UNKNOWN=0;
    static final public  int OUTSIDE=1;
    static final public  int CENTER=2;
    static final public  int INSIDE=3;

    static final public  int RED=1;
    static final public  int BLUE=2;

    static final String[] pStrings={"???","OUTSIDE","CENTER","INSIDE"};
    static final String[] aStrings={"???","RED","BLUE"};

    public static double targetSize=0.1524; //0.371; side of inner rectangle of apriltag (meters)

    public static Translation3d field_center_offset=new Translation3d(325.4,157,0); 
   
    public static Translation3d robot_offset=new Translation3d(0,0,0);
    public static Rotation3d robot_rotation=new Rotation3d(0,0,0);
    public static Translation3d field_center=field_center_offset.times(Units.inchesToMeters(1));

    static int type=FIELD_TAGS;  // changed by init function

    static int alliance=UNKNOWN;
    static int position=UNKNOWN;

    static Pose2d start_pose=new Pose2d();
    static boolean start_pose_set=false;
    public static boolean show_tag_info=false;
    
    static boolean field_relative=true;
    static public void init(){ 
        setFieldTargets();  
    }
    public static boolean FRCfield(){
        return true;
    }
    static public void setFieldRelative(boolean b){
        field_relative=b;
    }
    public static void clearStartPose(){
        start_pose=new Pose2d();
        start_pose_set=false;
    }
    public static Pose2d startPose(){
        return start_pose;
    }
    public static Pose2d fieldToRobot(Pose2d r){
        return r.relativeTo(start_pose);
    }
    static boolean fieldRelative(){
        return field_relative;
    }
    Pose2d getFieldPose(Pose2d robotpose){
        return robotpose;     
    }
    public static int getAlliance(){
        return alliance;
    }
    public static int getStartPosition(){
        return position;
    }

    public static boolean startPoseSet(){
        return start_pose_set;
    }
    public static Translation2d getTagTranslation(int id){
        TagTarget target = getTarget(id);
        Translation2d tag_trans = target.getPose().getTranslation().toTranslation2d();
        if(start_pose_set)
            tag_trans=tag_trans.minus(start_pose.getTranslation());
        return tag_trans;
    }
    
    public static void setStartPose(AprilTag[] tags) {
        position = UNKNOWN;
        alliance = UNKNOWN;
        // start_pose_set=true;
        if (tags == null) 
            return;
        
        if (tags.length > 0) {
            AprilTag closest = tags[0];
            double dp = 0;
            if (tags.length > 1)
                dp = Math.abs(closest.getPitch() - tags[1].getPitch());
            // determine alliance and position
            switch (closest.getTagId()) {
            default:
                break;
            case 3: // offset tag red
                alliance = RED;
                position = INSIDE;
                break;
            case 4: // center tag red
                alliance = RED;
                if (dp > 2)
                    position = OUTSIDE;
                else if (dp > 0)
                    position = CENTER;
                break;
            case 7: // center tag blue
                alliance = BLUE;
                if (dp > 2)
                    position = INSIDE;
                else if (dp > 0)
                    position = CENTER;
                break;
            case 8: // offset tag blue
                alliance = BLUE;
                position = OUTSIDE;
                break;
            }
        }
        SmartDashboard.putString("Alliance", aStrings[alliance]);
        SmartDashboard.putString("Position", pStrings[position]);
    }
    static void setFieldTargets(){
        // data taken from inset in field drawing 4 of 6

        TagTarget tags[]={
            new TagTarget(1,593,9.68,53.68,120),
            new TagTarget(2,637,34.8,53.68,120),
            new TagTarget(3,652,196,57,180), // red speaker 
            new TagTarget(4,652,218,57,180), // red speaker middle
            new TagTarget(5,578,323,53,270),
            new TagTarget(6,72.5,323,53,270), 
            new TagTarget(7,-1.5,218,57,0), // blue speaker middle
            new TagTarget(8,-1.5,196,57,0), // blue speaker
            new TagTarget(9,14,34,53,60),
            new TagTarget(10,57,9.7,53,60),
            new TagTarget(11,468,146,52,300),
            new TagTarget(12,468,177,52,60),
            new TagTarget(13,441,161,52,180),
            new TagTarget(14,209,161,52,0),
            new TagTarget(15,182,177,52,120),
            new TagTarget(15,182,146,52,240),
        };

        for(int i=0;i<tags.length;i++)
            tags[i].scale(Units.inchesToMeters(1));

        for(int i=0;i<tags.length;i++){
           
            targets.add(tags[i]); 
        }
        if(show_tag_info){
        System.out.println("tag offsets from FRC origin");
        for(int i=0;i<targets.size();i++){
            Translation3d tag=targets.get(i).getTranslation();
            System.out.println("id:"+(i+1)+" "+getString(tag));
        }
    }
     
    }
    static String getString(Translation3d t){
        return String.format("x:%-2.2f y:%-2.2f z:%-2.2f",t.getX(),t.getY(),t.getZ());
    }
  
    static Translation3d fromField(Translation3d tt){
        return tt.minus(field_center);
    }
    static Translation2d fromField(Translation2d tt){
        return tt.minus(field_center.toTranslation2d());
    }
    
    static public int maxTargetId(){
       int i=numTargets();
       return i;
    }
    static public int minTargetId(){
       return 1;
    }
    static public int numTargets(){
        return 16;   
    }
    static public TagTarget getTarget(int i){
        int id=i-1;
        if(id<0 || id >targets.size())
            return null;
        return targets.get(id);
    }

    static boolean tagsPresent(){
        return type == NO_TAGS ?false:true;
    }
    static public class TagTarget {
        Pose3d targetPose;
        int targetID=0;
    
        public TagTarget( int id, Pose3d pose) {
            targetPose = pose;
            targetID = id;
        }
    
        public void scale(double inchesToMeters) {
            Translation3d t=getTranslation();
            t=t.times(inchesToMeters);
        }

        public TagTarget(int id, double x, double y, double z, double a) {
            targetPose = new Pose3d(x,y,z,new Rotation3d(0,0,a));
            targetID = id;
        }
        
        public int getID(){ 
            return targetID;
        }
        public Pose3d getPose(){
            return targetPose;
        }
        public Translation3d getTranslation(){
            return targetPose.getTranslation();
        }
        public Rotation3d getRotation(){
            return targetPose.getRotation();
        }
        public double getTargetSize(){
            return targetSize;
        }

        public TagTarget moveTo(Translation3d loc){
            Translation3d trans=loc.minus(targetPose.getTranslation());
            Pose3d pose=new Pose3d(trans,targetPose.getRotation());
            return new TagTarget(targetID,pose);
        }
        public String toString(){
            double angle=targetPose.getRotation().toRotation2d().getDegrees();
            return String.format("id:%d x:%-2.1f y:%2.1f z:%1.2f a:%3.1f",
            targetID,targetPose.getX(),targetPose.getY(),targetPose.getZ(),angle);
        }
       
    }
    
}