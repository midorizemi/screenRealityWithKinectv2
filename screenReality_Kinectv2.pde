import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;


float zVal = 300;
float rotX = PI;
int camWidth;
int camHeight;

void setup() {
  size(1024, 768, P3D);

  kinect = new KinectPV2(this);
  kinect.enableBodyTrackImg(true);
  kinect.enableColorImg(true);

  //enable 3d  with (x,y,z) position
  kinect.enableSkeleton3DMap(true);

  kinect.init();

  camWidth = kinect.getBodyTrackImage().width;
  camHeight = kinect.getBodyTrackImage().height;

  cx = pixelToCm(width);
  cy = pixelToCm(height);
  
  noStroke();
  
  beginCamera();
  camera(0,0,500,0,0,0,0,1,0);
  endCamera();
}

void draw() {
  ambientLight(150, 150, 150);    //環境光を当てる
  lightSpecular(255, 255, 255);    //光の鏡面反射色（ハイライト）を設定
  directionalLight(100, 100, 100, 0, 1, -1);    //指向性ライトを設定
  background(0);
  
  detectHead();
  beginCamera();
  camera(camX, camY, camZ,0,0,0,0,1,0);
  endCamera();
  
  fill(255);
  box(100);
  
  noStroke();
  fill(255, 0,0);
  pillar(200, 10, 10);
  
  pushMatrix();
  rotateZ(HALF_PI);
  fill(0, 255,0);
  pillar(200, 10, 10);
  popMatrix();
  
  pushMatrix();
  rotateX(HALF_PI);
  fill(0, 0, 255);
  pillar(200, 10, 10);
  popMatrix();
}

float camZ=500;
float camX=0;
float camY=0;
float cx;
float cy;

public static final int JOINT_HEAD = KinectPV2.JointType_Head;
public static final float F = 500f;
public static final float PIXEL_NBR_PER_CM = 50.0f;
public static final float FAR = 60.0f;
public static final float NEAR = 1.0f;
public static final float RANGE_KINECT2 = 4.5f - 0.5f;

void detectHead() {
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();
  //individual JOINTS
  for (int i = 0; i < skeletonArray.size() && i < 1; i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();
      float rawHeadX = joints[JOINT_HEAD].getX();
      float rawHeadY = joints[JOINT_HEAD].getY();
      float rawHeadZ = joints[JOINT_HEAD].getZ();

      float normHeadZ = rawHeadZ / RANGE_KINECT2;
      float normHeadX = rawHeadX;
      float normHeadY = rawHeadY;

      float tempZ = normHeadZ*250;
      float tempX = normHeadX*500;
      float tempY = normHeadY*500;

      camZ = tempZ ;
      camX = tempX ;
      camY = -tempY ;
    }
  }
}

float pixelToCm(int size) {
  return (float) size/PIXEL_NBR_PER_CM;
}

void pillar(float length, float radius1, float radius2) {

  float x, y, z;
  pushMatrix();
  //upper base
  beginShape(TRIANGLE_FAN);
  y = -length / 2;
  vertex(0, y, 0);
  for (int deg = 0; deg <= 360; deg = deg + 10) {
    x = cos(radians(deg)) * radius1;
    z = sin(radians(deg)) * radius1;
    vertex(x, y, z);
  }
  endShape();
  //base
  beginShape(TRIANGLE_FAN);
  y = length / 2;
  vertex(0, y, 0);
  for (int deg = 0; deg <= 360; deg = deg + 10) {
    x = cos(radians(deg)) * radius2;
    z = sin(radians(deg)) * radius2;
    vertex(x, y, z);
  }
  endShape();
  //side
  beginShape(TRIANGLE_STRIP);
  for (int deg =0; deg <= 360; deg = deg + 5) {
    x = cos(radians(deg)) * radius1;
    y = -length / 2;
    z = sin(radians(deg)) * radius1;
    vertex(x, y, z);
    x = cos(radians(deg)) * radius2;
    y = length / 2;
    z = sin(radians(deg)) * radius2;
    vertex(x, y, z);
  }
  endShape();
  popMatrix();
}