import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;


float zVal = 300;
float rotX = PI;
int camWidth;
int camHeight;

float eyeX;
float eyeY;
float eyeZ;
float cX;
float cY;
float cZ;
float upX;
float upY;
float upZ;

void setup() {
  //size(1228, 928, P3D);
  fullScreen(P3D);
  
  smooth();
  noFill();
  frameRate(24);

  kinect = new KinectPV2(this);
  kinect.enableBodyTrackImg(true);
  kinect.enableColorImg(true);
  kinect.enableColorImg(true);

  //enable 3d  with (x,y,z) position
  kinect.enableSkeleton3DMap(true);

  kinect.init();

  camWidth = kinect.getBodyTrackImage().width;
  camHeight = kinect.getBodyTrackImage().height;

  cx = pixelToCm(width);
  cy = pixelToCm(height);
  
  noStroke();
  
  eyeX = width/2.0;
  eyeY = height/2.0;
  eyeZ = 300;
  cX = width/2.0;
  cY = height/2.0;
  cZ = 0;
  upX = 0;
  upY = 1;
  upZ = 0;
  camera(width/2, height/2, 300,width/2,height/2,0,0,1,0);
  
  //beginCamera();
  //camera(0,0,500,0,0,0,0,1,0);
  //endCamera();
}

void draw() {
  
  ambientLight(150, 150, 150); 
  lightSpecular(255, 255, 255);
  directionalLight(100, 100, 100, 0, 1, -1);
  background(0);
  image(kinect.getColorImage(), 0, 0, 320, 240);
  
  detectHead();
  //beginCamera();
  //camera(camX, camY, camZ,0,0,0,0,1,0);
  //endCamera();
  
  //MainBox
  int mainBoxX = 100;  //MainBox width
  int mainBoxY = 100;  //MainBox height
  int mainBoxZ = 80;   //MainBox depth
  pushMatrix();
  translate(width/2, height/2, 0);
  noFill();
  stroke(255);
  box(mainBoxX,mainBoxY,mainBoxZ);
  
  /*
  int lineNumber = 10;
  for(int i = 1; i <= lineNumber; i++){
    stroke(255);
    line(-((float)mainBoxX)/2 + i*((float)mainBoxX)/(lineNumber+1),((float)mainBoxY)/2,-mainBoxZ/2,
         -((float)mainBoxX)/2 + i*((float)mainBoxX)/(lineNumber+1),-((float)mainBoxY)/2,-mainBoxZ/2);
    
    line(-((float)mainBoxX)/2,-((float)mainBoxY)/2 + i*((float)mainBoxX)/(lineNumber+1),-mainBoxZ/2,
         ((float)mainBoxX)/2,-((float)mainBoxY)/2 + i*((float)mainBoxX)/(lineNumber+1),-mainBoxZ/2);
  }*/
  
  int keihinXZ[] = {-40,-20,
                    0,-20,
                    40, -20,
                    -40, 20,
                    0, 20,
                    40, 20};
  
  pushMatrix();
  fill(153,255,153);
  translate(keihinXZ[0], mainBoxY/2 - 1, keihinXZ[1]);
  box(10,2,10);
  popMatrix();
  
  pushMatrix();
  translate(keihinXZ[2], mainBoxY/2 - 1, keihinXZ[3]);
  box(10,2,10);
  popMatrix();
  
  pushMatrix();
  translate(keihinXZ[4], mainBoxY/2 - 1, keihinXZ[5]);
  box(10,2,10);
  popMatrix();
  
  pushMatrix();
  translate(keihinXZ[6], mainBoxY/2 - 1, keihinXZ[7]);
  box(10,2,10);
  popMatrix();
  
  pushMatrix();
  translate(keihinXZ[8], mainBoxY/2 - 1, keihinXZ[9]);
  box(10,2,10);
  popMatrix();
  
  pushMatrix();
  translate(keihinXZ[10], mainBoxY/2 - 1, keihinXZ[11]);
  box(10,2,10);
  popMatrix();
  
  //Sphere
  pushMatrix();
  translate(-mainBoxX/2 + 2, -mainBoxY/2 + 2, mainBoxZ/2 - 2);
  sphere(2);
  popMatrix();
  
  popMatrix();
  
  if(keyPressed){
    switch(key){
    case 'a':
      eyeX -= 1.0;
      break;
    case 'd':
      eyeX += 1.0;
      break;
    case 'w':
      eyeY -= 1.0;
      break;
    case 'x':
      eyeY += 1.0; 
      break;
    case 'q':
      camera();
      break;
    }
    if(keyCode == UP){
      eyeZ -= 1.0;
    } else if(keyCode == DOWN){
      eyeZ += 1.0;
    }
    camera(eyeX, eyeY, eyeZ, cX, cY, cZ, upX, upY, upZ);
  }
}

float camZ=0.5;
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

      float tempZ = normHeadZ;
      float tempX = normHeadX*600;
      float tempY = normHeadY*600;

      camZ = 1/tempZ ;
      camX = -tempX ;
      camY = tempY ;
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