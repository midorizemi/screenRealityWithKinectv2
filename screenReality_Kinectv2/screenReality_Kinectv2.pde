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
//for objectMovemnent
boolean movement = false;  //move or not
boolean direction = true;  //true : downward  false : upward
int dx = -1;         //to the home position
int dz = -1;         //to the home position
int dy = 1;          //downward movement
//for sphere position
int x = 40;
int y = 0;
int z = -20;

GO_OPERATION gopr = GO_OPERATION.S;

boolean isKinectEnabled =true;
boolean isWipeDisplayed = true;

color hDefo = color(255, 255, 255, 150);
color hOpen = color(255, 0, 0, 150);
color hClose = color(0, 0, 255, 150);
color hLasso = color(255, 255, 0, 150);
color hrStatusCol = hDefo;
color hlStatusCol = hDefo;

void setup() {
  size(1228, 928, P3D);
  //fullScreen(P3D);
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
  //eyeZ = (eyeY/2)/tan(PI/6);
  eyeZ = 600;
  cX = width/2.0;
  cY = height/2.0;
  cZ = 0;
  upX = 0;
  upY = 1;
  upZ = 0;

  beginCamera();
  //camera(eyeX, eyeY, eyeZ, cX, cY, cZ, upX, upY, upZ);
  camera();
  endCamera();
}
void draw() {
  ambientLight(150, 150, 150); 
  lightSpecular(255, 255, 255);
  directionalLight(100, 100, 100, 0, 1, -1);
  background(0);
  drawWipe();
  if (keyPressed == true && !isKinectEnabled) {
    println("HOGE:"+key);
    switch(key) {
    case 'a':
      eyeX -= 10;
      break;
    case 'd':
      eyeX += 10;
      break; //<>//
    case 'w':
      eyeY -= 10;
      break;
    case 'x':
      eyeY += 10; 
      break;
    case 'q':
      camera();
      break;
    }
    if (keyCode == UP) {
      eyeZ -= 10;
    } else if (keyCode == DOWN) {
      eyeZ += 10;
    }
  }
  detectHead();
  beginCamera();
  camera(eyeX, eyeY, eyeZ, cX, cY, cZ, upX, upY, upZ);
  endCamera();
  println("X:"+eyeX+ " Y:"+ eyeY+ " Z:"+ eyeZ);
  gopr = recognizeJesture();
  //for object
  object();
}

float cx;
float cy;
public static final int JOINT_HEAD = KinectPV2.JointType_Head;
public static final int JOINT_righthand = KinectPV2.JointType_HandRight;
public static final int JOINT_lefthand = KinectPV2.JointType_HandLeft;
public static final int JOINT_spineshoulder = KinectPV2.JointType_SpineShoulder;
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
      //float normHeadZ = rawHeadZ;
      float normHeadX = rawHeadX;
      float normHeadY = rawHeadY;
      float tempZ = normHeadZ;
      float tempX = normHeadX*400+width/2;
      float tempY = normHeadY*400-height/2+100;
      
      println("TMPZ:"+tempZ);
      eyeZ = 500+300*tempZ;
      eyeX = tempX ;
      eyeY = -tempY ;
    }
  }
}

void drawWipe() {
  if (isWipeDisplayed) {
    beginCamera();
    camera();
    endCamera();
    image(kinect.getColorImage(), 0, 0, 320, 240);
    pushMatrix();
    translate(320/2, 240/2, 0);
    rotateX(PI);
    ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();
    //individual JOINTS
    for (int i = 0; i < skeletonArray.size() && i < 1; i++) {
      KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
      if (skeleton.isTracked()) {
        KJoint[] joints = skeleton.getJoints();
        HState hr = handState(joints[KinectPV2.JointType_HandRight].getState());
        if (hr != HState.NOTRACKED) {
          hrStatusCol = getHandJestureColor(hr);
          fill(hrStatusCol);
          drawJoint(joints, KinectPV2.JointType_HandRight);
        }
        HState hl = handState(joints[KinectPV2.JointType_HandLeft].getState());
        if (hl != HState.NOTRACKED) {
          hlStatusCol = getHandJestureColor(hl);
          fill(hlStatusCol);
          drawJoint(joints, KinectPV2.JointType_HandLeft);
        }
      }
    }
    popMatrix();
  }
}
void drawJoint(KJoint[] joints, int jointType) {
  ellipse(joints[jointType].getX()*200, joints[jointType].getY()*200, 50, 50);
}

color getHandJestureColor(HState hs) {
  switch(hs) {
  case OPENED:
    return hOpen;
  case CLOSED:
    return hClose;
  case LASSO:
    return hLasso;
  default:
    return hDefo;
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
  for (int deg = 0; deg <= 360; deg = deg + 10) { //<>//
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
  beginShape(TRIANGLE_STRIP); //<>//
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
void object() {
  //MainBox
  int mainBoxX = 100;  //MainBox width
  int mainBoxY = 100;  //MainBox height
  int mainBoxZ = 80;   //MainBox depth
  int sphereRad = 4;   //radius of the sphere
  int thickness = 20;   //thickness of the keihin
  int breadth = 10;    //width of the keihin
  int num = 6;
  int keihinXZ[] = {-40, -20, //point(x,z) of the keihin
    0, -20, 
    40, -20, 
    -40, 20, 
    0, 20, 
    40, 20};  
  //for mainBox movement
  if (movement) {//Down operation
    //downward
    if (direction) {
      if (collisionDetection(sphereRad, y, breadth, thickness, keihinXZ, mainBoxY, num)) {  //collision detection
        direction = false;
      } else if (mainBoxY >= sphereRad + y) {  //move downward
        y += dy;
      } else {
        direction = false;
      }
    }
    //upward
    if (!direction) {
      if (sphereRad <= y) {  //move upward
        y -= dy;
      } else if (z <= -sphereRad + mainBoxZ / 2) {  //back to the home position(z)
        z -= dz;
      } else if (x >= sphereRad - mainBoxX / 2) {  //back to the home position(x)
        x += dx;
      } else {
        movement = false;
        direction = true;
      }
    }
    //TODO go to HOME position
  } else {
    //moveClane
    switch(gopr) {
    case R:
      x += 1;
      break;
    case F://no operation
      z -=1;
      break;
      // case S: //Stay
      //break;
    case L:
      x -= 1;
      break;
    case B:
      z += 1;
      break;
    case D:
      break;
    }
  }
  pushMatrix();
  translate(width/2, height/2, 0);
  scale(5);
  noFill();
  stroke(255);
  box(mainBoxX, mainBoxY, mainBoxZ);
  /*
  int lineNumber = 10;
   for(int i = 1; i <= lineNumber; i++){
   stroke(255);
   line(-((float)mainBoxX)/2 + i*((float)mainBoxX)/(lineNumber+1),((float)mainBoxY)/2,-mainBoxZ/2,
   -((float)mainBoxX)/2 + i*((float)mainBoxX)/(lineNumber+1),-((float)mainBoxY)/2,-mainBoxZ/2);
   
   line(-((float)mainBoxX)/2,-((float)mainBoxY)/2 + i*((float)mainBoxX)/(lineNumber+1),-mainBoxZ/2,
   ((float)mainBoxX)/2,-((float)mainBoxY)/2 + i*((float)mainBoxX)/(lineNumber+1),-mainBoxZ/2);
   }*/
  pushMatrix();
  fill(153, 255, 153);
  translate(keihinXZ[0], mainBoxY/2 - thickness / 2, keihinXZ[1]); //<>//
  box(breadth, thickness, breadth);
  popMatrix();
  pushMatrix();
  translate(keihinXZ[2], mainBoxY/2 - thickness / 2, keihinXZ[3]);
  box(breadth, thickness, breadth);
  popMatrix();
  pushMatrix();
  translate(keihinXZ[4], mainBoxY/2 - thickness / 2, keihinXZ[5]);
  box(breadth, thickness, breadth);
  popMatrix();
  pushMatrix();
  translate(keihinXZ[6], mainBoxY/2 - thickness / 2, keihinXZ[7]);
  box(breadth, thickness, breadth);
  popMatrix();
  pushMatrix();
  translate(keihinXZ[8], mainBoxY/2 - thickness / 2, keihinXZ[9]);
  box(breadth, thickness, breadth);
  popMatrix();
  pushMatrix();
  translate(keihinXZ[10], mainBoxY/2 - thickness / 2, keihinXZ[11]); //<>//
  box(breadth, thickness, breadth);
  popMatrix();
  //Sphere
  pushMatrix();
  translate(x, -mainBoxY/2 + sphereRad + y, z);
  sphere(sphereRad);
  popMatrix();
  popMatrix();
}
/*Detect collision of object and sphere.*/
boolean collisionDetection(int sphereRad, int y, int breadth, int thickness, int[] keihinXZ, int mainBoxY, int num) {
  for (int i=0; i<num; i++) {
    if (x <= keihinXZ[2*i] + breadth && x >= keihinXZ[2*i] - breadth &&
      y >= mainBoxY - thickness - sphereRad &&
      z <= keihinXZ[2*i+1] + breadth && z >= keihinXZ[2*i+1] - breadth) {
      print("1");
      return true;
    }
  }
  return false;
}
enum HState {
  OPENED, CLOSED, LASSO, NOTRACKED
};
enum GO_OPERATION {
  R, L, F, B, D, S
}//operation of movement;right, left, forward, backward, down and stay.
GO_OPERATION recognizeJesture() {
  GO_OPERATION go = GO_OPERATION.S;
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();
  //individual JOINTS
  for (int i = 0; i < skeletonArray.size() && i < 1; i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();
      KJoint hRight = joints[KinectPV2.JointType_HandRight];
      KJoint hLeft = joints[KinectPV2.JointType_HandLeft];
      HState hrStatus = handState(hRight.getState());
      HState hlStatus = handState(hLeft.getState());


      float righthandY = joints[JOINT_righthand].getY();
      float lefthandY = joints[JOINT_lefthand].getY();
      float spineshoulderY = joints[JOINT_spineshoulder].getY();

      if (righthandY > spineshoulderY) {
        switch(hrStatus) {
        case OPENED:
          go = GO_OPERATION.R; //<>//
          break;
        case CLOSED:
          go = GO_OPERATION.F;
          break;
        case LASSO:
          go = GO_OPERATION.D;
          movement=true;
          break;
        }
      } else if (lefthandY > spineshoulderY) {
        switch(hlStatus) {
        case OPENED:
          go = GO_OPERATION.L;
          break;
        case CLOSED:
          go = GO_OPERATION.B;
          break;
        case LASSO:
          go = GO_OPERATION.D;
          movement=true;
          break;
        }
      } else {
        break;
      }
    }
  }
  return go;
}
/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */
HState handState(int handState) {
  HState status;
  switch(handState) {
  case KinectPV2.HandState_Open:
    status = HState.OPENED;
    break;
  case KinectPV2.HandState_Closed:
    status = HState.CLOSED;
    break;
  case KinectPV2.HandState_Lasso:
    status = HState.LASSO;
    break;
  case KinectPV2.HandState_NotTracked:
    status = HState.NOTRACKED;
    break;
  default:
    status = HState.NOTRACKED;
  }
  return status;
}