#include <FaceTracker/Tracker.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <CoreGraphics/CGDirectDisplay.h>
#include <CoreGraphics/CGRemoteOperation.h>
#include <ApplicationServices/ApplicationServices.h>
#include <CoreFoundation/CoreFoundation.h>
#include <unistd.h>
#include <fcntl.h>


using namespace FACETRACKER;
typedef uint16_t CGKeyCode;
//=============================================================================

int FaceDirectionBaseY = 0;
void scroll(int num) {
  CGEventRef eve = CGEventCreateScrollWheelEvent(NULL, kCGScrollEventUnitLine, 1, num);
  CGEventPost( kCGHIDEventTap , eve ) ;
  CFRelease(eve) ;
}
void faceDirEstimationY ( cv::Mat &shape ) {
  int n = shape.rows/2;
  cv::Point templeL = cv::Point(shape.at<double>(2,0),shape.at<double>(2+n,0));
  cv::Point templeR = cv::Point(shape.at<double>(14,0),shape.at<double>(14+n,0));
  cv::Point nose = cv::Point(shape.at<double>(29,0),shape.at<double>(29+n,0));
  int faceDirValue = (templeR.y + templeL.y)/2 - nose.y;
  if( !FaceDirectionBaseY ) {
    FaceDirectionBaseY = faceDirValue;
  }
  else if ( FaceDirectionBaseY ){
    if (faceDirValue < FaceDirectionBaseY-30 || faceDirValue > FaceDirectionBaseY +30) {
      scroll (( faceDirValue - FaceDirectionBaseY )/10);
    }
  }
}

bool move = false;
void moveWindow(int which){
  move = true;
  int Key = 0;
  if(which > 0){//left
    Key = 123;
  }
  else{//right
    Key = 124;
  }
  CGEventSourceRef source = CGEventSourceCreate(kCGEventSourceStateCombinedSessionState);
  CGEventRef saveCommandDown = CGEventCreateKeyboardEvent(source, (CGKeyCode)Key, true);
  CGEventSetFlags(saveCommandDown, kCGEventFlagMaskCommand);
  CGEventRef saveCommandUp = CGEventCreateKeyboardEvent(source, (CGKeyCode)Key, false);

  CGEventPost(kCGAnnotatedSessionEventTap, saveCommandDown);
  CGEventPost(kCGAnnotatedSessionEventTap, saveCommandUp);

  CFRelease(saveCommandUp);
  CFRelease(saveCommandDown);
  CFRelease(source);
}

int faceDirEstimationX(cv::Mat &shape){
  int n = shape.rows/2;
  cv::Point nose0 = cv::Point(shape.at<double>(27,0),shape.at<double>(27+n,0));
  cv::Point nose1 = cv::Point(shape.at<double>(30,0),shape.at<double>(30+n,0));
  
  int dNose = nose0.x - nose1.x;
  if(dNose > 10 && move == false) {
    moveWindow(1); 
	return 0;
  }
  else if(dNose < -10 && move == false) {
    moveWindow(-1);
    return 0; 
  }
  else if(dNose > 10 || dNose < -10) {
    move = true;
	return 0;
  }
  else{
    move = false;
	return 1;
  }
}


void colorExtraction(cv::Mat* src, cv::Mat* dst,
    int code,
    int ch1Lower, int ch1Upper,
    int ch2Lower, int ch2Upper,
    int ch3Lower, int ch3Upper
    )
{
    cv::Mat colorImage;
    int lower[3];
    int upper[3];

    cv::Mat lut = cv::Mat(256, 1, CV_8UC3);

    cv::cvtColor(*src, colorImage, code);

    lower[0] = ch1Lower;
    lower[1] = ch2Lower;
    lower[2] = ch3Lower;

    upper[0] = ch1Upper;
    upper[1] = ch2Upper;
    upper[2] = ch3Upper;

    for (int i = 0; i < 256; i++){
        for (int k = 0; k < 3; k++){
            if (lower[k] <= upper[k]){
                if ((lower[k] <= i) && (i <= upper[k])){
                    lut.data[i*lut.step+k] = 255;
                }else{
                    lut.data[i*lut.step+k] = 0;
                }
            }else{
                if ((i <= upper[k]) || (lower[k] <= i)){
                    lut.data[i*lut.step+k] = 255;
                }else{
                    lut.data[i*lut.step+k] = 0;
                }
            }
        }
    }

    //LUTを使用して二値化
    cv::LUT(colorImage, lut, colorImage);

    //Channel毎に分解
    std::vector<cv::Mat> planes;
    cv::split(colorImage, planes);

    //マスクを作成
    cv::Mat maskImage;
    cv::bitwise_and(planes[0], planes[1], maskImage);
    cv::bitwise_and(maskImage, planes[2], maskImage);

    //出力
    cv::Mat maskedImage;
    src->copyTo(maskedImage, maskImage);
    *dst = maskedImage;
}

int blinktime = 0;
void closeEyes( cv::Mat image, cv::Mat shape ) {
  int n = shape.rows/2;
  
  cv::Point eyes = cv::Point(shape.at<double>(36,0),shape.at<double>(36+n,0));
  cv::Point eyeblow = cv::Point(shape.at<double>(17,0),shape.at<double>(17+n,0));
  
  cv::Mat orgImg = image(cv::Rect( (eyes.x + eyeblow.x)/2 , (eyes.y + eyeblow.y)/2 , 200 , 100 ) ) ;
  cv::Mat blinkImg;
  colorExtraction( &orgImg, &blinkImg , CV_RGB2HSV, 90, 130, 60, 255, 0, 255 );
  cv::cvtColor( blinkImg, blinkImg , CV_RGB2GRAY ) ;
  cv::threshold( blinkImg, blinkImg , 1 , 255 , CV_THRESH_BINARY ) ;
  cv::dilate( blinkImg, blinkImg , cv::Mat() , cv::Point( -1 , -1 ) ,3 );
  cv::erode( blinkImg, blinkImg , cv::Mat() , cv::Point( -1 , -1 ) ,3 );
  imshow("eyes",blinkImg);

  int eyeSpace = 0;
  for( int y = 0; y < blinkImg.rows ; y++) {
    for( int x = 0; x < blinkImg.cols ; x++) {
      if( blinkImg.data[ y*blinkImg.step + x*blinkImg.elemSize() ] == 0 ){
        eyeSpace++;
      }
    }
  }
  if( eyeSpace==0) { //blink
    blinktime++;
    //printf("eye blink\n" );
  }
  else{
    blinktime = 0;
  }
  
  if(blinktime > 50){
    CGEventSourceRef source = CGEventSourceCreate(kCGEventSourceStateCombinedSessionState);
    CGEventRef saveCommandDown = CGEventCreateKeyboardEvent(source, (CGKeyCode)0x0c, true);
    CGEventSetFlags(saveCommandDown, kCGEventFlagMaskCommand);
    CGEventRef saveCommandUp = CGEventCreateKeyboardEvent(source, (CGKeyCode)0x0c, false);

    CGEventPost(kCGAnnotatedSessionEventTap, saveCommandDown);
    CGEventPost(kCGAnnotatedSessionEventTap, saveCommandUp);

    CFRelease(saveCommandUp);
    CFRelease(saveCommandDown);
    CFRelease(source);
  }
}


void Draw(cv::Mat &image, cv::Mat &shape, cv::Mat &con, cv::Mat &tri, cv::Mat &visi)
{
  int i,n = shape.rows/2;
  cv::Point p1,p2;
  cv::Scalar c;

  //draw triangulation
  c = CV_RGB(0,0,0);
  for(i = 0; i < tri.rows; i++){
    if(visi.at<int>(tri.at<int>(i,0),0) == 0 ||
       visi.at<int>(tri.at<int>(i,1),0) == 0 ||
       visi.at<int>(tri.at<int>(i,2),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
  }
  //draw connections
  c = CV_RGB(0,0,255);
  for(i = 0; i < con.cols; i++){
    if(visi.at<int>(con.at<int>(0,i),0) == 0 ||
       visi.at<int>(con.at<int>(1,i),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(con.at<int>(0,i),0),
		   shape.at<double>(con.at<int>(0,i)+n,0));
    p2 = cv::Point(shape.at<double>(con.at<int>(1,i),0),
		   shape.at<double>(con.at<int>(1,i)+n,0));
    cv::line(image,p1,p2,c,1);
  }
  //draw points
  for(i = 0; i < n; i++){
    if(visi.at<int>(i,0) == 0) continue;
    p1 = cv::Point(shape.at<double>(i,0),shape.at<double>(i+n,0));
    c = CV_RGB(255,0,0); cv::circle(image,p1,2,c);
  }return;
}
//=============================================================================
int parse_cmd(int argc, const char** argv,
	      char* ftFile,char* conFile,char* triFile,
	      bool &fcheck,double &scale,int &fpd)
{
  int i; fcheck = false; scale = 1; fpd = -1;
  for(i = 1; i < argc; i++){
    if((std::strcmp(argv[i],"-?") == 0) ||
       (std::strcmp(argv[i],"--help") == 0)){
      std::cout << "track_face:- Written by Jason Saragih 2010" << std::endl
	   << "Performs automatic face tracking" << std::endl << std::endl
	   << "#" << std::endl
	   << "# usage: ./face_tracker [options]" << std::endl
	   << "#" << std::endl << std::endl
	   << "Arguments:" << std::endl
	   << "-m <string> -> Tracker model (default: ../model/face2.tracker)"
	   << std::endl
	   << "-c <string> -> Connectivity (default: ../model/face.con)"
	   << std::endl
	   << "-t <string> -> Triangulation (default: ../model/face.tri)"
	   << std::endl
	   << "-s <double> -> Image scaling (default: 1)" << std::endl
	   << "-d <int>    -> Frames/detections (default: -1)" << std::endl
	   << "--check     -> Check for failure" << std::endl;
      return -1;
    }
  }
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"--check") == 0){fcheck = true; break;}
  }
  if(i >= argc)fcheck = false;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-s") == 0){
      if(argc > i+1)scale = std::atof(argv[i+1]); else scale = 1;
      break;
    }
  }
  if(i >= argc)scale = 1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-d") == 0){
      if(argc > i+1)fpd = std::atoi(argv[i+1]); else fpd = -1;
      break;
    }
  }
  if(i >= argc)fpd = -1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-m") == 0){
      if(argc > i+1)std::strcpy(ftFile,argv[i+1]);
      else strcpy(ftFile,"../model/face2.tracker");
      break;
    }
  }
  if(i >= argc)std::strcpy(ftFile,"../model/face2.tracker");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-c") == 0){
      if(argc > i+1)std::strcpy(conFile,argv[i+1]);
      else strcpy(conFile,"../model/face.con");
      break;
    }
  }
  if(i >= argc)std::strcpy(conFile,"../model/face.con");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-t") == 0){
      if(argc > i+1)std::strcpy(triFile,argv[i+1]);
      else strcpy(triFile,"../model/face.tri");
      break;
    }
  }
  if(i >= argc)std::strcpy(triFile,"../model/face.tri");
  return 0;
}
//=============================================================================
int loop = 0;
int main(int argc, const char** argv)
{
  //parse command line arguments
  char ftFile[256],conFile[256],triFile[256];
  bool fcheck = false; double scale = 1; int fpd = -1; bool show = true;
  if(parse_cmd(argc,argv,ftFile,conFile,triFile,fcheck,scale,fpd)<0)return 0;

  //set other tracking parameters
  std::vector<int> wSize1(1); wSize1[0] = 7;
  std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
  int nIter = 5; double clamp=3,fTol=0.01;
  FACETRACKER::Tracker model(ftFile);
  cv::Mat tri=FACETRACKER::IO::LoadTri(triFile);
  cv::Mat con=FACETRACKER::IO::LoadCon(conFile);

  //initialize camera and display window
  cv::Mat frame,gray,im; double fps=0; char sss[256]; std::string text;
  cv::VideoCapture camera(CV_CAP_ANY); if(!camera.isOpened()) return -1;
  int64 t1,t0 = cvGetTickCount(); int fnum=0;
  cvNamedWindow("Face Tracker",1);
  std::cout << "Hot keys: "        << std::endl
	    << "\t ESC - quit"     << std::endl
	    << "\t d   - Redetect" << std::endl
      << "\t r   - Reset"    << std::endl;

  //loop until quit (i.e user presses ESC)
  bool failed = true;
  while(1){
    //grab image, resize and flip
    camera.read(frame);
    if(scale == 1)im = frame;
    else cv::resize(frame,im,cv::Size(scale*frame.cols,scale*frame.rows));
    cv::flip(im,im,1); cv::cvtColor(im,gray,CV_BGR2GRAY);

    //track this image
    std::vector<int> wSize; if(failed)wSize = wSize2; else wSize = wSize1;
    if(model.Track(gray,wSize,fpd,nIter,clamp,fTol,fcheck) == 0){
      int idx = model._clm.GetViewIdx(); failed = false;
      Draw(im,model._shape,con,tri,model._clm._visi[idx]);
    }else{
      if(show){cv::Mat R(im,cvRect(0,0,150,50)); R = cv::Scalar(0,0,255);}
      model.FrameReset(); failed = true;
    }
    //draw framerate on display image
    if(fnum >= 9){
      t1 = cvGetTickCount();
      fps = 10.0/((double(t1-t0)/cvGetTickFrequency())/1e+6);
      t0 = t1; fnum = 0;
    }else fnum += 1;

    if(show){
      sprintf(sss,"%d frames/sec",(int)round(fps)); text = sss;
      cv::putText(im,text,cv::Point(10,20),
		  CV_FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
    }
	closeEyes( im , model._shape );
	if( faceDirEstimationX( model._shape ) ){
      faceDirEstimationY( model._shape );
	}

    //show image and check for user input
    imshow("Face Tracker",im);
    loop++;
    if(loop >= 100){
      model.FrameReset();
      loop=0;
    }
    int c = cvWaitKey(10);
    if(c == 27)break;
    else if(char(c) == 'd')model.FrameReset();
    else if(char(c) == 'r'){
      model.FrameReset();
      FaceDirectionBaseY = 0;
    }
  }
  return 0;
}
//=============================================================================
