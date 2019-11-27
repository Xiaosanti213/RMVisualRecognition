#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "mynteye/api/api.h"
#include "mynteye/logger.h"



MYNTEYE_USE_NAMESPACE


using namespace cv;
using namespace std;

float calcDistance(Point2f p1, Point2f p2)
{
  return pow(pow(p1.x-p2.x,2)+pow(p2.y-p2.y,2),0.5);
}

int detectRedThd = 200;

void paramInit()
{
  fstream fin;
  fin.open("/home/nvidia/yolo_ws/param.txt");
  //maybe failed
  string strTemp;
  while(getline(fin, strTemp))
  {
    switch(strTemp[0])
    {
      case 'b':
	fin >> detectRedThd;
	break;
      default:
	break;
    }
  }
}

int main(int argc, char **argv)
{
  // create mynt camera handle
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  // ros variable definition
  ros::init(argc, argv, "target_detect");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("camera/messages_test",1000);
  ros::Rate loop_rate(20);
  
  // mynt camera params
  Model model = api->GetModel();


  // param init
  paramInit();  


  // Set manual exposure options fo s1030
  if (model == Model::STANDARD) {
    std::int32_t frame_rate = 0;
    // manual-exposure: 1
    api->SetOptionValue(Option::EXPOSURE_MODE, 1);
    // gain: range [0,48], default 24
    api->SetOptionValue(Option::GAIN, 24);
    // brightness/exposure_time: range [0,240], default 120
    api->SetOptionValue(Option::BRIGHTNESS, 120);
    // contrast/black_level_calibration: range [0,255], default 127
    api->SetOptionValue(Option::CONTRAST, 127);

    frame_rate = api->GetOptionValue(Option::FRAME_RATE);

    LOG(INFO) << "Enable manual-exposure";
    LOG(INFO) << "Set EXPOSURE_MODE to "
              << api->GetOptionValue(Option::EXPOSURE_MODE);
    LOG(INFO) << "Set GAIN to " << api->GetOptionValue(Option::GAIN);
    LOG(INFO) << "Set BRIGHTNESS to "
              << api->GetOptionValue(Option::BRIGHTNESS);
    LOG(INFO) << "Set CONTRAST to " << api->GetOptionValue(Option::CONTRAST);
  }

  // Set manual exposure options fo S2000/S2100/S210A
  if (model == Model::STANDARD2 || model == Model::STANDARD210A) {
    // manual-exposure: 1
    api->SetOptionValue(Option::EXPOSURE_MODE, 1);

    // brightness/exposure_time: range [0,240], default 120
    api->SetOptionValue(Option::BRIGHTNESS, 20);

    LOG(INFO) << "Enable manual-exposure";
    LOG(INFO) << "Set EXPOSURE_MODE to "
              << api->GetOptionValue(Option::EXPOSURE_MODE);
    LOG(INFO) << "Set BRIGHTNESS to "
              << api->GetOptionValue(Option::BRIGHTNESS);
  }


  // video stream request
  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  api->EnableStreamData(Stream::LEFT_RECTIFIED);
  api->EnableStreamData(Stream::RIGHT_RECTIFIED);

  api->Start(Source::VIDEO_STREAMING);

  double fps;
  double t = 0.01;
  //std::cout << "fps:" << std::endl;

  cv::namedWindow("Origin", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);

  /*
  //set color threashold
  int iLowH = 100, iHighH = 140;//0-179
  int iLowS = 90, iHighS = 255;//0-255
  int iLowV = 90, iHighV = 255;//0-255

  cvCreateTrackbar("LowH", "Control", &iLowH, 179);
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  cvCreateTrackbar("LowS", "Control", &iLowS, 255);
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);
  cvCreateTrackbar("LowV", "Control", &iLowV, 255);
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);
  */

  //loop 20Hz
  while (ros::ok()) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT_RECTIFIED);
    auto &&right_data = api->GetStreamData(Stream::RIGHT_RECTIFIED);

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      double t_c = cv::getTickCount() / cv::getTickFrequency();
      fps = 1.0/(t_c - t);
      //printf("\b\b\b\b\b\b\b\b\b%.2f", fps);
      t = t_c;

      cv::Mat imgLeft(left_data.frame);
      cv::Mat imgRight(right_data.frame);
      cv::imshow("Origin", imgLeft);//BGR Originally

      vector<cv::Mat> bgrSplitL;
      split(imgLeft, bgrSplitL);
      vector<cv::Mat> bgrSplitR;
      split(imgRight, bgrSplitR);
      vector<cv::Mat> judgeChannel;
      judgeChannel.push_back(bgrSplitL[2] - 0.5*(bgrSplitL[0] + bgrSplitL[1]));//judge Red Right Lamp
      judgeChannel.push_back(bgrSplitR[2] - 0.5*(bgrSplitR[0] + bgrSplitR[1]));//judge Red Left Lamp
      threshold(judgeChannel[0], judgeChannel[0], detectRedThd, 255, 0);//return >detectRedThd ? 255 : 0 
      threshold(judgeChannel[1], judgeChannel[1], detectRedThd, 255, 0);
      //cout << "drT " << detectRedThd << endl;
      //inRange(img, Scalar(iLowB, iLowG, iLowR), Scalar(iHighB, iHighG, iHighR), judgeChannel[1]);//judge yellow

      cv::Mat element = getStructuringElement(MORPH_RECT, Size(10,10));
      for(int i = 0; i < 2; i++) 
	dilate(judgeChannel[i],judgeChannel[i],element);

      // added by zx 11.25 yellow not easy to judge
      vector<vector<Point>> lightContoursL, lightContoursR;
      // find and draw red contours
      findContours(judgeChannel[0].clone(), lightContoursL, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
      findContours(judgeChannel[1].clone(), lightContoursR, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      //if(!lightContoursL.size() || !lightContoursR.size()) continue;

      cv::Mat drawingL = Mat::zeros(imgLeft.size(), CV_8UC1);//TODO drawing
      cv::Mat drawingR = Mat::zeros(imgRight.size(), CV_8UC1);
      vector<Vec4i>hierarchy;
      Point2f verticesL[4];
      Point2f verticesR[4];
      int maxInd[2] = {-1,-1};
      float maxRectArea[2] = {0,0};


      for(size_t i = 0; i < lightContoursL.size(); i++) 
      {
	if(lightContoursL[i].size() <= 5) continue;//Boundary Conditions
        RotatedRect lightRec = fitEllipse(lightContoursL[i]);
        lightRec.points(verticesL);//get 4 corner
        float rectArea = calcDistance(verticesL[0],verticesL[1])*calcDistance(verticesL[1],verticesL[2]);
 	if(rectArea > maxRectArea[0])//more limits
	{
	  maxRectArea[0] = rectArea;
	  maxInd[0] = i;
	}
      }

      for(size_t i = 0; i < lightContoursR.size(); i++) 
      {
	if(lightContoursR[i].size() <= 5) continue;//Boundary Conditions
        RotatedRect lightRec = fitEllipse(lightContoursR[i]);
        lightRec.points(verticesR);//get 4 corner
        float rectArea = calcDistance(verticesR[0],verticesR[1])*calcDistance(verticesR[1],verticesR[2]);
 	if(rectArea > maxRectArea[1])//more limits
	{
	  maxRectArea[1] = rectArea;
	  maxInd[1] = i;
	}
      }

      int leftFlag = 0, findFlag = 0;
      Point2f deltaP;
      if(maxInd[0] != -1 && maxInd[1] != -1)
      {
        RotatedRect lightRecL = fitEllipse(lightContoursL[maxInd[0]]);
        RotatedRect lightRecR = fitEllipse(lightContoursR[maxInd[1]]);
        lightRecL.points(verticesL);//get 4 corner
        lightRecR.points(verticesR);//get 4 corner
        for(int i = 0; i < 4; i++)
	  line(drawingL, verticesR[i], verticesR[(i+1)%4], 255, 2);
        //printf("\tCenter x: %.2f, y: %.2f\t, deltaX:%.2f, deltaX':%.2f\n", 
	//(vertices[0].x+ vertices[2].x)/2.0f, (vertices[0].y+ vertices[2].y)/2.0f, 
        //vertices[1].x-vertices[0].x,vertices[1].y-vertices[0].y);
        deltaP.x = 640 - (verticesL[0].x+ verticesL[2].x)/2.0f;
        deltaP.y = (verticesR[0].x+ verticesR[2].x)/2.0f;

        if(deltaP.x > deltaP.y) leftFlag = 1;
	else if(deltaP.x < deltaP.y) leftFlag = -1;
	else leftFlag = 0;
	
        findFlag = 1;
	//printf("leftFlag: %d\n",leftFlag);
      }


	//drawContours(drawing, lightRec, (int)i, 255, 2, LINE_8, hierarchy, 0);
        //imshow("judgeImage", judgeChannel[2]);
        imshow("Contours", drawingL);

      // ros part
      std_msgs::String msg;
      std::stringstream ss;
      ss << findFlag << "," << leftFlag;
      msg.data = ss.str();
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);


  return 0;
}





