#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <GenicamSystem.h>
#include <SurfaceControlDevice.h>

using namespace ME;
using namespace std;

ros::Time time_st;

//image_transport::Publisher pub_frame_left;
//image_transport::Publisher pub_frame_right;
image_transport::CameraPublisher image_pub_left;
image_transport::CameraPublisher image_pub_right;
sensor_msgs::CameraInfo cam_info_left;
sensor_msgs::CameraInfo cam_info_right;
camera_info_manager::CameraInfoManager *cinfo_left;
camera_info_manager::CameraInfoManager *cinfo_right;

megc_status_t status;
GenicamDeviceVec devices;
SurfaceControlDevicePtr connected_device;
//const int image_count = 2;
//meg_image_t camera_images[image_count];
//SurfaceControlDevice::CameraData image_identifiers[image_count];
meg_image_t camera_image_left;
meg_image_t camera_image_right;

SurfaceControlDevice::CameraData image_identifier_left;
SurfaceControlDevice::CameraData image_identifier_right;
int frame_count = 0;
mutex mutex_waiting;
bool data_received = false;
bool acquisition_active = false;
condition_variable event_condition;

void findDevices()
{
  cout << "Finding devices...\n";

  status = GenicamSystem::findDevices();
  if (status != megc_status_t::MEGC_OK)
  {
    cerr << "Error finding devices!\n";
    return;
  }

  devices = GenicamSystem::devices();
  cout << devices->size() << " device(s) found.\n";
}

void connectSurfaceControlDevice()
{
  cout << "Connecting to surfaceCONTROL device...\n";
  int count = 0;

  status = megc_status_t::MEGC_ERR_DEVICE_NOT_CONNECTED;
  for (GenicamDevicePtr device : *devices)
  {
    if (device->type() != GenicamDevice::T_SurfaceControl)
      continue;

    count++;
    status = device->connect();
    if (status == megc_status_t::MEGC_OK)
    {
      connected_device = device->toSurfaceControlDevice();
      cout << "Device connected: Name: \"" << device->modelName() << "\" SN: \"" << device->serialNumber() << "\"\n";
      //cinfo_->setCameraName(device->modelName());
      return;
    }
  }

  if (count == 0)
    cerr << "No surfaceCONTROL device found!\n";
  else
    cerr << "Connecting to surfaceCONTROL device failed!\n";
}

void setParameters()
{
  //Set some basic parameters
  cout << "Set exposure time to 2500 microseconds\n";
  status = connected_device->setExposureTime(2500.0f);
  if (status != megc_status_t::MEGC_OK)
    return;
}

static
void imageCallback(const vector<meg_image_t*>& images, const vector<SurfaceControlDevice::CameraData>& identifiers, void* /*user_data*/)
{
  camera_image_left = *(images[0]);
  camera_image_right = *(images[1]); // something is related to vectors and a variable. these are called in this call back as vector ! so who know !!! 
  image_identifier_left = identifiers[0];
  image_identifier_right = identifiers[1];
  //cout << *(images[0]) << endl;
  //cout << &identifiers<<endl<< identifiers[0]<<endl;

  //We signal the measurement loop that data has been received
  data_received = true;
  event_condition.notify_all();
}
void setupDataTransfer()
{
  cout << "Setup data transfer of camera images\n";
  status = connected_device->setCameraDataEnabled(
    SurfaceControlDevice::CD_ImageCamera0 | SurfaceControlDevice::CD_ImageCamera1);
    //status = connected_device->setCameraDataEnabled(SurfaceControlDevice::CD_ImageCamera0);
  if (status != megc_status_t::MEGC_OK)
    return;

  cout << "Set callback for camera images\n";
  connected_device->setCameraImageCallback(imageCallback);

  cout << "Start image acquisition (continuous mode)\n";
  status = connected_device->acquisitionStart(GenicamDevice::AcquisitionMode::AM_Continuous, false);
  if (status != megc_status_t::MEGC_OK)
    return;

  acquisition_active = true;
}

void processData()
{
    /*cv_bridge::CvImagePtr cv_ptr_left (new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_right (new cv_bridge::CvImage);
    cv_ptr_left->encoding = "mono8";
    cv_ptr_left->header.stamp = time;
    cv_ptr_left->header.frame_id = "camera_optical_frame_left";
    cv_ptr_right->encoding = "mono8";
    cv_ptr_right->header.stamp = times;
    cv_ptr_right->header.frame_id = "camera_optical_frame_right";*/
    ros::Time time = ros::Time::now();ros::Time times = ros::Time::now();
    cam_info_left  = cinfo_left->getCameraInfo();
    cam_info_right =  cinfo_right->getCameraInfo();
    cam_info_left.header.frame_id = "camera_optical_frame_left";
    cam_info_left.header.stamp = time;
    cam_info_right.header.frame_id = "camera_optical_frame_right";
    cam_info_right.header.stamp = times;
    int x_left = camera_image_left.width / 2;
    int y_left = camera_image_left.height / 2;
    int w_left = camera_image_left.width;
    int x_right = camera_image_right.width / 2;
    int y_right = camera_image_right.height / 2;
    int w_right = camera_image_right.width;
    int p_size_left = camera_image_left.width*camera_image_left.height;
    int p_size_right = camera_image_right.width*camera_image_right.height;
    //cout<< p_size<< endl;
    //cout<< camera_images.type<<endl;
    /*if ((camera_images.type != MEG_PIX_MONO8) && (camera_images.type != MEG_PIX_MONO12))
      cerr << "Unexpected image type!\n";
    else if (camera_images.type == MEG_PIX_MONO12)
    {
      meg_image_mono16_t* image_mono_16 = (meg_image_mono16_t*)&(camera_images);
      cout << "Pixel value image_mono_16(" << x << ", " << y << "): " << (int)(image_mono_16->data[y * w + x]) << "    "<< endl;
    }
    else
    {*/
      meg_image_mono8_t* image_mono_8_left = (meg_image_mono8_t*)&(camera_image_left);
      meg_image_mono8_t* image_mono_8_right = (meg_image_mono8_t*)&(camera_image_right);
      //meg_image_rgb8_t* image_rbg_8_left = (meg_image_rgb8_t*)&(camera_image_left);
     // meg_image_rgb8_t* image_rbg_8_right = (meg_image_rgb8_t*)&(camera_image_right);

      //cout << "Pixel value image_mono_8(" << x << ", " << y << "): " << (int)(image_mono_8->data[y * w + x]) << "    "<< endl;
      cv::Mat img_out_left = cv::Mat(image_mono_8_left->height, image_mono_8_left->width, CV_8UC1, image_mono_8_left->data);
      cv::Mat img_out_right = cv::Mat(image_mono_8_right->height, image_mono_8_right->width, CV_8UC1, image_mono_8_right->data);
      //cv::Mat img_out_left = cv::Mat(image_rbg_8_left->height, image_rbg_8_left->width, CV_8UC3, image_rbg_8_left->data);
      //cv::Mat img_out_right = cv::Mat(image_rbg_8_right->height, image_rbg_8_right->width, CV_8UC3, image_rbg_8_right->data);

      //std::cout<< "Im here "<< std::endl;
      /*cv_ptr_left->image = img_out_left;
      cv_ptr_right->image = img_out_right;*/
      sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_out_left).toImageMsg();
      sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_out_right).toImageMsg();
      //sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_out_left).toImageMsg();
      //sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_out_right).toImageMsg();
      //std::cout<< "Im here too "<< std::endl;
      msg_left->header.stamp = time;
      msg_left->header.frame_id = "camera_optical_frame_left";
      msg_right->header.stamp = times;
      msg_right->header.frame_id = "camera_optical_frame_right";
      /*cam_info_left.height = msg_left->height;
      cam_info_left.width = msg_left->width;
      cam_info_right.height = msg_right->height;
      cam_info_right.width =  msg_right->width;*/

      //pub_frame_left.publish(cv_ptr_left->toImageMsg());
      //pub_frame_right.publish(cv_ptr_right->toImageMsg());
      image_pub_left.publish(*msg_left,cam_info_left);
      image_pub_right.publish(*msg_right,cam_info_right);
      

      //cout<< img_out << endl;
      //cv::imshow("Display window", img_out);

    //}

  }

void measurementLoop()
{
  cout << "Measuring...\n";
  do
  {
    data_received = false;

    //Wait until data is received, we wait a maximum of 1 second
    unique_lock<mutex> lk(mutex_waiting);
    if ((event_condition.wait_for(lk, std::chrono::seconds(1), []() {return data_received; })))
      processData();
    else
      cerr << "Image timeout!\n";

    data_received = false;
  } while (acquisition_active);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surfacecontrol_ros_rgb2D");
  ros::NodeHandle nh;
  ros::NodeHandle nh_left(nh, "left_camera");
  ros::NodeHandle nh_right(nh, "right_camera");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ROS_INFO("Namaskaar!  RGB coming up");
  image_transport::ImageTransport it_(nh);
  const string camurl_left = "file:///home/terabotics/.ros/camera_info/camera_left.yaml";
  const string camurl_right = "file:///home/terabotics/.ros/camera_info/camera_right.yaml";
  //const string camurl_left = "file:///home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_nov_9_2nd/left.yaml";
  //const string camurl_right = "file:///home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_nov_9_2nd/right.yaml";
  cinfo_left = new camera_info_manager::CameraInfoManager(nh_left, "camera_left", camurl_left);
  cinfo_right = new camera_info_manager::CameraInfoManager(nh_right,"camera_right", camurl_right);
  //cinfo_left->setCameraName("camera_left");
  //cinfo_right->setCameraName("camera_right");
  //cam_info_left  = cinfo_left->getCameraInfo();
  //cam_info_right =  cinfo_right->getCameraInfo();

  image_pub_left = it_.advertiseCamera("/camera/left/image_raw", 1);
  image_pub_right = it_.advertiseCamera("/camera/right/image_raw", 1);

  //pub_frame_left = it_.advertise("camera/left/image_raw", 1);
  //pub_frame_right = it_.advertise("camera/right/image_raw", 1);
  

  status = GenicamSystem::initialize();

  if (status != megc_status_t::MEGC_OK)
    cerr << "Error initializing GenicamSystem!\n";
  else
    findDevices();

  if (status == megc_status_t::MEGC_OK)
    connectSurfaceControlDevice();

  if (status == megc_status_t::MEGC_OK)
    setParameters();

  if (status == megc_status_t::MEGC_OK)
    setupDataTransfer();

  if (status == megc_status_t::MEGC_OK)
  {
    cout << "\n\n Press enter to abort acquisition\n";
    std::thread t(measurementLoop);

    cin.ignore();
    acquisition_active = false;

    t.join();
  }

  if (status == megc_status_t::MEGC_OK)
    connected_device->acquisitionStop();

  if (connected_device != nullptr)
    connected_device->disconnect();
  

  ros::waitForShutdown();

  GenicamSystem::destroy();
}