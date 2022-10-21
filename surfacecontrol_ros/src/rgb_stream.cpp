#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <GenicamSystem.h>
#include <SurfaceControlDevice.h>

using namespace ME;
using namespace std;

ros::Time time_st;

image_transport::Publisher pub_frame;


megc_status_t status;
GenicamDeviceVec devices;
SurfaceControlDevicePtr connected_device;
//const int image_count = 2;
//meg_image_t camera_images[image_count];
//SurfaceControlDevice::CameraData image_identifiers[image_count];
meg_image_t camera_images;
SurfaceControlDevice::CameraData image_identifiers;
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
 // also need to change the global variables with this.. !
 /* if (images.size() != image_count)
  {
    cerr << "Unexpected image count!\n";
    return;
  }
  for (int i = 0; i < image_count; i++)
  {
    camera_images[i] = *(images[i]);
    image_identifiers[i] = identifiers[i];
  }*/

  camera_images = *(images[0]); // something is related to vectors and a variable. these are called in this call back as vector ! so who know !!! 
  image_identifiers = identifiers[0];
  //cout << *(images[0]) << endl;
  //cout << &identifiers<<endl<< identifiers[0]<<endl;

  //We signal the measurement loop that data has been received
  data_received = true;
  event_condition.notify_all();
}
void setupDataTransfer()
{
  cout << "Setup data transfer of camera images\n";
  /*status = connected_device->setCameraDataEnabled(
    SurfaceControlDevice::CD_ImageCamera0 | SurfaceControlDevice::CD_ImageCamera1);*/
    status = connected_device->setCameraDataEnabled(SurfaceControlDevice::CD_ImageCamera0);
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
/*void processData()
{
  frame_count++;
  cout << "Processing frame idx " << frame_count << ": ";
  for (int i = 0; i < image_count; i++)
  {
    switch (image_identifiers[i])
    {
    case SurfaceControlDevice::CameraData::CD_ImageCamera0: cout << "Camera 1: "; break;
    case SurfaceControlDevice::CameraData::CD_ImageCamera1: cout << "Camera 2: "; break;
    default: cerr << "Unexpected measurement type!\n";
    }

    int x = camera_images[i].width / 2;
    int y = camera_images[i].height / 2;
    int w = camera_images[i].width;
    if ((camera_images[i].type != MEG_PIX_MONO8) && (camera_images[i].type != MEG_PIX_MONO12))
      cerr << "Unexpected image type!\n";
    else if (camera_images[i].type == MEG_PIX_MONO12)
    {
      meg_image_mono16_t* image_mono_16 = (meg_image_mono16_t*)&(camera_images[i]);
      cout << "Pixel value (" << x << ", " << y << "): " << (int)(image_mono_16->data[y * w + x]) << "    ";
    }
    else
    {
      meg_image_mono8_t* image_mono_8 = (meg_image_mono8_t*)&(camera_images[i]);
      cout << "Pixel value (" << x << ", " << y << "): " << (int)(image_mono_8->data[y * w + x]) << "    ";
    }
  }

  cout << "\r";
}*/
void processData()
{
  //frame_count++;
  //cout << "Processing frame idx " << frame_count << ": " << endl;

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    cv_ptr->encoding = "mono8";
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "camera_optical_frame_left";

    int x = camera_images.width / 2;
    int y = camera_images.height / 2;
    int w = camera_images.width;
    int p_size = camera_images.width*camera_images.height;
    //cout<< p_size<< endl;
    //cout<< camera_images.type<<endl;
    if ((camera_images.type != MEG_PIX_MONO8) && (camera_images.type != MEG_PIX_MONO12))
      cerr << "Unexpected image type!\n";
    else if (camera_images.type == MEG_PIX_MONO12)
    {
      meg_image_mono16_t* image_mono_16 = (meg_image_mono16_t*)&(camera_images);
      cout << "Pixel value image_mono_16(" << x << ", " << y << "): " << (int)(image_mono_16->data[y * w + x]) << "    "<< endl;
    }
    else
    {
      meg_image_mono8_t* image_mono_8 = (meg_image_mono8_t*)&(camera_images);
      //cout<< image_mono_8->type<<endl;
      //cout << "Pixel value image_mono_8(" << x << ", " << y << "): " << (int)(image_mono_8->data[y * w + x]) << "    "<< endl;

      cv::Mat img_out = cv::Mat(image_mono_8->height, image_mono_8->width, CV_8UC1, image_mono_8->data);

      
      cv_ptr->image = img_out;
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "CV_8UC1", img_out).toImageMsg();
      pub_frame.publish(cv_ptr->toImageMsg());
      //cout<< img_out << endl;
      //cv::imshow("Display window", img_out);

    }

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
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ROS_INFO("Namaskaar!  RGB coming up");
  image_transport::ImageTransport it_(nh);
  pub_frame = it_.advertise("camera/image", 1);
  
  
  //pub = nh.advertise<> > ("camera/color/image",10);
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