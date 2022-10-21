#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <GenicamSystem.h>
#include <SurfaceControlDevice.h>


using namespace ME;
using namespace std;
using PointCloudTypePtr = PointCloudPtr<PointXYZ<float>>;
using PointCloudType = PointCloud<PointXYZ<float>>;


ros::Time time_st;
ros::Publisher pub;

megc_status_t status;
GenicamDeviceVec devices;
SurfaceControlDevicePtr connected_device;
mutex mutex_waiting;
bool data_received = false;
condition_variable event_condition;
PointCloudTypePtr point_cloud;


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

void outputDeviceInfo()
{
  for (int i = 0; i < (int)devices->size(); i++)
  {
    GenicamDevicePtr device = devices->at(i);

    cout << "Device info of device " << i + 1 << ":\n";
    cout << "Model name: " << device->modelName() << "\n";
    cout << "Vendor name: " << device->vendorName() << "\n";
    cout << "Serial number: " << device->serialNumber() << "\n";
    cout << "IP address: " << device->ipAddress() << "\n";
    cout << "MAC address: " << device->macAddress() << "\n";
    cout << "Subnet Mask: " << device->subnetMask() << "\n";
    cout << "Default Gateway: " << device->defaultGateway() << "\n";
    cout << "Has persistent IP address: " << (int)device->isPersistentEnabled() << "\n\n";
  }
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
void eventExposureEndCallback(meg_uint64_t /*timestamp*/, void* /*user_data*/)
{
  //Sensor exposure finished, sensor may be moved to next measurement position
  cout << "Exposure finished, data processing...\n";
}

static
void eventFrameEndCallback(meg_uint64_t /*timestamp*/, void* /*user_data*/)
{
  //Data processing finished, the next measurement may be triggered
  cout << "Data processing finished, transferring data...\n";
}

static
void eventErrorCallback(int error_code, std::string error_message, meg_uint64_t /*timestamp*/, void* /*user_data*/)
{
  //A sensor error has occured
  cerr << "Error " << error_code << ": " << error_message << "\n";
}

static
void pointCloudCallback(PointCloudTypePtr data, void* /*user_data*/)
{
  point_cloud = data;
  

  //We signal the measurement loop that data has been received
  data_received = true;
  event_condition.notify_all();
}

void processData()
{
    
    pcl::PointCloud<pcl::PointXYZ> point_cloud_data;
    pcl::PointXYZ points_of_pcl;
    point_cloud_data.header.frame_id = "camera_frame_raw";
    time_st = ros::Time::now ();
    point_cloud_data.header.stamp = time_st.toNSec()/1e3;
    const PointCloudType::Points& points_data = point_cloud->points();
    //cout<< points_data[0].x<<endl;
    if (points_data.count() == 0)
    {
      cerr << "Point cloud is empty!\n";
      return;
    }
    int valid_points = 0;
    for (const PointXYZ<float>& point : points_data)
    {
        if (PointCloudType::isValid(point))
           valid_points++;
        else
          continue;
        if (PointCloudType::isValid(point))
        points_of_pcl.x = points_data[valid_points].x*0.001f;
        points_of_pcl.y = points_data[valid_points].y*0.001f;
        points_of_pcl.z = points_data[valid_points].z*0.001f;
        point_cloud_data.points.push_back(points_of_pcl);
        

    }
    point_cloud_data.width = point_cloud_data.size();
    point_cloud_data.height = 1;
    cout<< point_cloud_data.size() << endl;
    //pub.publish(point_cloud_data.makeShared());
    pub.publish(point_cloud_data);
    //
    // point_cloud_data->points[valid_points].x = points[valid_points].x; 
    cout << point_cloud_data << endl;
}
void setupDataTransfer()
{
  cout << "Setup data transfer of 3D data\n";
  status = connected_device->setMeasurementDataEnabled();
  if (status != megc_status_t::MEGC_OK)
    return;

  cout << "Enable events\n";
  connected_device->setEventsEnabled(SurfaceControlDevice::EV_ExposureEnd | SurfaceControlDevice::EV_FrameEnd | SurfaceControlDevice::EV_Error);

  cout << "Set callback for exposure end\n";
  connected_device->setEventExposureEndCallback(eventExposureEndCallback);

  cout << "Set callback for frame end\n";
  connected_device->setEventFrameEndCallback(eventFrameEndCallback);

  cout << "Set callback for errors\n";
  connected_device->setEventErrorCallback(eventErrorCallback);

  cout << "Set callback for 3D data\n";
  connected_device->set3dDataCallback(pointCloudCallback);

  cout << "Start image acquisition (triggered mode)\n";
  status = connected_device->acquisitionStart(GenicamDevice::AcquisitionMode::AM_Continuous, true);
  if (status != megc_status_t::MEGC_OK)
    return;
}


void measurementLoop()
{
  while (ros::ok()){

    cout << "Trigger measurement\n";
    data_received = false;
    status = connected_device->triggerSoftware();
    if (status != megc_status_t::MEGC_OK)
        return;

    cout << "Measuring...\n";

    //Wait until data is received, we wait a maximum of 20 seconds
    unique_lock<mutex> lk(mutex_waiting);
    if ((event_condition.wait_for(lk, std::chrono::seconds(20), [](){return data_received; })))
        processData();
    else
        cerr << "Measurement timeout! No measurement data received!\n";
  } 
 
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "surfacecontrol_ros_points3D");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_INFO("Namaskaar!");
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("camera/depth/points",10);
    status = GenicamSystem::initialize();
    if (status != megc_status_t::MEGC_OK)
    cerr << "Error initializing GenicamSystem!\n";
    else
      findDevices();
    //if (status == megc_status_t::MEGC_OK)
     // outputDeviceInfo();
    if (status == megc_status_t::MEGC_OK)
      connectSurfaceControlDevice();
    if (status == megc_status_t::MEGC_OK)
      setParameters();
    if (status == megc_status_t::MEGC_OK)
      setupDataTransfer();
    if (status == megc_status_t::MEGC_OK)
      measurementLoop();
    if (connected_device != nullptr)
    connected_device->disconnect();

    
    ros::waitForShutdown();
    
    GenicamSystem::destroy();
    
}


