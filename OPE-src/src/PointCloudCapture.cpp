#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/image_grabber.h>
#include "OPESettings.h"
#include "OPEUtils.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/range_image/range_image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include <string>

#include "PointCloudCapture.h"

using namespace ope;



PointCloudCapture::PointCloudCapture() : ptCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), 
	filteredPtCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), dataCaptured(false) { 

}


PointCloudCapture::~PointCloudCapture() {

}


void PointCloudCapture::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {

	if (cloud->size() > 0)
	{
	    ptCloudPtr = cloud;
	    dataCaptured = true;
	    std::cout << "New Cloud Captured!\n";
	}
	else
	{
		std::cout << "No data in captured cloud!\n";
	}

}


void PointCloudCapture::run(pcl::PointCloud<pcl::PointXYZRGB>& ptCloud, const OPESettings& settings) {

	// Live data from camera
	if (!settings.fromFile)
	{

		cout << "Reading point cloud from camera B00364613926048B" << endl;
		// open up the grabber with the downward facing camera
		pcl::Grabber* grabber = new pcl::OpenNIGrabber("B00364613926048B");

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
			boost::bind (&PointCloudCapture::cloudCallback, this, _1);

		if (settings.verbose) {
			Utils::printCurrentDateTime();
			std::cout << ">> Capturing point cloud using the Kinect" << std::endl;
		}

		grabber->registerCallback(f);
		grabber->start();

		// Pause for 30 milliseconds to ensure that a point cloud is captured
	//	boost::this_thread::sleep(boost::posix_time::seconds(5));

	        while(!dataCaptured) {
	            boost::this_thread::sleep(boost::posix_time::microseconds(100));
	        }
	        
		grabber->stop();
	}
	// data from point cloud file
	else
	{
		cout << "Reading point cloud from file " << settings.cloudFile << endl;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloudTemp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (settings.cloudFile, *ptCloudTemp) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file \n");
			return;
		}
		ptCloudPtr = ptCloudTemp;
	}


	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (ptCloudPtr);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (settings.minTgtDepth, settings.maxTgtDepth);
	pass.filter (*filteredPtCloudPtr);

	// copy pointcloud to output parameter
	Utils::convertPointCloud(*filteredPtCloudPtr, ptCloud);

	if (settings.verbose) {
		std::cout << ">>\t ...done" << std::endl;
	}

	// Only for debugging purposes
	if (settings.doDebug) {
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Debugging Viewer"));
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(filteredPtCloudPtr);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGBA> (filteredPtCloudPtr, rgb, "FilteredCloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Filtered Cloud");
		viewer->initCameraParameters();
		viewer->resetCameraViewpoint("FilteredCloud");

		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds(10));
		}			
	}

}
