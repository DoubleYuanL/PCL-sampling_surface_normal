# include "../include/headers.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointNormal PointNormalType;
typedef pcl::Normal NormalType;

class keypoints {
public:
	pcl::PointCloud<PointType>				cloud1, cloud2;
	pcl::PointCloud<NormalType>				cloud1_normals, cloud2_normals;
	pcl::PointCloud<PointNormalType>		cloud1_point_normals, cloud2_point_normals;
	pcl::PointCloud<PointType>				cloud1_keypoints, cloud2_keypoints;

	template <typename T>
	void calculate_normals(float radius, T cloud1, T cloud2)
	{
		// Estimate the normals.
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setRadiusSearch(radius);
		normalEstimation.setNumberOfThreads(12);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);

		normalEstimation.setInputCloud(cloud1.makeShared());
		normalEstimation.compute(cloud1_normals);
	}

	void compute_sampling_surface_normal_keypoints()
	{
		pcl::PointCloud<PointNormalType> Temp_cloud1_point_normals;
		pcl::concatenateFields(cloud1, cloud1_normals, cloud1_point_normals);//SamplingSurfaceNormal处理的点云需要法线信息 连接法线
		
		pcl::SamplingSurfaceNormal<PointNormalType> ssn;

		ssn.setInputCloud(cloud1_point_normals.makeShared());
		ssn.setSeed(1);
		ssn.setSample(2);
		ssn.setRatio(0.1);//每个网格中采样的比率 比率越大点越多
		ssn.filter(Temp_cloud1_point_normals);

		cloud1_keypoints.resize(Temp_cloud1_point_normals.size());
		for (size_t i = 0; i < Temp_cloud1_point_normals.size(); i++)//显示关键点把XYZ另存
		{
			cloud1_keypoints.points[i].x = Temp_cloud1_point_normals.points[i].x;
			cloud1_keypoints.points[i].y = Temp_cloud1_point_normals.points[i].y;
			cloud1_keypoints.points[i].z = Temp_cloud1_point_normals.points[i].z;
		}
	}
};

int main()
{
	LARGE_INTEGER cpuFreq;
	LARGE_INTEGER startTime;
	LARGE_INTEGER endTime;
	QueryPerformanceFrequency(&cpuFreq);

	keypoints cb;

	pcl::io::loadPCDFile<PointType>("../dataset5pcd/MeshRegistration/Mario/mario000.pcd", cb.cloud1);   //867
	
	cb.calculate_normals(0.02, cb.cloud1, cb.cloud2);
	QueryPerformanceCounter(&startTime);
		cb.compute_sampling_surface_normal_keypoints();
	QueryPerformanceCounter(&endTime);

	std::cout << "Found " << cb.cloud1.points.size() << " points.\n";
	std::cout << "Found " << cb.cloud1_keypoints.points.size() << " key points.\n";
	cout << "Time taken for KeyPoints       calculation: " << (((endTime.QuadPart - startTime.QuadPart) * 1000.0f) / cpuFreq.QuadPart) << "(micro-second)" << endl;

	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	//pcl::visualization::PointCloudColorHandlerCustom<PointType>     cloud_color_handler(cb.cloud1_keypoints.makeShared(), 0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints_color_handler(cb.cloud1_keypoints.makeShared(), 0, 255, 0);

	pcl::visualization::Camera camera;
	viewer.getCameraParameters(camera);
	viewer.setCameraPosition(0, 0, 2, 0, -10, -1);

	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//viewer.addPointCloud(cb.cloud1.makeShared(), cloud_color_handler, "cloud");
	viewer.addPointCloud(cb.cloud1.makeShared(), "cloud");
	viewer.addPointCloud(cb.cloud1_keypoints.makeShared(), keypoints_color_handler, "keypoints");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");

	//  viewer.addCoordinateSystem();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	system("pause");
	return 0;
}
