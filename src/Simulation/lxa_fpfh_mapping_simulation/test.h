#pragma once
//以fhfp和曲率一起对比点云
float radii_search2fpfh = 0, radii_search2curva = 0;
int num_search = 0;

bool if_distance_measure = false;
float correValueOrdistance = 0;
float curvadiff_tol = 0, shapeindex_tol = 0;

int ransac_itera_number = 0;
double threshold_temp = 0;
bool if_ransac = false;

std::cout << "please input a search number for searcher: ";
std::cin >> num_search;
std::cout << std::endl;

std::cout << "please input a search radius for fpfh_searcher: ";
std::cin >> radii_search2fpfh;
std::cout << std::endl;

std::cout << "please input a search radius for curvature_searcher: ";
std::cin >> radii_search2curva;
std::cout << std::endl;

std::cout << "please input a bool value to whether or not distance_measure: ";
std::cin >> if_distance_measure;
std::cout << std::endl;

if (true == if_distance_measure)
{
	std::cout << "please input a value  for distance_sup: ";
	std::cin >> correValueOrdistance;
	std::cout << std::endl;
}
else
{
	std::cout << "please input a value  for correlation_inf: ";
	std::cin >> correValueOrdistance;
	std::cout << std::endl;
}

std::cout << "please input a curature difference_tol for tow pointClouds: ";
std::cin >> curvadiff_tol;
std::cout << std::endl;

std::cout << "please input a shapeindex difference_tol for tow pointClouds: ";
std::cin >> shapeindex_tol;
std::cout << std::endl;

std::cout << "please input a bool value  for  whether or not ransac: ";
std::cin >> if_ransac;
std::cout << std::endl;

if (true == if_ransac)
{
	std::cout << "please input a value  for ransac iteration number: ";
	std::cin >> ransac_itera_number;
	std::cout << std::endl;

	std::cout << "please input a ransac threshold for two pointClous: ";
	std::cin >> threshold_temp;
	std::cout << std::endl;
}


string file_outpath1 = "G:/mesh_test/cloud1.ply";
string file_outpath2 = "G:/mesh_test/cloud2.ply";
string file_outpath2_ori = "G:/mesh_test/cloud2_ori.ply";

pcl::PointCloud<pcl::PointNormal>::Ptr frame_normal_cloud1(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr frame_normal_cloud2(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr frame_normal_cloud2_ori(new pcl::PointCloud<pcl::PointNormal>);
load_pointcloud(frame_normal_cloud1, file_outpath1);
load_pointcloud(frame_normal_cloud2, file_outpath2);
load_pointcloud(frame_normal_cloud2_ori, file_outpath2_ori);

//根据点云1和旋转后的点云2,找最匹配的点对
CloudFeature::cloudFeature cloudfeature;
pcl::CorrespondencesPtr corres_set(new pcl::Correspondences());
cloudfeature.setFpfh_search_radius(radii_search2fpfh);
cloudfeature.setCurvature_search_radius(radii_search2curva);
cloudfeature.setFpfh_search_num(num_search);
cloudfeature.setCurvature_search_num(num_search);
cloudfeature.setIfdistance_measure(if_distance_measure);
cloudfeature.setDistance_tol(correValueOrdistance);
cloudfeature.setCorrelation_tol(correValueOrdistance);
cloudfeature.setCurvature_tol(curvadiff_tol);
cloudfeature.setShapeIndex_tol(shapeindex_tol);

//cloudfeature.compareClouds_fpfhAndcurature(frame_normal_cloud1, frame_normal_cloud2, *corres_set);
//cloudfeature.compareClouds_curaturecolor(frame_normal_cloud1, frame_normal_cloud2, *corres_set);
//cloudfeature.compareClouds_curvadness(frame_normal_cloud1, frame_normal_cloud2, *corres_set);

cloudfeature.compareClouds_fpfhAndcuratureAndshapeIndex(frame_normal_cloud1, frame_normal_cloud2, *corres_set);

//匹配点对集的处理
pcl::CorrespondencesPtr rejCorres(new pcl::Correspondences());
if (false == if_ransac)
{
	std::cout << " -----------------Reject correspondences by point's distance and one-2-one--------------" << std::endl;
	// firstly reject by point distance
	pcl::CorrespondencesPtr rej_dis_corres_set(new pcl::Correspondences());
	pcl::registration::CorrespondenceRejectorDistance rej_points_dis;
	rej_points_dis.setInputSource<pcl::PointNormal>(frame_normal_cloud1);
	rej_points_dis.setInputTarget<pcl::PointNormal>(frame_normal_cloud2);
	rej_points_dis.setMaximumDistance(1);
	rej_points_dis.setInputCorrespondences(corres_set);
	rej_points_dis.getCorrespondences(*rej_dis_corres_set);

	//secondly one to one reject
	pcl::registration::CorrespondenceRejectorOneToOne rej_one;
	rej_one.getRemainingCorrespondences(*rej_dis_corres_set, *rejCorres);


	//为了方便显示点云,将第二个点云(frame_normal_cloud2)沿z轴正方向平移5个单位
	pcl::PointCloud<pcl::PointNormal>::Ptr frame_normal_cloud2_trans(new pcl::PointCloud<pcl::PointNormal>);
	Eigen::Matrix4f T_trans = Eigen::Matrix4f::Identity();
	T_trans(0, 3) = 2; T_trans(1, 3) = 2; T_trans(2, 3) = 5;
	pcl::transformPointCloudWithNormals(*frame_normal_cloud2, *frame_normal_cloud2_trans, T_trans);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("viewer"));  //定义窗口共享指针
	view->setBackgroundColor(0, 0, 0);                                                 //设窗口的背景色

	view->addPointCloud<pcl::PointNormal>(frame_normal_cloud1, "pointcloud1_v1");      //将点云添加到窗口
	view->addPointCloud<pcl::PointNormal>(frame_normal_cloud2_trans, "pointcloud2_v2");//将点云添加到窗口

	view->addCorrespondences<pcl::PointNormal>(frame_normal_cloud1, frame_normal_cloud2_trans, *rejCorres, "correspond");//将源点云、目标点云、对应点对集加入
	view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "correspond");
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
else
{
	//ransac过滤掉误配的点对
	std::cout << " -----------------RAN_SAC--------------" << std::endl;

	// firstly reject by point distance
	pcl::CorrespondencesPtr rej_dis_corres_set(new pcl::Correspondences());
	pcl::registration::CorrespondenceRejectorDistance rej_points_dis;
	rej_points_dis.setInputSource<pcl::PointNormal>(frame_normal_cloud1);
	rej_points_dis.setInputTarget<pcl::PointNormal>(frame_normal_cloud2);
	rej_points_dis.setMaximumDistance(1);
	rej_points_dis.setInputCorrespondences(corres_set);
	rej_points_dis.getCorrespondences(*rej_dis_corres_set);

	//secondly one to one rejiect
	pcl::CorrespondencesPtr rejCorres_one(new pcl::Correspondences());
	pcl::registration::CorrespondenceRejectorOneToOne rej_one;
	rej_one.getRemainingCorrespondences(*rej_dis_corres_set, *rejCorres_one);

	//thirdly distance rejiect by ransac
	pcl::registration::CorrespondenceRejectorSampleConsensus <pcl::PointNormal> rej_sac;
	Eigen::Matrix4f T;
	rej_sac.setInputSource(frame_normal_cloud1);
	rej_sac.setInputTarget(frame_normal_cloud2);
	rej_sac.setRefineModel(false);
	rej_sac.setMaximumIterations(ransac_itera_number);
	rej_sac.setInlierThreshold(threshold_temp);
	rej_sac.setInputCorrespondences(rejCorres_one);
	rej_sac.getCorrespondences(*rejCorres);

	//为了方便显示点云,将第二个点云(frame_normal_cloud2)沿z轴正方向平移5个单位
	pcl::PointCloud<pcl::PointNormal>::Ptr frame_normal_cloud2_trans(new pcl::PointCloud<pcl::PointNormal>);
	Eigen::Matrix4f T_trans = Eigen::Matrix4f::Identity();
	T_trans(0, 3) = 2; T_trans(1, 3) = 2; T_trans(2, 3) = 5;
	pcl::transformPointCloudWithNormals(*frame_normal_cloud2, *frame_normal_cloud2_trans, T_trans);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("viewer"));  //定义窗口共享指针
	view->setBackgroundColor(0.0, 0.0, 0.0); //设窗口的背景色

	view->addPointCloud<pcl::PointNormal>(frame_normal_cloud1, "pointcloud1_v1");      //将点云添加到窗口
	view->addPointCloud<pcl::PointNormal>(frame_normal_cloud2_trans, "pointcloud2_v2");//将点云添加到窗口

	view->addCorrespondences<pcl::PointNormal>(frame_normal_cloud1, frame_normal_cloud2_trans, *rejCorres, "correspond");//将源点云、目标点云、对应点对集加入
	view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "correspond");
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//对筛选后的精确度最高的点对,标记颜色(利用点云1和旋转前的点云2进行标记)
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbCloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbCloud2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
PointXYZRGBNormal point1, point2;

float diff_x = 0, diff_y = 0, diff_z = 0, distance = 0;
int count_20 = 0, count_2050 = 0, count_50100 = 0, count_100200 = 0, count = 0;
for (auto c : *rejCorres)
{
	point1.x = frame_normal_cloud1->points[c.index_query].x;
	point1.y = frame_normal_cloud1->points[c.index_query].y;
	point1.z = frame_normal_cloud1->points[c.index_query].z;

	point1.normal_x = frame_normal_cloud1->points[c.index_query].normal_x;
	point1.normal_y = frame_normal_cloud1->points[c.index_query].normal_y;
	point1.normal_z = frame_normal_cloud1->points[c.index_query].normal_z;

	point1.r = 0;
	point1.g = 255;
	point1.b = 0;

	point2.x = frame_normal_cloud2_ori->points[c.index_match].x;
	point2.y = frame_normal_cloud2_ori->points[c.index_match].y;
	point2.z = frame_normal_cloud2_ori->points[c.index_match].z;

	point2.normal_x = frame_normal_cloud2_ori->points[c.index_match].normal_x;
	point2.normal_y = frame_normal_cloud2_ori->points[c.index_match].normal_y;
	point2.normal_z = frame_normal_cloud2_ori->points[c.index_match].normal_z;

	point2.r = 255;
	point2.g = 0;
	point2.b = 0;

	rgbCloud1->push_back(point1);
	rgbCloud2->push_back(point2);

	diff_x = (frame_normal_cloud1->points[c.index_query].x) - (frame_normal_cloud2_ori->points[c.index_match].x);
	diff_y = (frame_normal_cloud1->points[c.index_query].y) - (frame_normal_cloud2_ori->points[c.index_match].y);
	diff_z = (frame_normal_cloud1->points[c.index_query].z) - (frame_normal_cloud2_ori->points[c.index_match].z);
	distance = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
	if (distance<0.02 || distance == 0.02)
	{
		std::cout << "the minimal pairs <= 20 um: " << distance << std::endl;
		count_20++;
	}
	if (distance<0.05&& distance>0.02)
	{
		std::cout << "the minimal pairs < 50 um and >20um: " << distance << std::endl;
		count_2050++;
	}
	if (distance<0.1&&distance>0.05)
	{
		std::cout << "the minimal pairs < 100 um and >50um: " << distance << std::endl;
		count_50100++;
	}
	if (distance>0.1&&distance<0.2)
	{
		std::cout << "the minimal pairs > 100 um and <200um: " << distance << std::endl;
		count_100200++;
	}
	if (distance>0.2)
	{
		std::cout << "the minimal pairs > 200um: " << distance << std::endl;
		count++;
	}
	distance = 0;
}

std::cout << "the number of pairs: " << (*rejCorres).size() << std::endl;
std::cout << "the number for pairs <= 20 um: " << count_20 << std::endl;
std::cout << "the number for pairs < 50 um and > 20um: " << count_2050 << std::endl;
std::cout << "the number for pairs < 100 um and >50um: " << count_50100 << std::endl;
std::cout << "the number for pairs < 200 um and >100um  : " << count_100200 << std::endl;
std::cout << "the number for pairs > 200 um  : " << count << std::endl;

string file_outpathrgb1 = "G:/mesh_test/rgbCloud1.ply";
pcl::io::savePLYFile(file_outpathrgb1, *rgbCloud1);

string file_outpathrgb2 = "G:/mesh_test/rgbCloud2.ply";
pcl::io::savePLYFile(file_outpathrgb2, *rgbCloud2);

//点云1和点云2_ori的精确点对,以点云1为匹配点云,点云2_ori为查询点云
pcl::CorrespondencesPtr correct_corres_set(new pcl::Correspondences());
pcl::Correspondence correct_corr_pair;
pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
kdtree.setInputCloud(frame_normal_cloud2_ori);
std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方
for (int ipoint1 = 0; ipoint1 < frame_normal_cloud1->size(); ipoint1++)
{
	if (0 != kdtree.nearestKSearch(frame_normal_cloud1->points[ipoint1], 5, pointIdxNKNSearch, pointNKNSquaredDistance))
	{
		correct_corr_pair.index_match = pointIdxNKNSearch[0];
		correct_corr_pair.index_query = ipoint1;
		correct_corres_set->push_back(correct_corr_pair);
	}
}

int count_correct = 0;
for (auto rej_c : *rejCorres)
{
	for (auto correct_c : *correct_corres_set)
	{
		if ((rej_c.index_query == correct_c.index_query) && (rej_c.index_match == correct_c.index_match))
		{
			count_correct++;
		}
	}

}
std::cout << "the number for correct pairs: " << count_correct << std::endl;

		}