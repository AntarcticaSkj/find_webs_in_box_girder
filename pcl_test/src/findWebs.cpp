#include "findWebs.h"

void FindWebs::runMain()
{
    {
        while(1)
        {

            // 点数太少不运行
            if (originCloud->points.size() < RunMainPointSizeThre)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            clock_t start_all = clock();
            ++runTimes;
            //*****************************************************************
            std::cout 
                << "\n\n*****************************************************************"
                << "\n\nThread : START RUN MAIN No."  << runTimes << "..."
                << "\n\n*****************************************************************"
                << std::endl;
            

            // saveCloudAsPCD(originCloud, runTimes, 1); // DE

            clock_t start, end;
            start = clock();
            filterAndCorrect(originCloud, correctedCloud);
            end = clock();
            std::cout << "filterAndCorrect time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
        
            // // 去除离群点，以便计算箱梁短轴上的坐标
            // octreeRemovePoints(correctedCloud, removeBadPointCloud);
            // start3 = clock();
            // std::cout << "octreeRemovePoints time = " << double(start3-start2)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
            
            start = clock();
            // 去除离群点，以便计算箱梁短轴上的坐标
            octreeRemovePoints(correctedCloud, removeBadPointCloud);
            end = clock();
            std::cout << "octreeRemovePoints time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
          

            *mapCloud += *removeBadPointCloud;
            if (!isPCAIntialized) // 如果没有初始化完成，则每次都需要更新transform
            {
                start = clock();
                correctMapCloudToY();
                end = clock();
                std::cout << "correctMapCloudToY time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
            }

            // if (!isPCAIntialized) // 如果没有初始化完成，则每次都需要更新transform
            // {
            //     *mapCloudDense += *originCloud;
            //     start = clock();
            //     correctMapCloudDense();
            //     end = clock();
            //     std::cout << "correctMapCloudDense time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
            // }


            // saveCloudAsPCD(removeBadPointCloud, runTimes, 3); // DE
            // saveCloudAsPCD(mapCloud, runTimes, 4); // DE

            start = clock();
            densityMatchY(mapCloud, peak_y_map);
            end = clock();
            std::cout << "densityMatchY time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）

            mergeWebs(); // 合并webs结果，暂时还没有用

            start = clock();

            densityMatchX(removeBadPointCloud, peak_x_map);
            // densityMatchZ(removeBadPointCloud, peak_z_map);
            // if (peak_z_map.size() != 1) {
            //     std::cout << "No peak z!" << std::endl;
            //     continue;
            // }
            // std::cout << "peak_z_map:" << peak_z_map[0] << std::endl;
            std::cout << "peak_x_map:" << peak_x_map[0] << " " << peak_x_map[1] << " " << peak_x_map[2] << std::endl;
            end = clock();
            std::cout << "densityMatchX time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
            
            if (!isPCAIntialized && peak_x_map.size() == 3 && ifCorrectMapCloudPitch)
            {
                std::cout << "Start correctMapCloudPitch! " << std::endl;
                start = clock();
                correctMapCloudPitch();
                end = clock();
                std::cout << "correctMapCloudPitch time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
            }

            // 更新X方向最值
            if (peak_x_map.size() == 3)
            {
                if (mapCloud_x_max == 0.0)
                {
                    mapCloud_x_min = peak_x_map[0];
                    mapCloud_x_max = peak_x_map[2];
                }
                else
                {
                    mapCloud_x_min = 1.0 * (runTimes - 1) / runTimes * mapCloud_x_min + 1.0 / runTimes * peak_x_map[0];
                    mapCloud_x_max = 1.0 * (runTimes - 1) / runTimes * mapCloud_x_max + 1.0 / runTimes * peak_x_map[1];
                }
            }
            MyPoint minP, maxP;
            pcl::getMinMax3D(*removeBadPointCloud, minP, maxP);         
            if (ifUseXMaxMin)
            {
                mapCloud_x_min = minP.x;
                mapCloud_x_max = maxP.x;
            }

            // 更新Z方向最值
            if (mapCloud_z_max == 0.0)
            {
                mapCloud_z_max = maxP.z;  // peak_z_map[0];
            }
            else
            {
                mapCloud_z_max = 1.0 * (runTimes - 1) / runTimes * mapCloud_z_max + 1.0 / runTimes * maxP.z;
            }


            // 重置 originCloud
            originCloud->clear();
            // pcl::transformPointCloud(*removeBadPointCloud, *originCloud, T_target_origin.inverse());
            // lastSize = originCloud->points.size();

            std::cout << "Now T_target_origin:\n" << T_target_origin << std::endl;

            std::cout << "x_min, x_max = " << mapCloud_x_min << " " << mapCloud_x_max << std::endl;
            std::cout << "z_min, z_max = " << mapCloud_z_min << " " << mapCloud_z_max << std::endl;

            clock_t end_all = clock();
            std::cout << "All csot time = " << double(end_all - start_all)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）

            // if (runTimes > 0)
            //     runPostProcess();
            outputResultFile();
            // 发布 供Rviz作图
            pubMap();
            pubVisualMarker();

            saveCloudAsPCD(mapCloud, runTimes, 5); // DE

            if (isFirstPCAcorrect) isFirstPCAcorrect = false;
            // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 至少间隔10秒再执行一次
        }
    }
}
void FindWebs::correctMapCloudToY()
{
    MyPointCloudPtr tmpCloud(new pcl::PointCloud<MyPoint>());
    // MyPointCloudPtr tmpCloud3(new pcl::PointCloud<MyPoint>());
    
    // pcl::transformPointCloud(*tmpCloud2, *mapCloud, T_target_origin * T_target_origin_before.inverse());
    
    Eigen::Matrix4f T_target_origin_before = T_target_origin;

    correctPC(mapCloud, tmpCloud, T_target_origin, true);

    if (std::isnan(T_target_origin.determinant()))
    {
        std::cout << "! Meet nan!" << std::endl;
        return;
    }

    // TODO 是否可以原地旋转？
    // pcl::copyPointCloud(*tmpCloud, *mapCloudDense);
    pcl::copyPointCloud(*tmpCloud, *mapCloud);

    if (checkTransConvergent(Eigen::Matrix4f::Identity(), T_target_origin))
    {
        isPCAIntialized = true;
        std::cout << "PCA is intialized!" << std::endl;
    }


    std::cout << "correctMapCloudToY:\n" << T_target_origin << std::endl;

    T_target_origin = T_target_origin * T_target_origin_before;
}


void FindWebs::correctMapCloudDense()
{
    MyPointCloudPtr tmpCloud(new pcl::PointCloud<MyPoint>());
    MyPointCloudPtr tmpCloud2(new pcl::PointCloud<MyPoint>());
    // MyPointCloudPtr tmpCloud3(new pcl::PointCloud<MyPoint>());
    

    Eigen::Matrix4f T_target_origin_before = T_target_origin;

    correctPC(mapCloudDense, tmpCloud, T_target_origin, true);
    
    // TODO 是否可以原地旋转？
    // pcl::copyPointCloud(*tmpCloud, *mapCloudDense);
    pcl::copyPointCloud(*mapCloud, *tmpCloud2);
    // pcl::transformPointCloud(*tmpCloud, *mapCloudDense, T_target_origin * T_target_origin_before.inverse());
    pcl::transformPointCloud(*tmpCloud2, *mapCloud, T_target_origin * T_target_origin_before.inverse());

    // T_target_origin = T_target_origin * T_target_origin_before;

    if (checkTransConvergent(T_target_origin_before, T_target_origin))
        isPCAIntialized = true;
}

void FindWebs::octreeRemovePoints(MyPointCloudPtr cloud, 
    MyPointCloudPtr output)
{
    std::cout << "cloud_in size = " << cloud->points.size() << "\n";
    std::cout << "Start octreeRemovePoints:\n";
    
    
    pcl::octree::OctreePointCloud<MyPoint> octree(octreeSize);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();   // 构建Octree
	std::vector<int> vec_point_index; //, vec_total_index;    //  体素内点的索引，   要删除的点的索引
	std::vector<pcl::octree::OctreeKey> vec_key;
    // 使用pcl index 滤波
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	// outliners->indices.resize(vec_total_index.size());

    // int print_test = 0;
	for (auto iter = octree.leaf_depth_begin(); iter != octree.leaf_depth_end(); ++iter)
	{
		auto key = iter.getCurrentOctreeKey();
		vec_key.emplace_back(key);
		auto it_key = octree.findLeaf(key.x, key.y, key.z);
		if (it_key != nullptr)
		{
			vec_point_index = iter.getLeafContainer().getPointIndicesVector();
			if (vec_point_index.size() < octreeRemoveSize)           
			{
				for (size_t i = 0; i < vec_point_index.size(); i++)
				{
					outliners->indices.push_back(vec_point_index[i]);
				}
			}
            else{
                int thre = octreeSavePointSize < vec_point_index.size() ? octreeSavePointSize : vec_point_index.size();
                for (int i = 0; i < vec_point_index.size() - thre; i++)
                {
                    outliners->indices.push_back(vec_point_index[i]);
                }
            }
		}
	}



	// for (size_t i = 0; i < vec_total_index.size(); i++)
	// {
	// 	outliners->indices[i] = vec_total_index[i];
	// }
	pcl::ExtractIndices<MyPoint> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);
	extract.filter(*output);
    std::cout << "after cloud size = " << output->points.size() << "\n";
}

// 在原始点云上标注角落点，可视化处理（非必须）
void FindWebs::printCornerInOriginCloud()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("showCloudWithCorner"));

	viewer->addCoordinateSystem(10, 0.0, 0.0, 0.0, "originCloud");
	viewer->setBackgroundColor (255, 255, 255);
	viewer->addPointCloud (originCloud, "originCloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"originCloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"originCloud");

    MyPointCloudPtr corner(new pcl::PointCloud<MyPoint>());
    MyPoint tmpPoint;
    Eigen::Vector4f targetP;
    Eigen::Vector4f originP;

    float x_l = mapCloud_x_min, x_r = mapCloud_x_max;
    for (auto&& c : peak_y_map)
    {
        targetP << x_l, c, 0.0, 1.0;
        originP = T_target_origin.inverse() * targetP;
        tmpPoint.x = originP[0];
        tmpPoint.y = originP[1];
        tmpPoint.z = originP[2];
        corner->points.emplace_back(tmpPoint);

        targetP << x_r, c, 0.0, 1.0;
        originP = T_target_origin.inverse() * targetP;
        tmpPoint.x = originP[0];
        tmpPoint.y = originP[1];
        tmpPoint.z = originP[2];
        corner->points.emplace_back(tmpPoint);
    }

    viewer->addPointCloud (corner, "corner");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0,"corner");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,20,"corner");



    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

// 将峰值转化为具体的坐标值并存储，同时输出短轴上的极值(mapCloud_x_min，mapCloud_x_max)
// void FindWebs::peakToCorner(std::vector<int>& peak)
// {   
//     // 寻找所保存的稀疏地图mapCloud的范围
//     MyPoint minP, maxP;
//     pcl::getMinMax3D(*mapCloud, minP, maxP);

//     if (peak.size() < 1)
//     {
//         std::cout << "No peak, so it is impossible to locate the corner!\n";
//         return;
//     }
        
//     mapCloud_x_min = minP.x; 
//     mapCloud_x_max = maxP.x;
//     std::cout << "Get x_l = " << mapCloud_x_min << ",\tget x_r = " << mapCloud_x_max << "\n";
//     // std::cout << "intervelLength = " << intervalLength << "\n";

//     corner_target.clear(); // 清零
//     for (auto&& p : peak)
//     {
//         float y = intervalLength * (p + 1);
//         corner_target.emplace_back(y);
//     }

// }

void FindWebs::downsampling(MyPointCloudPtr in, float size, MyPointCloudPtr& out)
{
    pcl::VoxelGrid<MyPoint> vg;

    vg.setInputCloud(in);
    vg.setLeafSize(size, size, size);
    vg.filter(*out);
}

// 找到cloud中Y轴方向的隔板位置
void FindWebs::densityMatchY(MyPointCloudPtr cloud, std::vector<float>& peak_Y)
{
    peak_Y.clear();

    // 计算稀疏地图点云的范围
    MyPoint minP, maxP;
    pcl::getMinMax3D(*cloud, minP, maxP);

    mapCloud_y_min = minP.y;
    mapCloud_y_max = maxP.y;
    // 计算密度
    std::vector<int> dens_c;
    getDensityY(cloud, dens_c, minP, maxP);


    //TO DE
    // std::cout << "Print density ===============" << std::endl;
	// for (auto d : dens_c)
    // {
    //     std::cout << d<< " ";
    // }
    // std::cout  << "\nPrint density ===============" << std::endl;

    // {
    //     std::cout << "Print dens:" << std::endl;
    //     for (auto d : dens_c)
    //     {
    //         std::cout << d << " " ;
    //     }
    //     std::cout << std::endl;
    // }  // DE


    int avg_sz = avg_len / intervalLength + 1;
    avg_sz = avg_sz > dens_c.size() ? dens_c.size() : avg_sz;
    float avg_sum = 0.0;
    for (int i = 0; i < avg_sz; ++i)
        avg_sum += dens_c[i];
    
    avg_sum /= avg_sz;

    int start_peak = -1; // 防止出现连续的峰
    int detect_len = peak_len / intervalLength + 1;
    int detect_end = -1;
    int last_peak = -1;
    for (int i = 0; i < dens_c.size(); ++i)
    {
        if (i >= avg_sz)
        {
            avg_sum -= 1.0 * dens_c[i - avg_sz] / avg_sz;
            avg_sum += 1.0 * dens_c[i] / avg_sz;
            // std::cout << "Now avg!" << avg_sum << "! ";
        }
        if (dens_c[i] > peak_judge_thre * avg_sum)
        {
            last_peak = i;
            if (start_peak == -1 && i >= detect_end)
            {
                start_peak = i;
                detect_end = i + detect_len;
            }
        }
        else if (start_peak != -1 && (i > detect_end | i == dens_c.size() - 1))
        {
            float peak_now = 1.0 * intervalLength * (last_peak + 1 + start_peak) / 2.0 + minP.y;
            // float peak_now = 1.0 * intervalLength * (start_peak) + minP.y;
            if (peak_now > 0)
            {
                peak_Y.emplace_back(peak_now);
                std::cout << last_peak - start_peak + 1<< "|" ; // peak 所占区间的长度
                std::cout << peak_now << " " ;
            }
            start_peak = -1;
            // detect_end = __INT_MAX__;
        }
    }
    std::cout << "\n";

}

// 找到cloud中X轴方向的隔板位置
void FindWebs::densityMatchX(MyPointCloudPtr cloud, std::vector<float>& peak_x_map)
{
    peak_x_map.clear();

    // 计算稀疏地图点云的范围
    MyPoint minP, maxP;
    pcl::getMinMax3D(*cloud, minP, maxP);

    // 计算密度
    
    std::vector<int> dens_c;
    getDensityX(cloud, dens_c, minP, maxP);

    int avg_sum = cloud->points.size() / dens_c.size();
    int last_peak = -1;
    for (int i = 0; i < dens_c.size(); ++i)
    {
        if (dens_c[i] > densityMatchX_thre * avg_sum)
        {
            peak_x_map.emplace_back(1.0 * intervalLength * (i) + minP.x);
            break;
        }
    }
    for (int i = dens_c.size() - 1; i >= 0; --i)
    {
        if (dens_c[i] > densityMatchX_thre * avg_sum)
        {
            peak_x_map.emplace_back(1.0 * intervalLength * (i + 1) + minP.x);
            while (i >= 0 && dens_c[i] > densityMatchX_thre * avg_sum )
                --i;
            peak_x_map.emplace_back(1.0 * intervalLength * (i) + minP.x);
            break;
        }
    }

}

// 找到cloud中Z轴方向的隔板位置
void FindWebs::densityMatchZ(MyPointCloudPtr cloud, std::vector<float>& peak_z_map)
{
    peak_z_map.clear();

    // 计算稀疏地图点云的范围
    MyPoint minP, maxP;
    pcl::getMinMax3D(*cloud, minP, maxP);

    // 计算密度
    
    std::vector<int> dens_c;
    getDensityZ(cloud, dens_c, minP, maxP);

    int avg_sum = cloud->points.size() / dens_c.size();
    int last_peak = -1;
    for (int i = dens_c.size() - 1; i >= 0; --i)
    {
        if (dens_c[i] > peak_judge_thre * avg_sum )
        {
            peak_z_map.emplace_back(1.0 * intervalLength * (i + 1) + minP.z);
            break;
        }
    }

}



void FindWebs::filterAndCorrect(MyPointCloudPtr origin, 
    MyPointCloudPtr& correct)
{
    MyPointCloudPtr tmpCloud1(new pcl::PointCloud<MyPoint>());
    MyPointCloudPtr tmpCloud2(new pcl::PointCloud<MyPoint>());
    // Eigen::Matrix4f trans1(Eigen::Matrix4f::Identity()), trans2(Eigen::Matrix4f::Identity());

    clock_t start = clock(), end;
    // 1. PCA粗矫正
    correctPC(origin, tmpCloud1, T_target_origin, isFirstPCAcorrect);
    end = clock();
    std::cout << "correctPC time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）

 
    
    // 2. 仅保存质心附近的点云
    start = clock();
    filterCloud(tmpCloud1, correct);
    end = clock();
    std::cout << "filterCloud time = " << double(end - start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
 
    // 3. PCA精矫正
    // correct->clear();
    // correctPC(tmpCloud2, correct, trans2);
    // saveCloudAsPCD(correct, runTimes, 2);

    //T_target_origin = trans1;
}

// 去除质心两端的数据
void FindWebs::filterCloud(MyPointCloudPtr cloud, MyPointCloudPtr& filtered)
{
    // 计算target点云的范围
    MyPoint minP, maxP;
    pcl::getMinMax3D(*cloud, minP, maxP);

    // 计算密度
    std::vector<int> dens_c;
    getDensityY(cloud, dens_c, minP, maxP);
    std::cout << "Before filter, cloud size = " << cloud->points.size() << std::endl;

    // 计算点云质心所在的区间
    Eigen::Vector4f centroid;//容量为4的列向量
    pcl::compute3DCentroid(*cloud, centroid);//计算目标点云质心

    int densSz = dens_c.size();
    int centroidIndex = (centroid[1] - minP.y) / intervalLength;
    centroidIndex = centroidIndex >= 0? centroidIndex : 0; // 因为质心坐标是经过映射得到的，存在误差。dens_c[densSz - 1]处是有足够余量的，考虑dens_c[0]处是否越界即可。

    int head = centroidIndex, tail = centroidIndex + 1;
    tail = tail < densSz? tail : tail - 2;

    int threshold = int(centroidNearRetainThre * cloud->points.size()); // 所需的点数阈值
    int pointSum = 0;
    while (pointSum < threshold)
    {
        pointSum += dens_c[head] + dens_c[tail];

        if (head - 1 >= 0)
            --head;
        if (tail + 1 < densSz)
            ++tail;
    }

    // 区间滤波
    float filter_y_min = ++head * intervalLength + minP.y;
    float filter_y_max = tail * intervalLength + minP.y;


    if (ifFilterShape)
    {
        filter_y_max = std::min(filter_y_max, centroid[1] + YShapePositive);
        filter_y_min = std::max(filter_y_min, centroid[1] - YShapeNegative);
        std::cout << "filter_y_min & max = " << filter_y_min << " " << filter_y_max << std::endl;
    }

    
    filtered->clear();
    for (auto&& point : cloud->points)
    {
        if (point.y < filter_y_max && point.y > filter_y_min)
            filtered->points.emplace_back(point);
    }
    std::cout << "filtered cloud size = " << filtered->points.size() << std::endl;

}


//计算从向量a -> 向量b 的旋转角度和旋转轴
void FindWebs::calRotation(Eigen::Vector3f u, Eigen::Vector3f v, double &angle, Eigen::Vector3f &vec)
{
	angle = acos(u.dot(v) / (u.norm()*v.norm()));
    // std::cout << "angle before: " << angle << std::endl;
	// if (angle > M_PI / 2)
	// {
	// 	u = -u;
	// 	angle = M_PI - angle;
	// }
    // std::cout << "angle after: " << angle << std::endl;
    
	float i, j, k;
    //std::cout << "u,v:" << u << " " << v << std::endl;
	i = u(1)*v(2) - u(2)*v(1), j = v(0)*u(2) - v(2)*u(0), k = u(0)*v(1) - u(1)*v(0);
	vec << i, j, k;
	double value = sqrt(i*i + j*j + k*k);
	vec(0) = vec(0) / value;
	vec(1) = vec(1) / value;
	vec(2) = vec(2) / value;
    //std::cout << "vec after: " << vec(0) << " " << vec(1) << " " << vec(2) << std::endl;
}

// 罗德里格斯法：旋转角度和旋转轴 -> 旋转矩阵
Eigen::Matrix4f FindWebs::RodriguesMatrixTranslation(Eigen::Vector3f n, double angle)
{
	//罗德里格斯公式求旋转矩阵
	Eigen::Matrix4f x_transform(Eigen::Matrix4f::Identity());
	x_transform(0, 0) = cos(angle) + n(0)*n(0)*(1 - cos(angle));
	x_transform(1, 0) = n(2)*sin(angle) + n(0)*n(1)*(1 - cos(angle));
	x_transform(2, 0) = -n(1)*sin(angle) + n(0)*n(2)*(1 - cos(angle));
	x_transform(0, 1) = n(0)*n(1)*(1 - cos(angle)) - n(2)*sin(angle);
	x_transform(1, 1) = cos(angle) + n(1)*n(1)*(1 - cos(angle));
	x_transform(2, 1) = n(0)*sin(angle) + n(1)*n(2)*(1 - cos(angle));
	x_transform(0, 2) = n(1)*sin(angle) + n(0)*n(2)*(1 - cos(angle));
	x_transform(1, 2) = -n(0)*sin(angle) + n(1)*n(2)*(1 - cos(angle));
	x_transform(2, 2) = cos(angle) + n(2)*n(2)*(1 - cos(angle));

	return  x_transform;
}

// 计算PCA特征向量的坐标点，用于显示
void FindWebs::calPCAResultCoordinateForDisplay(
    const PCAResult& res, MyPointCloudPtr pointsDisp)
{
    MyPoint A, B;
    A.x = res.centroid[0];
    A.y = res.centroid[1];
    A.z = res.centroid[2];

    B.x = res.centroid[0] + res.eigenvector.col(2)[0] * 100.0;
    B.y = res.centroid[1] + res.eigenvector.col(2)[1] * 100.0;
    B.z = res.centroid[2] + res.eigenvector.col(2)[2] * 100.0; 
    pointsDisp->points.emplace_back(A);
    pointsDisp->points.emplace_back(B);
    pointsDisp->points.emplace_back(A);
}

// 计算Y轴点云密度
void FindWebs::getDensityY(MyPointCloudPtr cloud, std::vector<int>& dst, 
    MyPoint minP, MyPoint maxP)
{
    // 计算输入的origin点云的范围
    // MyPoint minP, maxP;
    // pcl::getMinMax3D(*cloud, minP, maxP);
    // std::cout << "\n\nStart cal density:" << "\n";
    // std::cout << "minP = " << minP << "\nmaxP =" << maxP << "\ncloud->points.size()=" << cloud->points.size() << "\n";
    
    float L_x = maxP.x - minP.x;
    float L_y = maxP.y - minP.y;


    
    float l_min, l_max;

    l_min = minP.y;
    l_max = maxP.y;
    dst.resize(int((l_max - l_min) / intervalLength) + 1);
    for (auto& point : cloud->points)
    {
        ++dst[(int)((point.y - minP.y)/ intervalLength)];
    }
    // if (L_x > L_y) // x是长轴
    // {
    //     l_min = minP.x;
    //     l_max = maxP.x;
    //     dst.resize(int((l_max - l_min) / intervalLength) + 1);
    //     for (auto& point : cloud->points)
    //     {
    //         ++dst[(int)((point.x - minP.x)/ intervalLength)];
    //     }
    // }
    // else // y是长轴，已写死，正常运行一定是此种情况
    // {
    //     l_min = minP.y;
    //     l_max = maxP.y;
    //     dst.resize(int((l_max - l_min) / intervalLength) + 1);
    //     for (auto& point : cloud->points)
    //     {
    //         ++dst[(int)((point.y - minP.y)/ intervalLength)];
    //     }
    // }

    // // 打印密度
    // int print_avg_star = 10; // 平均每行要打印多少个星
    // int base = cloud->points.size() / (int((l_max - l_min) / intervalLength) + 1) / print_avg_star; // 一颗星代表多少个点
    // std::cout << "Point Cloud Size = " << cloud->points.size() << std::endl;
    // std::cout << "Number of parts = " << (int((l_max - l_min) / intervalLength) + 1) << std::endl;
    // std::cout << "Base = " << base << std::endl;
    // std::cout << "=== Print Density ===" << std::endl;
    // base = base == 0? 1 : base;
    
    // for(int i = 0; i < dst.size(); ++i)
    // {
    //     std::string s = "";

    //     for (int j = 1; j < (int)(0.5 + dst[i] / base); ++j)
    //         s += "+";
        
    //     std::cout << i << ":" << s << std::endl;
    // }
    // std::cout << "=== Print Done ===" << std::endl;
}

// 计算X轴点云密度
void FindWebs::getDensityX(MyPointCloudPtr cloud, std::vector<int>& dst, 
    MyPoint minP, MyPoint maxP)
{
    float L_x = maxP.x - minP.x;
    float L_y = maxP.y - minP.y;
    
    float l_min, l_max;

    l_min = minP.x;
    l_max = maxP.x;
    dst.resize(int((l_max - l_min) / intervalLength) + 1);
    for (auto& point : cloud->points)
    {
        ++dst[(int)((point.x - minP.x)/ intervalLength)];
    }
}


// 计算Z轴点云密度
void FindWebs::getDensityZ(MyPointCloudPtr cloud, std::vector<int>& dst, 
    MyPoint minP, MyPoint maxP)
{
    
    float l_min, l_max;

    l_min = minP.z;
    l_max = maxP.z;
    dst.resize(int((l_max - l_min) / intervalLength) + 1);
    for (auto& point : cloud->points)
    {
        ++dst[(int)((point.z - minP.z)/ intervalLength)];
    }
}

// 暂时没什么用
void FindWebs::showCloud2(MyPointCloudPtr cloud1, MyPointCloudPtr cloud2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("showCloud"));

    viewer->setBackgroundColor(255,255,255);

    int v1(0);
	viewer->createViewPort (0.0, 0.0, 0.5, 1, v1);
	viewer->addCoordinateSystem(10, 0.0, 0.0, 0.0, "cloud1", v1);
	viewer->setBackgroundColor (255, 255, 255, v1);
	viewer->addPointCloud (cloud1, "cloud1", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud1");

    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1, 1, v2);
    viewer->addCoordinateSystem(10, 0.0, 0.0, 0.0,"cloud2", v2);	
	viewer->setBackgroundColor (255, 255, 255, v2);
	viewer->addPointCloud (cloud2, "cloud2", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud2");
  
    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


// doPCA = true: 将点云长轴矫正到Y轴上，并输出变换矩阵; doPCA = false, 使用输入的变换矩阵做变换
void FindWebs::correctPC(MyPointCloudPtr origin, 
    MyPointCloudPtr& translation, Eigen::Matrix4f& T, bool doPCA)
{
    // PCAResult resTarget = calPCA(targetCloud);
    if (doPCA)
    {
        PCAResult res = calPCA(origin);

        Eigen::Vector3f n2;
        double angle2;
        calRotation(res.eigenvector.col(2), ones_y, angle2, n2);
        // std::cout << "angle:" << angle2 << "\tn2:" << n2 << std::endl; 
        T = RodriguesMatrixTranslation(n2, angle2);
        pcl::transformPointCloud(*origin, *translation, T);  //源点云整体旋转，最大主方向对齐y轴
    }
    else{
        pcl::transformPointCloud(*origin, *translation, T);  //源点云整体旋转，最大主方向对齐y轴
    }
	

    // 更新质心坐标
    
    // if (show)
    // {
    //     pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    //     viewer.initCameraParameters();

    //     // show
    //     int v1(0);
    //     viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    //     viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
    //     viewer.addText("Cloud before transforming", 10, 10, "v1 test", v1);
    //     viewer.addCoordinateSystem(0.5, v1);
    //     pcl::visualization::PointCloudColorHandlerCustom<MyPoint> color_v1(targetCloud, 0, 255, 0);
    //     viewer.addPointCloud<MyPoint>(targetCloud, color_v1, "color_v1", v1);
    //     viewer.addPointCloud<MyPoint>(origin, "source", v1);

    //     // 显示PCA特征向量
    //     MyPointCloudPtr coordinate1(new pcl::PointCloud<MyPoint>);
    //     MyPointCloudPtr coordinate2(new pcl::PointCloud<MyPoint>);
    //     calPCAResultCoordinateForDisplay(resSource, coordinate1);
    //     calPCAResultCoordinateForDisplay(resTarget, coordinate2);
    //     viewer.addPolygon<MyPoint>(coordinate1,0,0,0, "coordinate1", v1);
    //     viewer.addPolygon<MyPoint>(coordinate2,0,0,0, "coordinate2", v1);

    //     int v2(0);
    //     viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    //     viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    //     viewer.addText("Cloud after transforming", 10, 10, "v2 test", v2);
    //     viewer.addCoordinateSystem(0.5, v2);
    //     pcl::visualization::PointCloudColorHandlerCustom<MyPoint> color_v2(targetCloud, 0, 255, 0);
    //     viewer.addPointCloud<MyPoint>(targetCloud, color_v2, "color_v2", v2);
    //     viewer.addPointCloud<MyPoint>(translation, "translationCloud", v2);

    //     while (!viewer.wasStopped())
    //     {
    //         //在此处可以添加其他处理  
    //         viewer.spinOnce(100);
    //     }
    // }

}

// 计算点云的PCA结果
PCAResult FindWebs::calPCA(MyPointCloudPtr target)
{
	Eigen::Vector4f pcaCentroidtarget;//容量为4的列向量
	pcl::compute3DCentroid(*target, pcaCentroidtarget);//计算目标点云质心
	Eigen::Matrix3f covariance;//创建一个3行3列的矩阵，里面每个元素均为float类型
	pcl::computeCovarianceMatrixNormalized(*target, pcaCentroidtarget, covariance);//计算目标点云协方差矩阵
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);//构造一个计算特定矩阵的类对象
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//eigenvectors计算特征向量
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//eigenvalues计算特征值


    // std::cout << "pcaCentroidtarget: \n" << pcaCentroidtarget << std::endl;
    // std::cout << "eigenVectorsPCA: \n" << eigenVectorsPCA << std::endl;
    if (pcaCentroidtarget[0] * eigenVectorsPCA.col(2)[0] + pcaCentroidtarget[1] * eigenVectorsPCA.col(2)[1]+ pcaCentroidtarget[2] * eigenVectorsPCA.col(2)[2] < 0)
        eigenVectorsPCA.col(2) = -eigenVectorsPCA.col(2);

    // std::cout << "eigenVectorsPCA: \n" << eigenVectorsPCA << std::endl;
    PCAResult ans = {pcaCentroidtarget, eigenVectorsPCA, eigenValuesPCA};

    return ans;
} 

// 寻找箱梁点云中的筋板
void FindWebs::findBottomFlange(MyPointCloudPtr XLcloud)
{
    std::cout << "Read points : " << XLcloud->points.size() << std::endl;
    
    // 旋转到Y轴
    MyPointCloudPtr XLcloud_Y (new pcl::PointCloud<MyPoint>());
    Eigen::Matrix4f T_Y_XLcloud;
    correctPC(XLcloud, XLcloud_Y, T_Y_XLcloud, true);

    // 旋转到X轴
    MyPointCloudPtr XLcloud_X (new pcl::PointCloud<MyPoint>());
    Eigen::AngleAxisf rotation_vector(M_PI / 2, Eigen::Vector3f(0,0,-1));
    Eigen::Matrix3f rotation_matrix = rotation_vector.matrix();
    Eigen::Matrix4f T_X_Y;
    T_X_Y.block<3,3>(0,0) = rotation_matrix;
    pcl::transformPointCloud(*XLcloud_Y, *XLcloud_X, T_X_Y);

    // 计算点云的范围
    MyPoint minP, maxP;
    pcl::getMinMax3D(*XLcloud_X, minP, maxP);


    // 剔除不需要的点云
    std::cout << "Short axis min = " << minP.y << ", max = " << maxP.y << std::endl;
    float short_len = maxP.y - minP.y;
    float short_start = 2 * short_len / 5 + minP.y;
    float short_end = -short_len / 10 + maxP.y;
    float height_start = minP.z;
    float height_end = minP.z + short_len / 5;

    MyPointCloudPtr XLcloud_X_cut (new pcl::PointCloud<MyPoint>());
    for (auto&& p : XLcloud_X->points)
    {
        if (p.y > short_start && p.y < short_end && p.z > height_start && p.z < height_end)
        {
           XLcloud_X_cut->points.push_back(p);
        }
    }
    std::cout << "XLcloud_X_cut size = " << XLcloud_X_cut->points.size() << std::endl;


    // X轴方向上求取密度
    std::vector<float> bottomFlangePeak;
    densityMatchY(XLcloud_X_cut, bottomFlangePeak);

    std::string path = "/home/sunkejia/savePCD/XLcloud_X_cut.pcd";
    pcl::PCDWriter writer;
    XLcloud_X_cut->height = XLcloud_X_cut->points.size();
    XLcloud_X_cut->width = 1;
    writer.write(path, *XLcloud_X_cut);
}


bool FindWebs::checkTransConvergent(Eigen::Matrix4f T1, Eigen::Matrix4f T2)
{
    Eigen::Matrix3f R = T1.block<3,3>(0,0) * (T2.block<3,3>(0,0)).transpose();

    float theta =  acos((R.trace() - 1.0) / 2.0);
    std::cout << "now theta : " << theta << std::endl;

    if (abs(theta < 0.0))
        return true;
    else
        return false;
}

void FindWebs::mergeWebs()
{

}


void FindWebs::pubMap()
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    if (mapCloud->points.size() > 0)
    {
        pcl::toROSMsg(*mapCloud, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time();
        laserCloudmsg.header.frame_id = "camera_init";
        pub_map.publish(laserCloudmsg);
    }
}
void FindWebs::pubVisualMarker()
{
    std::cout << "pubVisualMarker" << std::endl;
    // 1.发布平面
    visualization_msgs::Marker marker_;
    marker_.header.stamp = ros::Time();
    marker_.header.frame_id = "camera_init";
    marker_.ns = "marker_planes";
    marker_.id = 0;
    //set marker type
    marker_.type = visualization_msgs::Marker::CUBE_LIST;

    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    //set marker scale
    marker_.scale.x = mapCloud_x_max - mapCloud_x_min; //m
    marker_.scale.y = 0.04;
    marker_.scale.z = mapCloud_z_max - mapCloud_z_min;

    //decide the color of the marker
    marker_.color.a = 1; // Don't forget to set the alpha!
    marker_.color.r = 0.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.5;

    for (size_t i = 0; i < peak_y_map.size(); i++)
    {
        Eigen::Vector4f v1, v2;
        v1[0] = (mapCloud_x_max + mapCloud_x_min) / 2.0;
        v1[1] = peak_y_map[i];
        v1[2] = (mapCloud_z_max + mapCloud_z_min) / 2.0;
        v1[3] = 1.0;

        //v2 = T_target_origin.inverse() * v1;
        v2 = v1;
        geometry_msgs::Point center_pos;
        center_pos.x = v2[0];
        center_pos.y = v2[1];
        center_pos.z = v2[2];
        marker_.points.push_back(center_pos);  
    }

    //set marker action
    marker_.action = visualization_msgs::Marker::MODIFY;
    marker_.lifetime = ros::Duration(); //(sec,nsec),0 forever
    pub_marker_webs.publish(marker_);

    // 2. 发布直线
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "camera_init";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "marker_lines";
    // line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.05;
    // Line list is red
    line_list.color.b = 1;
    line_list.color.a = 0.7;
    // Create the vertices for the points and lines
    geometry_msgs::Point p;

    // line1
    p.x = mapCloud_x_min;
    p.y = mapCloud_y_min;
    p.z = mapCloud_z_min;
    line_list.points.push_back(p);
    p.y = mapCloud_y_max;
    line_list.points.push_back(p);
    // line2
    p.x = mapCloud_x_min;
    p.y = mapCloud_y_min;
    p.z = mapCloud_z_max;
    line_list.points.push_back(p);
    p.y = mapCloud_y_max;
    line_list.points.push_back(p);
    // line3
    p.x = mapCloud_x_max;
    p.y = mapCloud_y_min;
    p.z = mapCloud_z_max;
    line_list.points.push_back(p);
    p.y = mapCloud_y_max;
    line_list.points.push_back(p);
    // line4
    p.x = mapCloud_x_max;
    p.y = mapCloud_y_min;
    p.z = mapCloud_z_min;
    line_list.points.push_back(p);
    p.y = mapCloud_y_max;
    line_list.points.push_back(p);

    pub_marker_webslines.publish(line_list);

}
void FindWebs::outputResultFile()
{
    // 输出数组到文件
    ofstream outfile;
    outfile.open(result_path, ios::app);
    if(!outfile) //检查文件是否正常打开//不是用于检查文件是否存在
    {
        std::cout << "result.txt can't open" << std::endl;
    }
    else
    {
        outfile << mapCloud_x_min << " " << mapCloud_x_max << " " << mapCloud_z_min << " " << mapCloud_z_max << " ";
        for (auto y : peak_y_map) 
            outfile << y << " ";
        outfile << std:: endl;
        outfile.close();
    }
}
void FindWebs::runPostProcess()
{
    for (int i = 0; i < peak_y_map.size(); i++)
    {
        all_webs[i].emplace_back(peak_y_map[i]);
        webs_detect_times[i]++;
    }
    webs_avg.clear();
    webs_std.clear();

    std::cout << "~~~~~~~~~~~\n(avg, std) = ";
    for (int i = 0; i < webs_detect_times.size(); i++)
    {
        if (webs_detect_times[i] == 0)
            continue;
        
        float avg = 0.0, std = 0.0;
        for (auto y : all_webs[i])
            avg += y;
        avg /= webs_detect_times[i];

        webs_avg.emplace_back(avg);
        if (webs_detect_times[i] == 1)
            webs_std.emplace_back(std);
        else
        {

            for (auto y : all_webs[i])
                std += pow(y - avg, 2);
            std /= webs_detect_times[i] - 1;
            std = sqrt(std);
            webs_std.emplace_back(std);
        }
        std::cout << "(" << avg << "," << std << ") ";

    }
    std::cout << "\n~~~~~~~~~~~\n";

}
void FindWebs::correctMapCloudPitch()
{
    if (peak_x_map.size() != 3)
        return;
    MyPointCloudPtr tmpMap(new pcl::PointCloud<MyPoint>());
    // pcl::copyPointCloud(*mapCloud, *tmpMap);

    pcl::PassThrough<MyPoint> pass;   //定义一个直通滤波类
    pass.setInputCloud(mapCloud);              //设置数据集
    pass.setFilterFieldName("x");           //指定滤波纬度
    pass.setFilterLimits(peak_x_map[1] - 0.1, peak_x_map[1] + 0.3);         //指定滤波范围
    pass.setNegative(false);                 //true:过滤掉指定范围的点，保留范围外的点；false时相反
    pass.filter(*tmpMap);           //执行滤波，返回滤波后的数据


    PCAResult ans = calPCA(tmpMap);
    std::cout << "plane normal: \n" << ans.eigenvector.col(0) << std::endl;

    float theta = 0.0;
    if (ans.eigenvector.col(0)[0] * ans.eigenvector.col(0)[2] >= 0)
        theta = 1.0;
    else
        theta = -1.0;
    theta *= acos(abs(ans.eigenvector.col(0)[0]));
    std::cout << "theta:" << theta << std::endl;

    if (std::isnan(theta))
        return;
    Eigen::Matrix4f pitch_transform;
    pitch_transform << cos(theta), 0, sin(theta), 0,
                        0, 1, 0, 0,
                        -sin(theta), 0, cos(theta), 0,
                        0, 0, 0, 1;

    pcl::copyPointCloud(*mapCloud, *tmpMap);                   
    pcl::transformPointCloud(*tmpMap, *mapCloud, pitch_transform);

    T_target_origin = pitch_transform * T_target_origin;
}