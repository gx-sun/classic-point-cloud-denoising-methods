# classic-point-cloud-denoising-methods
经典点云去噪算法代码总结
​
1.移动最小二乘MLS

基于PCL

```
#include "stdafx.h"


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
int main(int argc, char** argv)
{
	// 将一个适当类型的输入文件加载到对象PointCloud中
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// 加载bun0.pcd文件，加载的文件在 PCL的测试数据中是存在的 
	pcl::io::loadPCDFile("bunny_hi_noise.pcd", *cloud);
	// 创建一个KD树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//设置参数
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.06);
	// 曲面重建
	mls.process(mls_points);
	// 保存结果
	pcl::io::savePCDFile("bunny_hi_noise_mls.pcd", mls_points);
	
}
```

2.双边滤波

2.1基于PCL

实验编译成功了，但是因为其需要intensity分量，一般都没有，故无法运行 

```
#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>

typedef pcl::PointXYZI PointT;

int
main(int argc, char *argv[])
{
	std::string incloudfile = "block_noise.pcd";
	std::string outcloudfile = "block_bf.pcd";
	float sigma_s = 0.1;
	float sigma_r = 0.2;

	// Load cloud
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(incloudfile.c_str(), *cloud);

	pcl::PointCloud<PointT> outcloud;

	// Set up KDTree
	//pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
	pcl::search::KdTree<PointT>::Ptr tree1(new pcl::search::KdTree<PointT>);
	pcl::BilateralFilter<PointT> bf;
	bf.setInputCloud(cloud);
	bf.setSearchMethod(tree1);
	bf.setHalfSize(sigma_s);
	bf.setStdDev(sigma_r);
	bf.filter(outcloud);

	// Save filtered output
	pcl::io::savePCDFile(outcloudfile.c_str(), outcloud);
	return (0);
}
```

2.2 基于CGAL

官方example代码，无需强度分类，只需点位置和法线信息，运行成功版本：

```
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/tags.h>

#include <utility> // defines std::pair
#include <fstream>

// Types
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;

// Concurrency
typedef CGAL::Parallel_if_available_tag Concurrency_tag;

int main(int argc, char*argv[])
{
  const char* input_filename =  (argc>1)?argv[1]:"data/fin90_with_PCA_normals.xyz";
  const char* output_filename = (argc>2)?argv[2]:"data/fin90_with_PCA_normals_bilateral_smoothed.xyz";

  // Reads a .xyz point set file in points[] * with normals *.
  std::vector<PointVectorPair> points;
  std::ifstream stream(input_filename);
  if (!stream ||
      !CGAL::read_xyz_points(stream,
                     std::back_inserter(points),
                     CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
                     normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
  {
     std::cerr << "Error: cannot read file " << input_filename << std::endl;
     return EXIT_FAILURE;
  }

  // Algorithm parameters
  int k = 120;                 // size of neighborhood. The bigger the smoother the result will be.
                               // This value should bigger than 1.
  double sharpness_angle = 25; // control sharpness of the result.
                               // The bigger the smoother the result will be
  int iter_number = 3;         // number of times the projection is applied

  for (int i = 0; i < iter_number; ++i)
  {
    /* double error = */
    CGAL::bilateral_smooth_point_set <Concurrency_tag>(
      points,
      k,
      CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
      normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()).
      sharpness_angle (sharpness_angle));
  }

  //// Save point set.
  std::ofstream out(output_filename);
  out.precision(17);
  if (!out ||
      !CGAL::write_xyz_points(
      out, points,
      CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
      normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
  {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
```


3.WLOP

基于CGAL：

安装CGAL可以参考：https://blog.csdn.net/Meet_csdn/article/details/110290766

安装好后的工程文件是CGAL自带的，在文件夹example/Point_set_processing_3/wlop_simplify_and_regularize_point_set_example,运行结果如下：

![image](https://user-images.githubusercontent.com/42514510/145545864-543ce201-e99c-45ad-83df-f4bcae31bd8c.png)


原始模型

![image](https://user-images.githubusercontent.com/42514510/145545894-e430b6c2-7cbf-4b61-8c6e-0719c1f3c274.png)

去噪下采样后模型

                       

参考代码：https://doc.cgal.org/latest/Point_set_processing_3/index.html#C

文章：Huang H, Li D, Zhang H, et al. Consolidation of unorganized point clouds for surface reconstruction[J]. ACM transactions on graphics (TOG), 2009, 28(5): 1-7.

4.CLOP

官方实现，https://www.cg.tuwien.ac.at/research/publications/2014/preiner2014clop/

对应文章："Reinhold Preiner, Oliver Mattausch, Murat Arikan, Renato Pajarola, Michael Wimmer
Continuous Projection for Fast L1 Reconstruction
ACM Transactions on Graphics (Proc. of ACM SIGGRAPH 2014), 33(4):47:1-47:13, August 2014."

下载下来后可以运行，速度还不错，会对点云下采样

5.EAR

官方代码，有exe版本和源码版本，exe可以双击运行。

参考地址：https://vcc.tech/research/2013/EAR

文章：Huang H, Wu S, Gong M, et al. Edge-aware point set resampling[J]. ACM transactions on graphics (TOG), 2013, 32(1): 1-12.


6.PCN(point-clean-net)

可以去除离群点和噪声点

地址：https://github.com/mrakotosaon/pointcleannet


7.TD(total denoising)

第一篇点云无监督去噪方法，结果偏向平滑。

地址：https://github.com/phermosilla/TotalDenoising
复现细节版本：https://blog.csdn.net/Meet_csdn/article/details/119615049?spm=1001.2014.3001.5501

8.Pointfilter

目前SOTA，效果比较好,编码解码结构，简单有效

地址：https://github.com/dongbo-BUAA-VR/Pointfilter

​
