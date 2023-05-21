#include <chrono> // 包含std::chrono头文件
#include <thread> // 包含std::thread头文件
#include <cmath>

// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <pcl/PolygonMesh.h>
// #include <pcl/TextureMesh.h>
#include <pcl/common/geometry.h>
// #include <pcl/common/intersections.h>
#include <pcl/features/board.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/visualization/common/actor_map.h>

// 叉积
pcl::PointXYZ xmulti(pcl::PointXYZ p1,pcl::PointXYZ p2){
    return pcl::PointXYZ(p1.y*p2.z-p1.z*p2.y,p1.z*p2.x-p1.x*p2.z,p1.x*p2.y-p1.y*p2.x);
}
// 点积
double dmulti(pcl::PointXYZ p1,pcl::PointXYZ p2){
    return p1.x*p2.x+p1.y*p2.y+p1.z*p2.z;
}
// 向量之差
pcl::PointXYZ sub(pcl::PointXYZ p1,pcl::PointXYZ p2){
     return pcl::PointXYZ(p1.x-p2.x,p1.y-p2.y,p1.z-p2.z);
}
pcl::PointXYZ IsLinePlaneIntersected(pcl::PointXYZ p1,pcl::PointXYZ p2,pcl::PointXYZ s1,pcl::PointXYZ s2,pcl::PointXYZ s3){
    pcl::PointXYZ vec = xmulti(sub(s1,s2),sub(s2,s3));
    double bi=dmulti(vec,sub(s1,p1))/dmulti(vec,sub(p2,p1));  //三菱锥体积之比
    vec.x=p1.x+(p2.x-p1.x)*bi;
    vec.y=p1.y+(p2.y-p1.y)*bi;
    vec.z=p1.z+(p2.z-p1.z)*bi;
    return vec;
}
// 判断两点在平面同侧  点积等于0表示在平面上
bool IsLineOnPlane(pcl::PointXYZ p1,pcl::PointXYZ p2,pcl::PointXYZ s1,pcl::PointXYZ s2,pcl::PointXYZ s3){
// std::cout << "IsLineOnPlane"<< std::endl;
    pcl::PointXYZ v=xmulti(sub(s1,s2),sub(s2,s3));  //平面法向量
    return dmulti(v,sub(s1,p1))*dmulti(v,sub(s1,p2)) > 1e-7;
}
// 判断线和平面平行
bool IsLinePlaneParal(pcl::PointXYZ p1,pcl::PointXYZ p2,pcl::PointXYZ s1,pcl::PointXYZ s2,pcl::PointXYZ s3){
// std::cout << "IsLinePlaneParal"<< std::endl;
    return dmulti(sub(p1,p2),xmulti(sub(s1,s2),sub(s2,s3))) < 1e-7;
}

bool IsPointOnSgement(
    const pcl::PointXYZ& point,    const pcl::PointXYZ& start_point,
    const pcl::PointXYZ& end_point) {
  // 判断点是否在线段上
  float threshold = 0.01;
  Eigen::Vector3f pt_vec(point.x, point.y, point.z);
  Eigen::Vector3f start_vec(start_point.x, start_point.y, start_point.z);
  Eigen::Vector3f end_vec(end_point.x, end_point.y, end_point.z);
  float line_len = (end_vec - start_vec).norm();
  float len1 = (pt_vec - start_vec).norm();
  float len2 = (pt_vec - end_vec).norm();
  return fabs(len1 + len2 - line_len) < threshold;
}


bool IsLinePolygonMeshIntersected(
    const pcl::PointXYZ& start_point,
    const pcl::PointXYZ& end_point,
    const pcl::PolygonMesh& mesh,
    const pcl::visualization::PCLVisualizer::Ptr& polygon_viewer,
    pcl::PointXYZ * intersect_point_3d) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
    polygon_viewer->spinOnce(50);
    for (size_t i =0 ; i < mesh.polygons.size(); i++) {
      auto polygon = mesh.polygons[i];
      pcl::PointXYZ polygon_point_1;
      pcl::PointXYZ polygon_point_2;
      pcl::PointXYZ polygon_point_3;
      pcl::PointCloud<pcl::PointXYZ> polygon_points_3d;
      pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_points_3d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr intersect_point_on_mesh(new pcl::PointCloud<pcl::PointXYZ>);
      polygon_viewer->removePointCloud("polygon_cloud");
      polygon_viewer->removePointCloud("intersect_point_on_mesh");
        polygon_viewer->addPointCloud(polygon_points_3d_ptr, "polygon_cloud");
      for (size_t j =0 ; j < polygon.vertices.size(); j++) {
        auto index = polygon.vertices[j];
        pcl::PointXYZ polygon_point((*mesh_cloud)[index]);
        if (j == 0) polygon_point_1 = polygon_point;
        if (j == 1) polygon_point_2 = polygon_point;
        if (j == 2) polygon_point_3 = polygon_point;
        polygon_points_3d.push_back(polygon_point);
        polygon_points_3d_ptr->push_back(polygon_point);
      }
        polygon_viewer->updatePointCloud(polygon_points_3d_ptr, "polygon_cloud");
        polygon_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "polygon_cloud");
        polygon_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "polygon_cloud");

        polygon_viewer->addPolygon<pcl::PointXYZ>(polygon_points_3d_ptr, "polygon"+std::to_string(i));
        polygon_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "polygon"+std::to_string(i));
        polygon_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "polygon"+std::to_string(i));
      polygon_viewer->spinOnce(10);

      // std::cout << "continue"<< std::endl;
      // if (IsLineOnPlane(start_point,end_point,
      //           polygon_point_1,polygon_point_2,polygon_point_3) ||
      //     IsLinePlaneParal(start_point,end_point,
      //           polygon_point_1,polygon_point_2,polygon_point_3)) {
      //   continue;
      // }

      *intersect_point_3d = IsLinePlaneIntersected(
        start_point,end_point,polygon_point_1,polygon_point_2,polygon_point_3);
      bool is_in_polygon = pcl::isPointIn2DPolygon(*intersect_point_3d, polygon_points_3d);
      // if (1) {
      //   polygon_viewer->addLine(start_point,*intersect_point_3d, 1, 1, 0, "line_points_3d");
      //   polygon_viewer->setShapeRenderingProperties(
      //           pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "line_points_3d");
      //   polygon_viewer->spinOnce(10);
      //   polygon_viewer->removeShape("line_points_3d");
      //   }
      if (is_in_polygon) {
        std::cout << "   " << is_in_polygon << "   "<< std::endl;
        intersect_point_on_mesh->push_back(*intersect_point_3d);
        polygon_viewer->updatePointCloud(intersect_point_on_mesh, "intersect_point_on_mesh");
        polygon_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0, "intersect_point_on_mesh");
        polygon_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "intersect_point_on_mesh");
        polygon_viewer->spinOnce(500);
        // while(!polygon_viewer->wasStopped())
        // {
        //   polygon_viewer->spinOnce(100);
        //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }
        polygon_viewer->spinOnce(500);
        pcl::visualization::ShapeActorMapPtr shape_actor_map;
        shape_actor_map = polygon_viewer->getShapeActorMap();
        for (size_t remove_index = 0; remove_index <= i; remove_index++) {
          std::string objectName = "polygon"+std::to_string(remove_index);  // 待删除的物体名称
          polygon_viewer->removeShape(objectName);
        }

        return is_in_polygon;
      }
    }

    return false;
}

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "my_ros_tool");
  // ros::NodeHandle nh;

  // 定义一个激光雷达数据
  // ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 1);
  // sensor_msgs::LaserScan scan_msg;
  // // 设置LaserScan消息的参数
  // scan_msg.header.frame_id = "base_laser_link";
  // scan_msg.angle_min = -M_PI / 2;
  // scan_msg.angle_max = M_PI / 2;
  // scan_msg.angle_increment = M_PI / 180;
  // scan_msg.time_increment = 0.1;
  // scan_msg.scan_time = 1;
  // scan_msg.range_min = 0;
  // scan_msg.range_max = 100;


  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  float angle = M_PI / 2;
  transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));
  // pcl::PointCloud<pcl::PolygonMesh> vehicle_mesh;
  pcl::PolygonMesh vehicle_mesh;
  pcl::io::loadOBJFile("../vehicle_body.obj", vehicle_mesh);
  pcl::PointCloud<pcl::PointNormal>::Ptr transformed(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromPCLPointCloud2(vehicle_mesh.cloud, *transformed);
  pcl::transformPointCloud(*transformed, *transformed, transform);
  pcl::toPCLPointCloud2 (*transformed, vehicle_mesh.cloud);

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> colorHandler(
  //   transformed, 255, 255, 255);
  // viewer->addPointCloud<pcl::PointNormal>(transformed, colorHandler, "cloud_without_normal");
  // viewer->addPointCloudNormals<pcl::PointNormal>(
  //   transformed, 10, 0.05, "cloud_normal");

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
  viewer->addCoordinateSystem(3);
  viewer->addPolygonMesh(vehicle_mesh, "vehicle_mesh");
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "vehicle_mesh");
  viewer->spinOnce(100);

  pcl::PointXYZ start_point(1.5,0.0,1.6);
  const float distance = 10;
  const float pitch_start_angle = 90 -25; // ++
  const float pitch_end_angle = 25.; // --
  const float yaw_resolution = 0.1;
  const float pitch_resolution = 25./64.;
  const size_t total_yaw_counts = 360.0 / yaw_resolution;// (0,360)
  const size_t total_pitch_counts = 128;// (0,360)
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr intersect_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      viewer->addPointCloud(line_points, "line_points");
      viewer->addPointCloud(intersect_cloud, "intersect_cloud");
  for (size_t i = 0; i < total_pitch_counts; i++) {
  // for (size_t i = total_pitch_counts-1; i > 0; i--) {
      if (i%20 != 0) continue;
      // line_points->clear();
      // line_points->push_back(start_point);
    // std:: cout << "----------------------------------------" << std::endl;

    for (size_t j = 0; j < total_yaw_counts; j++) {
    // for (size_t j = total_yaw_counts; j > 0 ; j--) {
      if (j%200 != 0) continue;
      // viewer->removeShape("line"+std::to_string(i)+"_"+std::to_string(j));
      float pitch = (pitch_start_angle + i * pitch_resolution)/ 180 * 3.14;
      float yaw = (j * yaw_resolution) / 180 * 3.14;
      // std:: cout << "patch: " << pitch << std::endl;
      // std:: cout << "yaw: " << yaw << std::endl;
      float x = distance * std::sin(pitch) * std::cos(yaw) + 1.5;
      float y = distance * std::sin(pitch) * std::sin(yaw) + 1.6;
      float z = distance * std::cos(pitch) + 0.50;
      pcl::PointXYZ end_point(x,y,z);
      line_points->push_back(end_point);
      viewer->updatePointCloud(line_points, "line_points");
      viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "line_points");
      viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "line_points");

      pcl::PointXYZ intersect_point;
      if (IsLinePolygonMeshIntersected(start_point,end_point,vehicle_mesh,viewer,&intersect_point)) {
          intersect_cloud->push_back(intersect_point);
          if (IsPointOnSgement(intersect_point,start_point,end_point)) {
            viewer->addLine(start_point,end_point, 1.0, 1.0, 0, "line"+std::to_string(i)+"_"+std::to_string(j));
          }
          else {
            viewer->addLine(start_point,end_point, 1.0, 0.0, 0, "line"+std::to_string(i)+"_"+std::to_string(j));
          }
          viewer->spinOnce(100);
      }
    }
    viewer->spinOnce(100);
    // viewer->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  viewer->updatePointCloud(intersect_cloud, "intersect_cloud");
        viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "intersect_cloud");
        viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "intersect_cloud");

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    // viewer->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ros::Rate loop_rate(10);
  // while (ros::ok())
  // {
  //   // 发布一个激光雷达数据
  //   scan_msg.header.stamp = ros::Time::now();
  //   for (int i = 0; i < scan_msg.ranges.size(); i++)
  //   {
  //     scan_msg.ranges[i] = i;
  //   }
  //   laser_pub.publish(scan_msg);

  //   loop_rate.sleep();
  // }

  return 0;
}






bool UpSampleFromPolygon(const pcl::PolygonMesh& input_mesh, 
                         pcl::PolygonMesh& output_mesh) {
  

}


