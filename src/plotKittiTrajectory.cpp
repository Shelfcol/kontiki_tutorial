#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
#include <fstream>
#include "kontiki_tutorial/global_definition.h"
#include <yaml-cpp/yaml.h>
// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;


void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);
Eigen::Quaterniond euler2quat(const double fai, const double theta, const double psi);


int main(int argc, char **argv) {

  std::string config_file_path = WORK_SPACE_PATH+"/config/vis_config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::string trajectory_file = config_node["trajectory_file_path"].as<std::string>();
  std::string data_type = config_node["data_type"].as<std::string>();


  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  ifstream fin(trajectory_file);
  if (!fin) {
    cout << "cannot find trajectory file at " << trajectory_file << endl;
    return 1;
  }

  if(data_type == "TUM")
  {
    while (!fin.eof()) {  //fin.eof()判断文件是否为空
      double time, tx, ty,tz,qx,qy,qz,qw;
      fin >> time >> tx >> ty >>tz>> qx>>qy>>qz>>qw;
      Eigen::Quaterniond q(qw,qx,qy,qz);
      Isometry3d Twr(q);  //变换矩阵的旋转部分
      Twr.pretranslate(Vector3d(tx, ty, tz));//变换矩阵的平移部分
      poses.push_back(Twr);
    }
  }
  else if(data_type == "KITTI")
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
                 
    while (!fin.eof()) {  //fin.eof()判断文件是否为空
      double time, tx, ty,tz;
      fin>> R(0,0)>>R(0,1)>>R(0,2)>>tx>>
                    R(1,0)>>R(1,1)>>R(1,2)>>ty>>
                    R(2,0)>>R(2,1)>>R(2,2)>>tz;
      Eigen::Quaterniond q(R);
      Isometry3d Twr(q);  //变换矩阵的旋转部分
      Twr.pretranslate(Vector3d(tx, ty, tz));//变换矩阵的平移部分
      poses.push_back(Twr);
    }
  }
  else
  {
    printf("wrong data type");
    return 0;
  }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

Eigen::Quaterniond euler2quat(const double fai, const double theta, const double psi)
{
  Eigen::Quaterniond q;
  double fai_2=fai/2;
  double theta_2=theta/2;
  double psi_2=psi/2;
  q.w()=cos(fai_2)*cos(theta_2)*cos(psi_2)+sin(fai_2)*sin(theta_2)*sin(psi_2);
  q.x()=sin(fai_2)*cos(theta_2)*cos(psi_2)-cos(fai_2)*sin(theta_2)*sin(psi_2);
  q.y()=cos(fai_2)*sin(theta_2)*cos(psi_2)+sin(fai_2)*cos(theta_2)*sin(psi_2);
  q.z()=cos(fai_2)*cos(theta_2)*sin(psi_2)-sin(fai_2)*sin(theta_2)*cos(psi_2);
  return q;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size()-1; i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}
