#include "voxelslam.hpp"
#include "global_correction.hpp"
#include "btc_io.hpp"
#include "imu_rate_predictor.hpp"
#include "localization_state.hpp"
#include "dual_tf.hpp"
#include "global_match_audit.hpp"
#include "direct_matching.hpp"
#include "scancontext_relocalization.hpp"
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "localization_display.hpp"
#include "localization_btc.hpp"
#include "localization_history.hpp"
#include "scan_map.hpp"
#include "mapping_capture.hpp"
#include "prior_map_tracker.hpp"
#include "relocalizer.hpp"
#include "session_aligner.hpp"
#include "spatial_map_manager.hpp"
#include "spatial_map_matcher.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <atomic>
#include <boost/bind.hpp>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <new>
#include <stdexcept>
#include <unordered_set>

using namespace std;

namespace {

struct SpatialFrameLogScope {
  std::function<void()> write;
  ~SpatialFrameLogScope() { if (write) write(); }
};

CorrectionState toCorrectionState(const IMUST &state) {
  CorrectionState result;
  result.R = state.R;
  result.p = state.p;
  result.v = state.v;
  result.bg = state.bg;
  result.ba = state.ba;
  result.g = state.g;
  result.cov = state.cov;
  return result;
}

void updateFromCorrectionState(const CorrectionState &source, IMUST &target) {
  target.R = source.R;
  target.p = source.p;
  target.v = source.v;
  target.bg = source.bg;
  target.ba = source.ba;
  target.g = source.g;
  target.cov = source.cov;
}

double spatialSnapshotMemoryMb(
    const std::shared_ptr<const SpatialMapSnapshot> &snapshot) {
  if (!snapshot) return 0.0;
  std::size_t bytes = sizeof(SpatialMapSnapshot);
  bytes += snapshot->blocks.size() *
           sizeof(std::pair<const BlockIndex,
                            std::shared_ptr<const StaticMapBlock>>);
  for (const auto &entry : snapshot->blocks) {
    if (!entry.second) continue;
    const StaticMapBlock &block = *entry.second;
    bytes += sizeof(StaticMapBlock);
    bytes += block.roots.capacity() * sizeof(RootVoxelRecord);
    bytes += block.nodes.capacity() * sizeof(StaticOctreeNode);
    bytes += block.planes.capacity() * sizeof(StaticPlane);
    bytes += block.display_points.capacity() * sizeof(SpatialMapPoint);
  }
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

}  // namespace

class ResultOutput
{
public:
  static ResultOutput &instance()
  {
    static ResultOutput inst;
    return inst;
  }

  void pub_odom_func(IMUST &xc, bool publish_scan_tf,
                     const string &odom_frame, const string &child_frame)
  {
    if(!publish_scan_tf) return;
    Eigen::Quaterniond q_this(xc.R);
    Eigen::Vector3d t_this = xc.p;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_this.x(), t_this.y(), t_this.z()));
    q.setW(q_this.w());
    q.setX(q_this.x());
    q.setY(q_this.y());
    q.setZ(q_this.z());
    transform.setRotation(q);
    ros::Time ct = ros::Time::fromSec(xc.t);
    br.sendTransform(
        tf::StampedTransform(transform, ct, odom_frame, child_frame));
  }

  void pub_localtraj(PLV(3) &pwld, double jour, IMUST &x_curr,
                     int cur_session, pcl::PointCloud<PointType> &pcl_path,
                     bool publish_scan_tf, const string &odom_frame,
                     const string &child_frame)
  {
    pub_odom_func(x_curr, publish_scan_tf, odom_frame, child_frame);
    pcl::PointCloud<PointType> pcl_send;
    pcl_send.reserve(pwld.size());
    for(Eigen::Vector3d &pw: pwld)
    {
      Eigen::Vector3d pvec = pw;
      PointType ap;
      ap.x = pvec.x();
      ap.y = pvec.y();
      ap.z = pvec.z();
      pcl_send.push_back(ap);
    }
    pub_pl_func(pcl_send, pub_scan, odom_frame, x_curr.t);
    
    Eigen::Vector3d pcurr = x_curr.p;

    PointType ap;
    ap.x = pcurr[0];
    ap.y = pcurr[1];
    ap.z = pcurr[2];
    ap.curvature = jour;
    ap.intensity = cur_session;
    pcl_path.push_back(ap);
    pub_pl_func(pcl_path, pub_curr_path, odom_frame, x_curr.t);
  }

  void pub_localmap(int mgsize, int cur_session, vector<PVecPtr> &pvec_buf, vector<IMUST> &x_buf, pcl::PointCloud<PointType> &pcl_path, int win_base, int win_count, const string &frame = "camera_init")
  {
    pcl::PointCloud<PointType> pcl_send;
    for(int i=0; i<mgsize; i++)
    {
      for(int j=0; j<pvec_buf[i]->size(); j+=3)
      {
        pointVar &pv = pvec_buf[i]->at(j);
        Eigen::Vector3d pvec = x_buf[i].R*pv.pnt + x_buf[i].p;
        PointType ap;
        ap.x = pvec[0];
        ap.y = pvec[1];
        ap.z = pvec[2];
        ap.intensity = cur_session;
        pcl_send.push_back(ap);
      }
    }

    for(int i=0; i<win_count; i++)
    {
      Eigen::Vector3d pcurr = x_buf[i].p;
      pcl_path[i+win_base].x = pcurr[0];
      pcl_path[i+win_base].y = pcurr[1];
      pcl_path[i+win_base].z = pcurr[2];
    }

    pub_pl_func(pcl_path, pub_curr_path, frame, x_buf[win_count-1].t);
    pub_pl_func(pcl_send, pub_cmap, frame, x_buf[win_count-1].t);
  }

  void pub_global_path(vector<vector<ScanPose*>*> &relc_bl_buf, ros::Publisher &pub_relc, vector<int> &ids)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pcl::PointXYZI pp;
    int idsize = ids.size();

    for(int i=0; i<idsize; i++)
    {
      pp.intensity = ids[i];
      for(ScanPose* bl: *(relc_bl_buf[ids[i]]))
      {
        pp.x = bl->x.p[0]; pp.y = bl->x.p[1]; pp.z = bl->x.p[2];
        pl.push_back(pp);
      }
    }
    pub_pl_func(pl, pub_relc);
  }

  void pub_globalmap(vector<vector<Keyframe*>*> &relc_submaps, vector<int> &ids, ros::Publisher &pub)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pub_pl_func(pl, pub);
    pcl::PointXYZI pp;

    uint interval_size = 5e6;
    uint psize = 0;
    for(int id: ids)
    {
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
        psize += smps[i]->plptr->size();
    }
    int jump = psize / (10 * interval_size) + 1;

    for(int id: ids)
    {
      pp.intensity = id;
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
      {
        IMUST xx = smps[i]->x0;
        for(int j=0; j<smps[i]->plptr->size(); j+=jump)
        // for(int j=0; j<smps[i]->plptr->size(); j+=1)
        {
          PointType &ap = smps[i]->plptr->points[j];
          Eigen::Vector3d vv(ap.x, ap.y, ap.z);
          vv = xx.R * vv + xx.p;
          pp.x = vv[0]; pp.y = vv[1]; pp.z = vv[2];
          pl.push_back(pp);
        }

        if(pl.size() > interval_size)
        {
          pub_pl_func(pl, pub);
          sleep(0.05);
          pl.clear();
        }
      }
    }
    pub_pl_func(pl, pub);
  }

};

class FileReaderWriter
{
public:
  static FileReaderWriter &instance()
  {
    static FileReaderWriter inst;
    return inst;
  }

  void save_pcd(PVecPtr pptr, IMUST &xx, int count, const string &savename)
  {
    pcl::PointCloud<pcl::PointXYZI> pl_save;
    for(pointVar &pw: *pptr)
    {
      pcl::PointXYZI ap;
      ap.x = pw.pnt[0]; ap.y = pw.pnt[1]; ap.z = pw.pnt[2];
      pl_save.push_back(ap);
    }
    string pcdname = savename + "/" + to_string(count) + ".pcd";
    pcl::io::savePCDFileBinary(pcdname, pl_save); 
  }

  void save_pose(vector<ScanPose*> &bbuf, string &fname, string posename, string &savepath)
  {
    if(bbuf.size() < 100) return;
    int topsize = bbuf.size();

    ofstream posfile(savepath + fname + posename);
    for(int i=0; i<topsize; i++)
    {
      IMUST &xx = bbuf[i]->x;
      Eigen::Quaterniond qq(xx.R);
      posfile << fixed << setprecision(6) << xx.t << " ";
      posfile << setprecision(7) << xx.p[0] << " " << xx.p[1] << " " << xx.p[2] << " ";
      posfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w();
      posfile << " " << xx.v[0] << " " << xx.v[1] << " " << xx.v[2];
      posfile << " " << xx.bg[0] << " " << xx.bg[1] << " " << xx.bg[2];
      posfile << " " << xx.ba[0] << " " << xx.ba[1] << " " << xx.ba[2];
      posfile << " " << xx.g[0] << " " << xx.g[1] << " " << xx.g[2];
      for(int j=0; j<6; j++) posfile << " " << bbuf[i]->v6[j];
      posfile << endl;
    }
    posfile.close();

  }

  // The loop clousure edges of multi sessions
  void pgo_edges_io(PGO_Edges &edges, vector<string> &fnames, int io, string &savepath, string &bagname)
  {
    static vector<string> seq_absent;
    Eigen::Matrix<double, 6, 1> v6_init;
    v6_init << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    if(io == 0) // read
    {
      ifstream infile(savepath + "edge.txt");
      string lineStr, str;
      vector<string> sts;
      while(getline(infile, lineStr))
      {
        sts.clear();
        stringstream ss(lineStr);
        while(ss >> str)
          sts.push_back(str);
        
        int mp[2] = {-1, -1};
        for(int i=0; i<2; i++)
        for(int j=0; j<fnames.size(); j++)
        if(sts[i] == fnames[j])
        {
          mp[i] = j;
          break;
        }

        if(mp[0] != -1 && mp[1] != -1)
        {
          int id1 = stoi(sts[2]);
          int id2 = stoi(sts[3]);
          Eigen::Vector3d v3; 
          v3 << stod(sts[4]), stod(sts[5]), stod(sts[6]);
          Eigen::Quaterniond qq(stod(sts[10]), stod(sts[7]), stod(sts[8]), stod(sts[9]));
          Eigen::Matrix3d rot(qq.matrix());
          if(mp[0] <= mp[1])
            edges.push(mp[0], mp[1], id1, id2, rot, v3, v6_init);
          else
          {
            v3 = -rot.transpose() * v3;
            rot = qq.matrix().transpose();
            edges.push(mp[1], mp[0], id2, id1, rot, v3, v6_init);
          }
        }
        else
        {
          if(sts[0] != bagname && sts[1] != bagname)
            seq_absent.push_back(lineStr);
        }

      }
    }
    else // write
    {
      ofstream outfile(savepath + "edge.txt");
      for(string &str: seq_absent)
        outfile << str << endl;

      for(PGO_Edge &edge: edges.edges)
      {
        for(int i=0; i<edge.rots.size(); i++)
        {
          outfile << fnames[edge.m1] << " ";
          outfile << fnames[edge.m2] << " ";
          outfile << edge.ids1[i] << " ";
          outfile << edge.ids2[i] << " ";
          Eigen::Vector3d v(edge.tras[i]);
          outfile << setprecision(7) << v[0] << " " << v[1] << " " << v[2] << " ";
          Eigen::Quaterniond qq(edge.rots[i]);
          outfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w() << endl;
        }
      }
      outfile.close();
    }

  }

  // loading the offline map
  void previous_map_names(ros::NodeHandle &n, vector<string> &fnames, vector<double> &juds)
  {
    string premap;
    n.param<string>("General/previous_map", premap, "");
    premap.erase(remove_if(premap.begin(), premap.end(), ::isspace), premap.end());
    stringstream ss(premap);
    string str;
    while(getline(ss, str, ','))
    {
      stringstream ss2(str);
      vector<string> strs;
      while(getline(ss2, str, ':'))
        strs.push_back(str);
      
      if(strs.size() != 2)
      {
        printf("previous map name wrong\n");
        return;
      }

      if(strs[0][0] != '#')
      {
        fnames.push_back(strs[0]);
        juds.push_back(stod(strs[1]));
      }
    }

  }

  void previous_map_read(vector<STDescManager*> &std_managers, vector<vector<ScanPose*>*> &multimap_scanPoses, vector<vector<Keyframe*>*> &multimap_keyframes, ConfigSetting &config_setting, PGO_Edges &edges, ros::NodeHandle &n, vector<string> &fnames, vector<double> &juds, string &savepath, int win_size)
  {
    int acsize = 10; int mgsize = 5;
    n.param<int>("Loop/acsize", acsize, 10);
    n.param<int>("Loop/mgsize", mgsize, 5);

    for(int fn=0; fn<fnames.size() && n.ok(); fn++)
    {
      string fname = savepath + fnames[fn];
      vector<ScanPose*>* bl_tem = new vector<ScanPose*>();
      vector<Keyframe*>* keyframes_tem = new vector<Keyframe*>();
      STDescManager *std_manager = new STDescManager(config_setting);

      std_managers.push_back(std_manager);
      multimap_scanPoses.push_back(bl_tem);
      multimap_keyframes.push_back(keyframes_tem);
      read_lidarstate(fname+"/alidarState.txt", *bl_tem);

      cout << "Reading " << fname << ": " << bl_tem->size() << " scans." << "\n";
      deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> plbuf;
      deque<IMUST> xxbuf;
      pcl::PointCloud<PointType> pl_lc;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_btc(new pcl::PointCloud<pcl::PointXYZI>());

      for(int i=0; i<bl_tem->size() && n.ok(); i++)
      {
        IMUST &xc = bl_tem->at(i)->x;
        string pcdname = fname + "/" + to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr pl_tem(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(pcdname, *pl_tem);

        xxbuf.push_back(xc);
        plbuf.push_back(pl_tem);
        
        if(xxbuf.size() < win_size)
          continue;
        
        pl_lc.clear();
        Keyframe *smp = new Keyframe(xc);
        smp->id = i;
        PointType pt;
        for(int j=0; j<win_size; j++)
        {
          Eigen::Vector3d delta_p = xc.R.transpose() * (xxbuf[j].p - xc.p);
          Eigen::Matrix3d delta_R = xc.R.transpose() *  xxbuf[j].R;

          for(pcl::PointXYZI pp: plbuf[j]->points)
          {
            Eigen::Vector3d v3(pp.x, pp.y, pp.z);
            v3 = delta_R * v3 + delta_p;
            pt.x = v3[0]; pt.y = v3[1]; pt.z = v3[2];
            pl_lc.push_back(pt);
          }
        }

        down_sampling_voxel(pl_lc, voxel_size/10);
        smp->plptr->reserve(pl_lc.size());
        for(PointType &pp: pl_lc.points)
          smp->plptr->push_back(pp);
        keyframes_tem->push_back(smp);
        
        for(int j=0; j<win_size; j++)
        {
          plbuf.pop_front(); xxbuf.pop_front();
        }
      }
      
      cout << "Generating BTC descriptors..." << "\n";

      int subsize = keyframes_tem->size();
      for(int i=0; i+acsize<subsize && n.ok(); i+=mgsize)
      {
        int up = i + acsize;
        pl_btc->clear();
        IMUST &xc = keyframes_tem->at(up - 1)->x0;
        for(int j=i; j<up; j++)
        {
          IMUST &xj = keyframes_tem->at(j)->x0;
          Eigen::Vector3d delta_p = xc.R.transpose() * (xj.p - xc.p);
          Eigen::Matrix3d delta_R = xc.R.transpose() *  xj.R;
          pcl::PointXYZI pp;
          for(PointType ap: keyframes_tem->at(j)->plptr->points)
          {
            Eigen::Vector3d v3(ap.x, ap.y, ap.z);
            v3 = delta_R * v3 + delta_p;
            pp.x = v3[0]; pp.y = v3[1]; pp.z = v3[2];
            pl_btc->push_back(pp);
          }
        }

        vector<STD> stds_vec;
        std_manager->GenerateSTDescs(pl_btc, stds_vec, keyframes_tem->at(up-1)->id);
        std_manager->AddSTDescs(stds_vec);
      }
      std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);

      cout << "Read " << fname << " done." << "\n\n";
    }

    vector<int> ids_all;
    for(int fn=0; fn<fnames.size() && n.ok(); fn++)
      ids_all.push_back(fn);

    // gtsam::Values initial;
    // gtsam::NonlinearFactorGraph graph;
    // vector<int> ids_cnct, stepsizes;
    // Eigen::Matrix<double, 6, 1> v6_init;
    // v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    // gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    // build_graph(initial, graph, ids_all.back(), edges, odom_noise, ids_cnct, stepsizes, 1);

    // gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    // gtsam::ISAM2 isam(parameters);
    // isam.update(graph, initial);

    // for(int i=0; i<5; i++) isam.update();
    // gtsam::Values results = isam.calculateEstimate();
    // int resultsize = results.size();
    // int idsize = ids_cnct.size();
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
    //   {
    //     int ord = j - stepsizes[ii];
    //     multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
    //   }
    // }
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(Keyframe *kf: *multimap_keyframes[tip])
    //     kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
    // }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids_all);
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids_all, pub_pmap);

    printf("All the maps are loaded\n");
  }
  
};

class Initialization
{
public:
  static Initialization &instance()
  {
    static Initialization inst;
    return inst;
  }

  void align_gravity(vector<IMUST> &xs)
  {
    Eigen::Vector3d g0 = xs[0].g;
    Eigen::Vector3d n0 = g0 / g0.norm();
    Eigen::Vector3d n1(0, 0, 1);
    if(n0[2] < 0)
      n1[2] = -1;
    
    Eigen::Vector3d rotvec = n0.cross(n1);
    double rnorm = rotvec.norm();
    rotvec = rotvec / rnorm;

    Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
    Eigen::Matrix3d rot = angaxis.matrix();
    g0 = rot * g0;

    Eigen::Vector3d p0 = xs[0].p;
    for(int i=0; i<xs.size(); i++)
    {
      xs[i].p = rot * (xs[i].p - p0) + p0;
      xs[i].R = rot * xs[i].R;
      xs[i].v = rot * xs[i].v;
      xs[i].g = g0;
    }

  }

  void motion_blur(pcl::PointCloud<PointType> &pl, PVec &pvec, IMUST xc, IMUST xl, deque<sensor_msgs::msg::Imu::SharedPtr> &imus, double pcl_beg_time, IMUST &extrin_para)
  {
    xc.bg = xl.bg; xc.ba = xl.ba;
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
    Eigen::Matrix3d R_imu(xc.R);
    vector<IMUST> imu_poses;

    for(auto it_imu=imus.end()-1; it_imu!=imus.begin(); it_imu--)
    {
      sensor_msgs::msg::Imu &head = **(it_imu-1);
      sensor_msgs::msg::Imu &tail = **(it_imu);
      
      angvel_avr << 0.5*(head.angular_velocity.x + tail.angular_velocity.x), 
                    0.5*(head.angular_velocity.y + tail.angular_velocity.y), 
                    0.5*(head.angular_velocity.z + tail.angular_velocity.z);
      acc_avr << 0.5*(head.linear_acceleration.x + tail.linear_acceleration.x), 
                 0.5*(head.linear_acceleration.y + tail.linear_acceleration.y), 
                 0.5*(head.linear_acceleration.z + tail.linear_acceleration.z);

      angvel_avr -= xc.bg;
      acc_avr = acc_avr * imupre_scale_gravity - xc.ba;

      double dt = rclcpp::Time(head.header.stamp).seconds() -
                  rclcpp::Time(tail.header.stamp).seconds();
      Eigen::Matrix3d acc_avr_skew = hat(acc_avr);
      Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);

      acc_imu = R_imu * acc_avr + xc.g;
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      vel_imu = vel_imu + acc_imu * dt;
      R_imu = R_imu * Exp_f;

      double offt = rclcpp::Time(head.header.stamp).seconds() - pcl_beg_time;
      imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
    }

    pointVar pv; pv.var.setIdentity();
    if(point_notime)
    {
      for(PointType &ap: pl.points)
      {
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;
        pvec.push_back(pv);
      }
      return;
    }
    auto it_pcl = pl.end() - 1;
    // for(auto it_kp=imu_poses.end(); it_kp!=imu_poses.begin(); it_kp--)
    for(auto it_kp=imu_poses.begin(); it_kp!=imu_poses.end(); it_kp++)
    {
      // IMUST &head = *(it_kp - 1);
      IMUST &head = *it_kp;
      R_imu = head.R;
      acc_imu = head.ba;
      vel_imu = head.v;
      pos_imu = head.p;
      angvel_avr = head.bg;

      for(; it_pcl->curvature > head.t; it_pcl--)
      {
        double dt = it_pcl->curvature - head.t;
        Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
        Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;

        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

        pv.pnt = P_compensate;
        pvec.push_back(pv);
        if(it_pcl == pl.begin()) break;
      }

    }
  }

  int motion_init(vector<pcl::PointCloud<PointType>::Ptr> &pl_origs, vector<deque<sensor_msgs::msg::Imu::SharedPtr>> &vec_imus, vector<double> &beg_times, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<IMUST> &x_buf, unordered_map<VOXEL_LOC, OctoTree*> &surf_map, unordered_map<VOXEL_LOC, OctoTree*> &surf_map_slide, vector<PVecPtr> &pvec_buf, int win_size, vector<vector<SlideWindow*>> &sws, IMUST &x_curr, deque<IMU_PRE*> &imu_pre_buf, IMUST &extrin_para)
  {
    PLV(3) pwld;
    double last_g_norm = x_buf[0].g.norm();
    int converge_flag = 0;

    double min_eigen_value_orig = min_eigen_value;
    vector<double> eigen_value_array_orig = plane_eigen_value_thre;

    min_eigen_value = 0.02;
    for(double &iter: plane_eigen_value_thre)
      iter = 1.0 / 4;

    double t0 = ros::Time::now().toSec();
    double converge_thre = 0.05;
    int converge_times = 0;
    bool is_degrade = true;
    Eigen::Vector3d eigvalue; eigvalue.setZero();
    for(int iterCnt = 0; iterCnt < 10; iterCnt++)
    {
      if(converge_flag == 1)
      {
        min_eigen_value = min_eigen_value_orig;
        plane_eigen_value_thre = eigen_value_array_orig;
      }

      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();

      for(int i=0; i<win_size; i++)
      {
        pwld.clear();
        pvec_buf[i]->clear();
        int l = i==0 ? i : i - 1;
        motion_blur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

        if(converge_flag == 1)
        {
          for(pointVar &pv: *pvec_buf[i])
            calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
          pvec_update(pvec_buf[i], x_buf[i], pwld);
        }
        else
        {
          for(pointVar &pv: *pvec_buf[i])
            pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }

        cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
      }

      // LidarFactor voxhess(win_size);
      voxhess.clear(); voxhess.win_size = win_size;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->recut(win_size, x_buf, sws[0]);
        iter->second->tras_opt(voxhess);
      }

      if(voxhess.plvec_voxels.size() < 10)
        break;
      LI_BA_OptimizerGravity opt_lsv;
      vector<double> resis;
      opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);
      Eigen::Matrix3d nnt; nnt.setZero();

      printf("%d: %lf %lf %lf: %lf %lf\n", iterCnt, x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm(), fabs(resis[0] - resis[1]) / resis[0]);

      for(int i=0; i<win_size-1; i++)
        delete imu_pre_buf[i];
      imu_pre_buf.clear();

      for(int i=1; i<win_size; i++)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[i-1].bg, x_buf[i-1].ba));
        imu_pre_buf.back()->push_imu(vec_imus[i]);
      }

      if(fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
      {
        for(Eigen::Matrix3d &iter: voxhess.eig_vectors)
        {
          Eigen::Vector3d v3 = iter.col(0);
          nnt += v3 * v3.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
        eigvalue = saes.eigenvalues();
        is_degrade = eigvalue[0] < 15 ? true : false;

        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          align_gravity(x_buf);
          converge_flag = 1;
          continue;
        }
        else
          break;
      }
    }

    x_curr = x_buf[win_size - 1];
    double gnm = x_curr.g.norm();
    if(is_degrade || gnm < 9.6 || gnm > 10.0)
    {
      converge_flag = 0;
    }
    if(converge_flag == 0)
    {
      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();
    }

    printf("mn: %lf %lf %lf\n", eigvalue[0], eigvalue[1], eigvalue[2]);
    Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y, vec_imus[0][0]->angular_velocity.z);
    Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y, vec_imus[0][0]->linear_acceleration.z);
    acc *= 9.8;

    pl_origs.clear(); vec_imus.clear(); beg_times.clear();
    double t1 = ros::Time::now().toSec();
    printf("init time: %lf\n", t1 - t0);

    // align_gravity(x_buf);
    pcl::PointCloud<PointType> pcl_send; PointType pt;
    for(int i=0; i<win_size; i++)
    for(pointVar &pv: *pvec_buf[i])
    {
      Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
      pt.x = vv[0]; pt.y = vv[1]; pt.z = vv[2];
      pcl_send.push_back(pt);
    }
    pub_pl_func(pcl_send, pub_init);

    return converge_flag;
  }

};

class VOXEL_SLAM
{
public:
  pcl::PointCloud<PointType> pcl_path;
  IMUST x_curr, extrin_para;
  IMUEKF odom_ekf;
  unordered_map<VOXEL_LOC, OctoTree*> surf_map, surf_map_slide;
  double down_size;

  int win_size;
  vector<IMUST> x_buf;
  vector<PVecPtr> pvec_buf;
  deque<IMU_PRE*> imu_pre_buf;
  int win_count = 0, win_base = 0;
  vector<vector<SlideWindow*>> sws;

  vector<ScanPose*> *scanPoses;
  mutex mtx_loop;
  deque<ScanPose*> buf_lba2loop, buf_lba2loop_tem;
  vector<Keyframe*> *keyframes;
  int loop_detect = 0;
  unordered_map<VOXEL_LOC, OctoTree*> map_loop;
  IMUST dx;
  pcl::PointCloud<PointType>::Ptr pl_kdmap;
  pcl::KdTreeFLANN<PointType> kd_keyframes;
  int history_kfsize = 0;
  vector<OctoTree*> octos_release;
  std::atomic<int> reset_flag{0};
  bool has_global_correction_generation = false;
  std::uint64_t global_correction_generation = 0;
  int g_update = 0;
  int thread_num = 5;
  int degrade_bound = 10;

  vector<vector<ScanPose*>*> multimap_scanPoses;
  vector<vector<Keyframe*>*> multimap_keyframes;
  volatile int gba_flag = 0;
  int gba_size = 0;
  vector<int> cnct_map;
  mutex mtx_keyframe;
  PGO_Edges gba_edges1, gba_edges2;
  std::atomic<bool> is_finish{false};
  std::atomic<bool> finish_without_ba{false};
  std::atomic<bool> stop_gba_worker{false}, gba_worker_stopped{false};

  vector<string> sessionNames;
  string bagname, savepath;
  string operation_mode, map_export_path, localization_map_path;
  bool map_export_btc_cache = false;
  bool map_export_spatial_blocks = false;
  bool dual_tf = false;
  bool local_ba_enabled = true;
  string map_frame = "map";
  DualTfOptions dual_options;
  DirectMatchOptions direct_options;
  string relocalization_backend = "btc";
  ScanContextOptions scancontext_options;
  DualTfAlignment dual_alignment;
  string initialpose_base_frame;
  std::unique_ptr<tf2_ros::Buffer> initialpose_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> initialpose_tf_listener;
  struct DualScan { std::uint64_t epoch; IMUST state; PVecPtr points; };
  deque<DualScan> dual_scans;
  deque<DualScan> dual_latest_scans;
  int dual_queue_capacity = 30;
  std::atomic<double> dual_sensor_time{0.0};
  std::atomic<double> dual_prediction_wall{-1.0};
  std::atomic<bool> dual_initializing{true};
  ros::Publisher pub_global_odom, pub_local_valid, pub_global_age, pub_global_stamp;
  rclcpp::TimerBase::SharedPtr dual_status_timer;
  SavedMap localization_map;
  SavedMap descriptor_history_map;
  std::shared_ptr<ScanMapStore> scan_map_store;
  std::unique_ptr<ScanMapWriter> scan_map_writer;
  double map_scan_leaf = 0.1;
  int map_cache_mb = 512, history_scans = 5;
  double history_distance = 0.5, history_angle = 10.0;
  vector<pcl::PointCloud<PointType>::Ptr> detail_window, initialization_details;
  unordered_map<double, int> scan_export_ids;
  vector<pair<int, ScanPose*>> scan_pose_links;
  bool scan_export_failed = false;

  static SavedPose saved_pose(const IMUST &x) {
    SavedPose p; p.stamp=x.t; p.R=x.R; p.t=x.p; return p;
  }
  bool needs_details() const { return dual_tf || (operation_mode=="mapping" && !map_export_path.empty()); }
  pcl::PointCloud<PointType>::Ptr imu_cloud(const pcl::PointCloud<PointType> &cloud) {
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    output->reserve(cloud.size());
    for(const auto &p : cloud) {
      const Eigen::Vector3d v=extrin_para.R*p.getVector3fMap().cast<double>()+extrin_para.p;
      if(!v.allFinite()) continue;
      PointType q{}; q.x=v.x(); q.y=v.y(); q.z=v.z(); q.intensity=p.intensity; output->push_back(q);
    }
    return output;
  }
  void export_scan(const pcl::PointCloud<PointType> &cloud, const IMUST &state) {
    if(!scan_map_writer || scan_export_failed) return;
    try {
      const int id=scan_map_writer->append(cloud,saved_pose(state),frontend_session_epoch.capture());
      scan_export_ids.emplace(state.t,id);
    } catch(const std::exception &e) {
      scan_export_failed=true;
      ROS_ERROR_STREAM("Scan map capture failed; no complete map will be published: " << e.what());
    }
  }
  bool localization_publish_map_cloud = false;
  string localization_map_cloud_path;
  ros::Publisher localization_map_cloud_pub;
  SpatialMapConfig localization_spatial_config;
  SpatialMapConfig map_export_spatial_config;
  SpatialMapRuntimeConfig spatial_runtime_config;
  std::shared_ptr<FileSpatialBlockSource> spatial_block_source;
  std::unique_ptr<SpatialMapManager> spatial_map_manager;
  std::unique_ptr<StaticMapMatcher> spatial_map_matcher;
  std::uint64_t spatial_map_generation = 0;
  double local_map_prepare_started_sec = 0.0;
  double map_gap_started_sec = -1.0;
  PriorMapTracker prior_map_tracker;
  std::atomic<bool> prior_map_ready{false};
  std::atomic<bool> localization_valid{false};
  bool imu_rate_output = true;
  bool publish_before_localized = false;
  string imu_odom_topic = "/LIO/odom_imu";
  string odom_frame = "camera_init";
  string imu_child_frame = "aft_mapped";
  double max_prediction_sec = 0.5;
  ImuRatePredictor imu_rate_predictor;
  std::atomic<bool> imu_prediction_ready{false};
  mutex imu_output_mutex;
  ros::CallbackQueue imu_callback_queue;
  ros::Subscriber sub_imu;
  ros::Publisher pub_imu_odom;
  tf::TransformBroadcaster imu_tf_broadcaster;
  std::unique_ptr<ros::AsyncSpinner> imu_spinner;
  mutex frontend_pipeline_mutex;
  mutex localization_state_mutex;
  LocalizationStateMachine localization_machine;
  FrontendSessionEpoch frontend_session_epoch;
  ros::Publisher pub_localization_state;
  ros::Publisher pub_localization_valid;
  int lost_bad_scan_limit = 5;
  int tracking_min_matched_points = 50;
  double tracking_max_mean_abs_point_plane = 0.5;
  double tracking_min_normal_eigenvalue = 14.0;
  TrackingQuality last_tracking_quality;
  ros::Subscriber sub_initialpose;
  InitialPoseRequestBuffer initial_pose_requests;
  bool rviz_initialpose_enabled = true;
  bool imu_orientation_ready = false;
  bool direct_validation_active = false;
  std::uint64_t active_direct_generation = 0;
  std::uint64_t handled_direct_generation = 0;
  int direct_validation_streak = 0;
  int direct_pose_validation_scans = 3;
  double direct_pose_validation_timeout_sec = 5.0;
  double direct_validation_started_sec = 0.0;
  bool use_prior_pose_for_btc = true;
  Eigen::Vector2d btc_prior_xy = Eigen::Vector2d::Zero();
  double btc_search_radius = 30.0;
  bool btc_expand_on_failure = true;
  double btc_max_search_radius = 100.0;
  bool btc_fallback_to_global = true;
  double alignment_max_translation_m = 1000000.0;
  double alignment_max_rotation_deg = 180.0;
  std::atomic<bool> frontend_reset_requested{false};
  bool lost_recovery_pending = false;
  vector<pcl::PointCloud<PointType>::Ptr> initialization_clouds;
  vector<double> initialization_times;
  vector<deque<sensor_msgs::msg::Imu::SharedPtr>> initialization_imus;
  int is_save_map;

  VOXEL_SLAM(ros::NodeHandle &n)
  {
    double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
    double btc_prior_x = 0.0, btc_prior_y = 0.0;
    vector<double> vecR(9), vecT(3);
    scanPoses = new vector<ScanPose*>();
    keyframes = new vector<Keyframe*>();
    
    string lid_topic, imu_topic;
    n.param<string>("General/lid_topic", lid_topic, "/livox/lidar");
    n.param<string>("General/imu_topic", imu_topic, "/livox/imu");
    n.param<string>("General/bagname", bagname, "site3_handheld_4");
    n.param<string>("General/save_path", savepath, "");
    n.param<string>("General/mode", operation_mode, "mapping");
    bool finish_no_ba_default = false;
    n.param<bool>("finish_no_ba", finish_no_ba_default, false);
    n.param<string>("General/map_export_path", map_export_path, "");
    n.param<bool>("General/map_export_btc_cache", map_export_btc_cache, false);
    n.param<bool>("General/map_export_spatial_blocks", map_export_spatial_blocks, false);
    n.param<bool>("Localization/dual_tf/enabled", dual_tf, operation_mode == "localization");
    dual_tf = dual_tf && operation_mode == "localization";
    n.param<string>("Localization/dual_tf/map_frame", map_frame, "map");
#define DUAL_PARAM(field) n.param("Localization/dual_tf/" #field, dual_options.field, dual_options.field)
    DUAL_PARAM(query_period_sec);
    DUAL_PARAM(max_result_age_sec);
    DUAL_PARAM(stale_after_sec);
    DUAL_PARAM(global_valid_timeout_sec);
    DUAL_PARAM(large_translation_m);
    DUAL_PARAM(large_rotation_deg);
    DUAL_PARAM(confirmation_translation_m);
    DUAL_PARAM(confirmation_rotation_deg);
    DUAL_PARAM(confirmation_timeout_sec);
    DUAL_PARAM(audit_min_delay_sec);
    DUAL_PARAM(audit_scans);
    DUAL_PARAM(audit_min_matches);
    DUAL_PARAM(audit_min_overlap);
    DUAL_PARAM(audit_max_plane_distance_m);
    DUAL_PARAM(audit_max_point_distance_m);
    DUAL_PARAM(audit_max_rmse_m);
    DUAL_PARAM(audit_max_p95_m);
    DUAL_PARAM(audit_min_normal_eigenvalue);
    DUAL_PARAM(audit_min_spatial_eigenvalue_m2);
    DUAL_PARAM(audit_prior_comparison_m);
    DUAL_PARAM(audit_improvement_margin_m);
#undef DUAL_PARAM
    n.param("Localization/scancontext/correction_jump_check_enabled",
            dual_options.sc_correction_jump_check_enabled, dual_options.sc_correction_jump_check_enabled);
    n.param("Localization/scancontext/max_correction_translation_m",
            dual_options.sc_max_correction_translation_m, dual_options.sc_max_correction_translation_m);
    n.param("Localization/scancontext/max_correction_rotation_deg",
            dual_options.sc_max_correction_rotation_deg, dual_options.sc_max_correction_rotation_deg);
    dual_alignment.configure(dual_options);
    direct_options = read_direct_match_options(n);
    relocalization_backend = "scancontext_quatro_gicp";
    scancontext_options = read_scancontext_options(n);
    if(operation_mode == "localization" && relocalization_backend != "btc" &&
       (!dual_tf || !scancontext_backend_available()))
      throw std::invalid_argument("Scan Context/Quatro requires dual TF and a TEASER++ enabled build");
    dual_alignment.reset(frontend_session_epoch.capture());
    dual_queue_capacity = 30;
    if(dual_queue_capacity <= 0) throw std::invalid_argument("invalid BTC queue capacity");
    n.param<string>("Localization/map_path", localization_map_path, "");
    n.param<int>("Localization/map_cache_mb", map_cache_mb, 512);
    n.param<int>("Localization/history/window_scans", history_scans, 5);
    n.param<double>("Localization/history/translation_m", history_distance, 0.5);
    n.param<double>("Localization/history/rotation_deg", history_angle, 10.0);
    n.param<double>("General/map_scan_leaf_size_m", map_scan_leaf, 0.1);
    if(map_cache_mb<1 || map_cache_mb>65536) throw std::invalid_argument("invalid map cache size");
    n.param<bool>("Localization/visualization/publish_map_cloud",
                  localization_publish_map_cloud, false);
    n.param<string>("Localization/visualization/map_cloud_path",
                    localization_map_cloud_path, "");
    n.param<double>("Localization/spatial_map/preload_radius",
                    spatial_runtime_config.preload_radius, 15.0);
    n.param<double>("Localization/spatial_map/load_radius",
                    spatial_runtime_config.load_radius, 30.0);
    n.param<double>("Localization/spatial_map/unload_radius",
                    spatial_runtime_config.unload_radius, 40.0);
    n.param<double>("Localization/spatial_map/prefetch_time_sec",
                    spatial_runtime_config.prefetch_time_sec, 2.0);
    n.param<double>("Localization/spatial_map/map_gap_timeout_sec",
                    spatial_runtime_config.map_gap_timeout_sec, 1.0);
    n.param<double>("Localization/spatial_map/preload_timeout_sec",
                    spatial_runtime_config.preload_timeout_sec, 10.0);
    n.param<int>("Localization/spatial_map/max_background_loads",
                 spatial_runtime_config.max_background_loads, 2);
    n.param<double>("Localization/spatial_map/max_block_memory_mb",
                    spatial_runtime_config.max_block_memory_mb, 0.0);
    n.param<double>("Localization/spatial_map/io_rate_mb_per_sec",
                    spatial_runtime_config.io_rate_mb_per_sec, 0.0);
    n.param<double>("Localization/spatial_map/io_fixed_latency_ms",
                    spatial_runtime_config.io_fixed_latency_ms, 0.0);
    n.param<string>("Localization/spatial_map/metrics_directory",
                   spatial_runtime_config.metrics_directory, "");
    n.param<double>("MapBlock/size_x", map_export_spatial_config.size_x,
                    20.0);
    n.param<double>("MapBlock/size_y", map_export_spatial_config.size_y,
                    20.0);
    n.param<double>("MapBlock/size_z", map_export_spatial_config.size_z,
                    10.0);
    n.param<double>("MapBlock/fusion_leaf_size",
                    map_export_spatial_config.fusion_leaf_size, 0.1);
    n.param<double>("MapBlock/root_voxel_size",
                    map_export_spatial_config.root_voxel_size, 1.0);
    n.param<int>("MapBlock/max_layer",
                 map_export_spatial_config.max_layer, 2);
    n.param<double>("MapBlock/min_eigen_value",
                    map_export_spatial_config.min_eigen_value, 0.0025);
    n.param<bool>("MapBlock/save_display_cloud",
                  map_export_spatial_config.save_display_cloud, false);
    n.param<bool>("Localization/rviz_initialpose_enabled",
                  rviz_initialpose_enabled, true);
    n.param<int>("Localization/direct_pose_validation_scans",
                 direct_pose_validation_scans, 3);
    n.param<double>("Localization/direct_pose_validation_timeout_sec",
                    direct_pose_validation_timeout_sec, 5.0);
    n.param<int>("Localization/lost_bad_scan_limit",
                 lost_bad_scan_limit, 5);
    n.param<int>("Localization/tracking_min_matched_points",
                 tracking_min_matched_points, 50);
    n.param<double>("Localization/tracking_max_mean_abs_point_plane",
                    tracking_max_mean_abs_point_plane, 0.5);
    n.param<double>("Localization/tracking_min_normal_eigenvalue",
                    tracking_min_normal_eigenvalue, 14.0);
    n.param<bool>("Localization/use_prior_pose_for_btc",
                  use_prior_pose_for_btc, true);
    n.param<double>("Localization/btc_prior_pose/x",
                    btc_prior_x, 0.0);
    n.param<double>("Localization/btc_prior_pose/y",
                    btc_prior_y, 0.0);
    btc_prior_xy << btc_prior_x, btc_prior_y;
    n.param<double>("Localization/btc_search_radius",
                    btc_search_radius, 30.0);
    n.param<bool>("Localization/btc_expand_on_failure",
                  btc_expand_on_failure, true);
    n.param<double>("Localization/btc_max_search_radius",
                    btc_max_search_radius, 100.0);
    n.param<bool>("Localization/btc_fallback_to_global",
                  btc_fallback_to_global, true);
    n.param<double>("Localization/alignment_max_translation_m",
                    alignment_max_translation_m, 1000000.0);
    n.param<double>("Localization/alignment_max_rotation_deg",
                    alignment_max_rotation_deg, 180.0);
    n.param<int>("General/lidar_type", feat.lidar_type, 0);
    n.param<double>("General/blind", feat.blind, 0.1);
    n.param<int>("General/point_filter_num", feat.point_filter_num, 3);
    n.param<vector<double>>("General/extrinsic_tran", vecT, vector<double>());
    n.param<vector<double>>("General/extrinsic_rota", vecR, vector<double>());
    n.param<int>("General/is_save_map", is_save_map, 0);
    if(is_save_map) ROS_WARN("Legacy per-scan output is disabled; use the SC scan-map exporter.");
    is_save_map=0;
    if(operation_mode=="mapping" && !map_export_path.empty())
      scan_map_writer.reset(new ScanMapWriter(map_export_path,map_scan_leaf));
    // Localization never writes legacy mapping outputs.
    if(operation_mode == "localization")
      is_save_map = 0;

    if(operation_mode != "mapping" && operation_mode != "localization")
    {
      ROS_FATAL_STREAM("Unknown General/mode '" << operation_mode
                       << "'; expected 'mapping' or 'localization'.");
      throw std::invalid_argument("unknown General/mode");
    }
    if(operation_mode != "mapping" && !map_export_path.empty())
    {
      ROS_FATAL_STREAM("General/map_export_path is only valid in mapping mode.");
      throw std::invalid_argument(
          "map_export_path is only valid in mapping mode");
    }
    if(operation_mode == "localization" &&
       (direct_pose_validation_scans <= 0 ||
        direct_pose_validation_timeout_sec <= 0.0 ||
        lost_bad_scan_limit <= 0 || btc_search_radius <= 0.0 ||
        tracking_min_matched_points <= 0 ||
        tracking_max_mean_abs_point_plane <= 0.0 ||
        tracking_min_normal_eigenvalue <= 0.0 ||
        btc_max_search_radius < btc_search_radius ||
        alignment_max_translation_m <= 0.0 ||
        alignment_max_rotation_deg <= 0.0 ||
        alignment_max_rotation_deg > 180.0 ||
        !btc_prior_xy.array().isFinite().all() ||
        !std::isfinite(btc_search_radius) ||
        !std::isfinite(btc_max_search_radius) ||
        !std::isfinite(tracking_max_mean_abs_point_plane) ||
        !std::isfinite(tracking_min_normal_eigenvalue) ||
        !std::isfinite(alignment_max_translation_m) ||
        !std::isfinite(alignment_max_rotation_deg)))
    {
      ROS_FATAL("Localization or direct-validation settings are invalid.");
      throw std::invalid_argument("invalid localization settings");
    }
    string spatial_config_error;
    if(!dual_tf && operation_mode == "localization" &&
       !spatial_runtime_config.validate(spatial_config_error))
    {
      ROS_FATAL_STREAM("Invalid Localization/spatial_map settings: "
                       << spatial_config_error);
      throw std::invalid_argument("invalid spatial map runtime settings");
    }
    if(operation_mode == "mapping" && !map_export_path.empty() &&
       !map_export_spatial_config.validate(spatial_config_error))
    {
      ROS_FATAL_STREAM("Invalid MapBlock settings: " << spatial_config_error);
      throw std::invalid_argument("invalid map block settings");
    }

    if(feat.lidar_type == LIVOX) {
#if VOXEL_SLAM_HAS_LIVOX
      sub_pcl = n.subscribe<LivoxCustomMsg>(
          lid_topic, 1000,
          [](LivoxCustomMsg::ConstSharedPtr message) { pcl_handler(message); });
#else
      ROS_WARN("Livox ROS2 driver is unavailable; treating the lidar input as PointCloud2.");
      sub_pcl = n.subscribe<sensor_msgs::msg::PointCloud2>(
          lid_topic, 1000,
          [](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            pcl_handler(message);
          }, nullptr, true);
#endif
    } else {
      sub_pcl = n.subscribe<sensor_msgs::msg::PointCloud2>(
          lid_topic, 1000,
          [](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            pcl_handler(message);
          }, nullptr, true);
    }
    if(operation_mode == "localization" && rviz_initialpose_enabled)
      sub_initialpose = n.subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 5, &VOXEL_SLAM::initialpose_callback, this);
    if(operation_mode == "localization")
    {
      localization_machine = LocalizationStateMachine(lost_bad_scan_limit);
      pub_localization_state = n.advertise<std_msgs::String>(
          "/localization/state", 1, true);
      pub_localization_valid = n.advertise<std_msgs::Bool>(
          "/localization/valid", 1, true);
      publish_localization_status();
    }
    odom_ekf.imu_topic = imu_topic;

    n.param<double>("Odometry/cov_gyr", cov_gyr, 0.1);
    n.param<double>("Odometry/cov_acc", cov_acc, 0.1);
    n.param<double>("Odometry/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("Odometry/rdw_acc", rand_walk_acc, 1e-4);
    n.param<double>("Odometry/down_size", down_size, 0.1);
    double localization_frontend_leaf = -1.0;
    n.param<double>("Localization/frontend_down_size", localization_frontend_leaf, -1.0);
    if(operation_mode == "localization" && localization_frontend_leaf != -1.0) {
      if(!std::isfinite(localization_frontend_leaf) || localization_frontend_leaf <= 0)
        throw std::invalid_argument("Localization.frontend_down_size must be -1 or positive");
      down_size = localization_frontend_leaf;
    }
    n.param<double>("Odometry/dept_err", dept_err, 0.02);
    n.param<double>("Odometry/beam_err", beam_err, 0.05);
    n.param<double>("Odometry/voxel_size", voxel_size, 1);
    n.param<double>("Odometry/min_eigen_value", min_eigen_value, 0.0025);
    n.param<int>("Odometry/degrade_bound", degrade_bound, 10);
    n.param<int>("Odometry/point_notime", point_notime, 0);
    n.param<bool>("Odometry/imu_rate_output", imu_rate_output, true);
    n.param<string>("Odometry/imu_odom_topic", imu_odom_topic,
                    "/LIO/odom_imu");
    n.param<string>("Odometry/odom_frame", odom_frame, dual_tf ? string("odom") : string("camera_init"));
    n.param<string>("Odometry/imu_child_frame", imu_child_frame,
                    "aft_mapped");
    n.param<bool>("Odometry/publish_before_localized",
                  publish_before_localized, false);
    n.param<double>("Odometry/max_prediction_sec", max_prediction_sec, 0.5);
    if(imu_odom_topic.empty() || odom_frame.empty() ||
       imu_child_frame.empty() || !std::isfinite(max_prediction_sec) ||
       max_prediction_sec <= 0.0)
    {
      ROS_FATAL("Invalid Odometry IMU-rate output settings.");
      throw std::invalid_argument("invalid IMU-rate odometry settings");
    }
    odom_ekf.point_notime = point_notime;
    imu_rate_predictor.setEnabled(imu_rate_output);
    if(dual_tf && (!imu_rate_output || map_frame.empty() || map_frame == odom_frame ||
                   imu_child_frame == odom_frame || imu_child_frame == map_frame))
      throw std::invalid_argument("dual TF requires IMU-rate output and distinct map/odom/IMU frames");
    imu_rate_predictor.setMaxPredictionSec(max_prediction_sec);
    pub_imu_odom = n.advertise<nav_msgs::Odometry>(imu_odom_topic, 1000);
    if(dual_tf)
    {
      n.param<string>("Localization/initialpose_base_frame", initialpose_base_frame, "base_link");
      if(rviz_initialpose_enabled) {
        initialpose_tf_buffer.reset(new tf2_ros::Buffer(n.node()->get_clock()));
        initialpose_tf_listener.reset(new tf2_ros::TransformListener(*initialpose_tf_buffer));
      }
      pub_global_odom = n.advertise<nav_msgs::Odometry>("/localization/odometry", 1000);
      pub_local_valid = n.advertise<std_msgs::Bool>("/localization/local_valid", 1, true);
      pub_global_age = n.advertise<std_msgs::msg::Float64>("/localization/correction_age", 1, true);
      pub_global_stamp = n.advertise<std_msgs::msg::Float64>("/localization/last_correction_stamp", 1, true);
      dual_status_timer = n.node()->create_wall_timer(std::chrono::milliseconds(200),
          [this] { publish_dual_status(); });
    }

    feat.blind = feat.blind * feat.blind;
    odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
    odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
    odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
    odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
    odom_ekf.Lid_offset_to_IMU  << vecT[0], vecT[1], vecT[2];
    odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2],
                            vecR[3], vecR[4], vecR[5],
                            vecR[6], vecR[7], vecR[8];                
    extrin_para.R = odom_ekf.Lid_rot_to_IMU;
    extrin_para.p = odom_ekf.Lid_offset_to_IMU;
    min_point << 5, 5, 5, 5;

    n.param<bool>("LocalBA/enabled", local_ba_enabled, true);
    n.param<int>("LocalBA/win_size", win_size, 10);
    n.param<int>("LocalBA/max_layer", max_layer, 2);
    n.param<double>("LocalBA/cov_gyr", cov_gyr, 0.1);
    n.param<double>("LocalBA/cov_acc", cov_acc, 0.1);
    n.param<double>("LocalBA/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("LocalBA/rdw_acc", rand_walk_acc, 1e-4);
    n.param<int>("LocalBA/min_ba_point", min_ba_point, 20);
    n.param<vector<double>>("LocalBA/plane_eigen_value_thre", plane_eigen_value_thre, vector<double>({1, 1, 1, 1}));
    n.param<double>("LocalBA/imu_coef", imu_coef, 1e-4);
    n.param<int>("LocalBA/thread_num", thread_num, 5);

    for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;
    // for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;

    noiseMeas.setZero(); noiseWalk.setZero();
    noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr, 
                            cov_acc, cov_acc, cov_acc;
    noiseWalk.diagonal() << 
    rand_walk_gyr, rand_walk_gyr, rand_walk_gyr, 
    rand_walk_acc, rand_walk_acc, rand_walk_acc;

    if(operation_mode == "mapping")
    {
      int ss = 0;
      if(access((savepath+bagname+"/").c_str(), X_OK) == -1)
      {
        string cmd = "mkdir " + savepath + bagname + "/";
        ss = system(cmd.c_str());
      }
      else
        ss = -1;

      if(ss != 0 && is_save_map == 1)
      {
        printf("The pointcloud will be saved in this run.\n");
        printf("So please clear or rename the existed folder.\n");
        exit(0);
      }
    }

    sws.resize(thread_num);
    cout << "bagname: " << bagname << endl;

    sub_imu = n.subscribe<sensor_msgs::msg::Imu>(
        imu_topic, 80000,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          imu_callback(message);
        }, &imu_callback_queue);
    imu_spinner.reset(new ros::AsyncSpinner(1, &imu_callback_queue));
    imu_spinner->start();
  }

  ~VOXEL_SLAM()
  {
    shutdown_imu_rate_output();
  }

  bool publish_scan_tf() const
  {
    if(dual_tf) return false;
    return !imu_rate_output || !imu_prediction_ready.load();
  }

  void reset_imu_rate_predictor_locked()
  {
    imu_prediction_ready.store(false);
    imu_rate_predictor.reset();
  }

  void reset_imu_rate_predictor()
  {
    lock_guard<mutex> output_lock(imu_output_mutex);
    reset_imu_rate_predictor_locked();
  }

  void shutdown_imu_rate_output()
  {
    if(dual_status_timer) dual_status_timer->cancel();
    sub_imu.shutdown();
    if(imu_spinner) imu_spinner->stop();
    reset_imu_rate_predictor();
  }

  void refresh_imu_rate_predictor_snapshot()
  {
    if(!imu_rate_output) return;
    if(!std::isfinite(imupre_scale_gravity) ||
       !std::isfinite(x_curr.t) || !x_curr.R.array().isFinite().all() ||
       !x_curr.p.array().isFinite().all() ||
       !x_curr.v.array().isFinite().all() ||
       !x_curr.bg.array().isFinite().all() ||
       !x_curr.ba.array().isFinite().all() ||
       !x_curr.g.array().isFinite().all() ||
       !x_curr.cov.array().isFinite().all())
    {
      reset_imu_rate_predictor();
      return;
    }

    lock_guard<mutex> output_lock(imu_output_mutex);
    const bool snapshot_ready =
        imu_rate_predictor.updateSnapshot(x_curr, imupre_scale_gravity);
    imu_prediction_ready.store(snapshot_ready);
  }

  void publish_imu_rate_odometry(const ImuRatePrediction &prediction)
  {
    nav_msgs::Odometry odometry;
    odometry.header.stamp =
        ros::Time::fromSec(prediction.state.t);
    odometry.header.frame_id = odom_frame;
    odometry.child_frame_id = imu_child_frame;

    Eigen::Quaterniond orientation(prediction.state.R);
    orientation.normalize();
    odometry.pose.pose.position.x = prediction.state.p.x();
    odometry.pose.pose.position.y = prediction.state.p.y();
    odometry.pose.pose.position.z = prediction.state.p.z();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = prediction.state.v.x();
    odometry.twist.twist.linear.y = prediction.state.v.y();
    odometry.twist.twist.linear.z = prediction.state.v.z();
    odometry.twist.twist.angular.x =
        prediction.unbiased_angular_velocity.x();
    odometry.twist.twist.angular.y =
        prediction.unbiased_angular_velocity.y();
    odometry.twist.twist.angular.z =
        prediction.unbiased_angular_velocity.z();

    for(int index = 0; index < 36; ++index)
    {
      odometry.pose.covariance[index] = 0.0;
      odometry.twist.covariance[index] = 0.0;
    }
    for(int row = 0; row < 3; ++row)
    {
      for(int column = 0; column < 3; ++column)
      {
        odometry.pose.covariance[row * 6 + column] =
            prediction.state.cov(3 + row, 3 + column);
        odometry.pose.covariance[row * 6 + 3 + column] =
            prediction.state.cov(3 + row, column);
        odometry.pose.covariance[(3 + row) * 6 + column] =
            prediction.state.cov(row, 3 + column);
        odometry.pose.covariance[(3 + row) * 6 + 3 + column] =
            prediction.state.cov(row, column);
      }
      odometry.twist.covariance[row * 6 + row] = -1.0;
      odometry.twist.covariance[(3 + row) * 6 + 3 + row] = -1.0;
    }

    if(dual_tf)
    {
      const Eigen::Vector3d body_velocity = prediction.state.R.transpose() * prediction.state.v;
      odometry.twist.twist.linear.x = body_velocity.x();
      odometry.twist.twist.linear.y = body_velocity.y();
      odometry.twist.twist.linear.z = body_velocity.z();
      const auto parent_covariance = dual_tf_pose_covariance(
          prediction.state.cov.topLeftCorner<6, 6>(), prediction.state.R);
      for(int r=0; r<6; ++r) for(int c=0; c<6; ++c)
        odometry.pose.covariance[r*6+c] = parent_covariance(r,c);
    }
    pub_imu_odom.publish(odometry);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(prediction.state.p.x(),
                                    prediction.state.p.y(),
                                    prediction.state.p.z()));
    tf::Quaternion tf_orientation;
    tf_orientation.setX(orientation.x());
    tf_orientation.setY(orientation.y());
    tf_orientation.setZ(orientation.z());
    tf_orientation.setW(orientation.w());
    transform.setRotation(tf_orientation);
    imu_tf_broadcaster.sendTransform(tf::StampedTransform(
        transform, odometry.header.stamp, odom_frame, imu_child_frame));
    if(dual_tf)
    {
      const auto global = dual_alignment.view(prediction.state.t);
      if(!global.aligned) return;
      const auto &T = global.map_from_odom;
      const Eigen::Quaterniond qr(T.linear());
      tf::Transform correction;
      correction.setOrigin(tf::Vector3(T.translation().x(), T.translation().y(), T.translation().z()));
      correction.setRotation(tf::Quaternion(qr.x(), qr.y(), qr.z(), qr.w()));
      imu_tf_broadcaster.sendTransform(tf::StampedTransform(
          correction, odometry.header.stamp, map_frame, odom_frame));
      const Eigen::Vector3d position = T * prediction.state.p;
      const Eigen::Quaterniond rotation(T.linear() * prediction.state.R);
      odometry.header.frame_id = map_frame;
      odometry.pose.pose.position.x = position.x();
      odometry.pose.pose.position.y = position.y();
      odometry.pose.pose.position.z = position.z();
      odometry.pose.pose.orientation.x = rotation.x();
      odometry.pose.pose.orientation.y = rotation.y();
      odometry.pose.pose.orientation.z = rotation.z();
      odometry.pose.pose.orientation.w = rotation.w();
      Eigen::Matrix<double, 6, 6> covariance, rotation6 = Eigen::Matrix<double, 6, 6>::Zero();
      rotation6.topLeftCorner<3, 3>() = T.linear();
      rotation6.bottomRightCorner<3, 3>() = T.linear();
      for(int r=0; r<6; ++r) for(int c=0; c<6; ++c)
        covariance(r,c) = odometry.pose.covariance[r*6+c];
      covariance = (rotation6 * covariance * rotation6.transpose()).eval();
      for(int r=0; r<6; ++r) for(int c=0; c<6; ++c)
        odometry.pose.covariance[r*6+c] = covariance(r,c);
      pub_global_odom.publish(odometry);
    }
  }

  static double steady_seconds()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_dual_status()
  {
    if(!dual_tf) return;
    const auto global = dual_alignment.view(dual_sensor_time.load());
    const bool local_ok = imu_prediction_ready.load() &&
        steady_seconds() - dual_prediction_wall.load() <= max_prediction_sec;
    std_msgs::Bool local, valid;
    local.data = local_ok;
    valid.data = local_ok && global.aligned && global.age <= dual_options.global_valid_timeout_sec;
    std_msgs::String state;
    state.data = !local_ok ? (dual_initializing.load() ? "INITIALIZING" : "LOCAL_LOST") : !global.aligned ? "LOCAL_ONLY" :
                 global.age > dual_options.global_valid_timeout_sec ? "GLOBAL_UNTRUSTED" :
                 global.stale ? "GLOBAL_STALE" : "GLOBAL_TRACKING";
    std_msgs::msg::Float64 age, stamp;
    age.data = global.age;
    stamp.data = global.last_correction_stamp;
    localization_valid.store(valid.data);
    pub_local_valid.publish(local);
    pub_localization_valid.publish(valid);
    pub_localization_state.publish(state);
    pub_global_age.publish(age);
    pub_global_stamp.publish(stamp);
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
  {
    if(!msg_in) return;
    static int first_message = 1;
    if(first_message)
    {
      first_message = 0;
      printf("Time0: %lf\n", rclcpp::Time(msg_in->header.stamp).seconds());
    }

    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    if(dual_tf && std::isfinite(timestamp)) dual_sensor_time.store(timestamp);
    bool timestamp_rollback = false;
    {
      lock_guard<mutex> lock(mBuf);
      timestamp_rollback =
          imu_last_time >= 0.0 && timestamp < imu_last_time;
      if(timestamp_rollback) imu_buf.clear();
      imu_last_time = timestamp;
      imu_buf.push_back(msg);
    }

    ImuRateSample sample;
    sample.t = timestamp;
    sample.gyro << msg->angular_velocity.x, msg->angular_velocity.y,
                   msg->angular_velocity.z;
    sample.accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
                    msg->linear_acceleration.z;
    lock_guard<mutex> output_lock(imu_output_mutex);
    if(timestamp_rollback)
    {
      ROS_WARN("IMU timestamp rolled back; resetting IMU-rate prediction.");
      reset_imu_rate_predictor_locked();
      if(dual_tf) frontend_reset_requested.store(true);
    }
    const ImuRatePrediction prediction = imu_rate_predictor.pushImu(sample);
    if(!prediction.publishable) return;

    if(!imu_prediction_ready.load()) return;
    if(dual_tf) dual_prediction_wall.store(steady_seconds());
    if(!dual_tf && operation_mode == "localization" && !publish_before_localized &&
       !localization_valid.load())
      return;
    publish_imu_rate_odometry(prediction);
  }

  void publish_localtraj(PLV(3) &pwld, double jour, IMUST &state,
                         int cur_session)
  {
    ResultOutput::instance().pub_localtraj(
        pwld, jour, state, cur_session, pcl_path, publish_scan_tf(),
        odom_frame, imu_child_frame);
  }

  void apply_dual_initialpose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &message)
  {
    string frame = message->header.frame_id;
    if(!frame.empty() && frame.front() == '/') frame.erase(0, 1);
    const auto &pose = message->pose.pose;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    const Eigen::Vector2d xy(pose.position.x, pose.position.y);
    if(frame != map_frame || !xy.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-12) {
      ROS_WARN_STREAM("Rejected RViz initial pose: expected finite 2D pose in " << map_frame);
      return;
    }
    if(!initialpose_tf_buffer || !imu_prediction_ready.load()) {
      ROS_WARN("Rejected RViz initial pose: local odometry is not initialized; retry after LIO is ready.");
      return;
    }
    const auto epoch = frontend_session_epoch.capture();
    try {
      // RViz's pose describes the robot base, not the IMU. Lookup includes the
      // calibrated IMU->base extrinsic and matches the click's sensor time.
      const auto local_tf = initialpose_tf_buffer->lookupTransform(
          odom_frame, initialpose_base_frame, rclcpp::Time(message->header.stamp),
          rclcpp::Duration::from_seconds(0.2));
      const auto &t = local_tf.transform;
      Eigen::Quaterniond local_q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
      Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
      local.linear() = local_q.normalized().toRotationMatrix();
      local.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
      q.normalize();
      const auto rotation = q.toRotationMatrix();
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      string error;
      // Serialize with the IMU publisher so no already-computed old transform
      // can be broadcast after the manual correction has been committed.
      lock_guard<mutex> output_lock(imu_output_mutex);
      const double now = dual_sensor_time.load();
      const auto current = dual_alignment.view(now);
      const Eigen::Isometry3d current_base = current.aligned ? current.map_from_odom * local : local;
      const auto desired = dual_tf_planar_base_pose(current_base, xy, yaw);
      if(!frontend_session_epoch.is_current(epoch) ||
         !dual_alignment.accept_manual(epoch, rclcpp::Time(local_tf.header.stamp).seconds(),
                                       now, desired, local, error)) {
        ROS_WARN_STREAM("Rejected RViz initial pose: " << error);
        return;
      }
      ROS_INFO_STREAM("DUAL_TF accepted RViz initial pose: base=" << initialpose_base_frame
                      << " x=" << xy.x() << " y=" << xy.y() << " yaw=" << yaw
                      << "; updated map->odom, local odometry unchanged");
    } catch(const tf2::TransformException &e) {
      ROS_WARN_STREAM("Rejected RViz initial pose: cannot obtain timestamped local base pose: " << e.what());
    } catch(const std::exception &e) {
      ROS_WARN_STREAM("Rejected RViz initial pose: " << e.what());
    }
  }

  void initialpose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &message)
  {
    if(!rviz_initialpose_enabled || !message) return;
    if(dual_tf) { apply_dual_initialpose(message); return; }
    const geometry_msgs::msg::Quaternion &orientation =
        message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if(!quaternion.coeffs().array().isFinite().all() ||
       quaternion.norm() < 1e-12)
    {
      ROS_WARN("Rejected RViz initial pose with an invalid quaternion.");
      return;
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const Eigen::Vector2d xy(message->pose.pose.position.x,
                             message->pose.pose.position.y);
    std::uint64_t generation = 0;
    string error;
    if(!initial_pose_requests.submit(message->header.frame_id,
                                     rclcpp::Time(message->header.stamp).seconds(), xy, yaw,
                                     generation, error))
    {
      ROS_WARN_STREAM("Rejected RViz initial pose: " << error);
      return;
    }
    ROS_INFO_STREAM("Captured RViz initial pose generation " << generation);
  }

  void publish_localization_status()
  {
    if(operation_mode != "localization") return;
    std_msgs::String state_message;
    std_msgs::Bool valid_message;
    state_message.data = localization_state_name(localization_machine.state());
    valid_message.data = localization_machine.localization_valid();
    pub_localization_state.publish(state_message);
    pub_localization_valid.publish(valid_message);
  }

  LocalizationTransition handle_localization_event(LocalizationEvent event)
  {
    lock_guard<mutex> lock(localization_state_mutex);
    string warning;
    LocalizationTransition transition =
        localization_machine.handle(event, warning);
    if(!transition.accepted)
      ROS_WARN_STREAM("Ignored " << warning << " in state "
                      << localization_state_name(localization_machine.state()));
    localization_valid.store(localization_machine.localization_valid());
    publish_localization_status();
    return transition;
  }

  LocalizationState localization_state_snapshot()
  {
    lock_guard<mutex> lock(localization_state_mutex);
    return localization_machine.state();
  }

  bool localization_local_map_ready_snapshot()
  {
    lock_guard<mutex> lock(localization_state_mutex);
    return localization_machine.local_map_ready();
  }

  AlignmentSource alignment_source_snapshot()
  {
    lock_guard<mutex> lock(localization_state_mutex);
    return localization_machine.alignment_source();
  }

  bool run_if_active_alignment(AlignmentSource source,
                               const std::function<void()> &action)
  {
    lock_guard<mutex> lock(localization_state_mutex);
    if(localization_machine.state() != LocalizationState::ALIGNING ||
       localization_machine.alignment_source() != source || !action)
      return false;
    action();
    return true;
  }

  void release_voxel_map(unordered_map<VOXEL_LOC, OctoTree*> &map)
  {
    vector<OctoTree*> descendants;
    for(auto &entry: map)
    {
      if(entry.second == nullptr) continue;
      entry.second->clear_slwd(sws[0]);
      entry.second->tras_ptr(descendants);
      delete entry.second;
      entry.second = nullptr;
    }
    for(OctoTree *node: descendants)
      delete node;
    map.clear();
  }

  void reset_spatial_map_session()
  {
    if(spatial_map_manager)
    {
      spatial_map_generation = spatial_map_manager->begin_session();
      map_gap_started_sec = -1.0;
    }
  }

  bool start_spatial_map_preparation(const Eigen::Vector3d &center,
                                     string &error)
  {
    error.clear();
    if(!spatial_map_manager || !spatial_map_matcher)
    {
      error = "spatial map manager is not initialized";
      return false;
    }
    spatial_map_generation = spatial_map_manager->begin_session();
    if(!spatial_map_manager->request(center, x_curr.v,
                                     spatial_map_generation, &error))
      return false;
    local_map_prepare_started_sec = ros::Time::now().toSec();
    map_gap_started_sec = -1.0;
    record_preparation_diagnostic("alignment_applied");

    // The temporary local-odometry map must not leak into map tracking.
    release_voxel_map(surf_map);
    surf_map_slide.clear();
    return true;
  }

  bool apply_frontend_global_correction(const IMUST &correction,
                                        string &error)
  {
    error.clear();
    lock_guard<mutex> state_lock(mtx_loop);
    if(global_correction_generation ==
       numeric_limits<std::uint64_t>::max())
    {
      error = "global correction generation is exhausted";
      return false;
    }
    if(win_count <= 0 || static_cast<size_t>(win_count) > x_buf.size())
    {
      error = "global correction found an invalid frontend window";
      return false;
    }

    CorrectionTargets targets;
    targets.current_state = toCorrectionState(x_curr);
    targets.rotate_gravity = (g_update == 1);
    targets.has_applied_generation = has_global_correction_generation;
    targets.last_applied_generation = global_correction_generation;
    targets.window_states.reserve(win_count);
    for(int i=0; i<win_count; ++i)
      targets.window_states.push_back(toCorrectionState(x_buf[i]));
    targets.cached_states.reserve(buf_lba2loop.size());
    for(ScanPose *pose: buf_lba2loop)
    {
      if(pose == nullptr)
      {
        error = "global correction found a null cached pose";
        return false;
      }
      targets.cached_states.push_back(toCorrectionState(pose->x));
    }

    const std::uint64_t next_generation = global_correction_generation + 1;
    if(!apply_global_correction_once(correction, next_generation, targets,
                                     error))
      return false;

    updateFromCorrectionState(targets.current_state, x_curr);
    for(int i=0; i<win_count; ++i)
      updateFromCorrectionState(targets.window_states[i], x_buf[i]);
    size_t cached_index = 0;
    for(ScanPose *pose: buf_lba2loop)
    {
      updateFromCorrectionState(targets.cached_states[cached_index], pose->x);
      ++cached_index;
    }
    has_global_correction_generation = true;
    global_correction_generation = targets.last_applied_generation;
    refresh_imu_rate_predictor_snapshot();
    return true;
  }

  // Returns 1 when a newer request starts validation, 0 when there is no
  // actionable request yet, and -1 after a hard failure requiring LIO reset.
  int try_start_direct_pose_validation(double now_sec, string &error)
  {
    error.clear();
    if(!rviz_initialpose_enabled || !prior_map_ready.load() ||
       !imu_orientation_ready)
      return 0;

    const PendingInitialPose request = initial_pose_requests.snapshot();
    if(!request.valid || request.generation == handled_direct_generation ||
       request.generation == active_direct_generation)
      return 0;

    if(!initial_pose_requests.is_current(request.generation))
      return 0;
    if(!handle_localization_event(
           LocalizationEvent::DIRECT_POSE_STARTED).accepted)
    {
      error = "localization state rejected the RViz initial pose";
      handled_direct_generation = request.generation;
      return -1;
    }

    DirectPoseSeed seed;
    if(!make_direct_pose_seed(request, true, x_curr.R, x_curr.p,
                              prior_map_tracker, seed, error))
    {
      handled_direct_generation = request.generation;
      handle_localization_event(LocalizationEvent::DIRECT_POSE_FAILED);
      return -1;
    }
    if(!initial_pose_requests.is_current(request.generation))
      return 0;

    IMUST correction;
    correction.R = seed.T_W_O.R;
    correction.p = seed.T_W_O.t;
    bool correction_succeeded = false;
    const bool request_was_current = initial_pose_requests.run_if_current(
        request.generation, [&]() {
          correction_succeeded =
              apply_frontend_global_correction(correction, error);
        });
    if(!request_was_current)
      return 0;
    if(!correction_succeeded)
    {
      handled_direct_generation = request.generation;
      reset_spatial_map_session();
      handle_localization_event(LocalizationEvent::DIRECT_POSE_FAILED);
      return -1;
    }
    if(!start_spatial_map_preparation(seed.T_W_C.t, error) ||
       !handle_localization_event(
            LocalizationEvent::ALIGNMENT_APPLIED).accepted)
    {
      handled_direct_generation = request.generation;
      reset_spatial_map_session();
      handle_localization_event(LocalizationEvent::DIRECT_POSE_FAILED);
      if(error.empty())
        error = "localization state rejected spatial-map preparation";
      return -1;
    }

    direct_validation_active = true;
    active_direct_generation = request.generation;
    handled_direct_generation = request.generation;
    direct_validation_streak = 0;
    direct_validation_started_sec = now_sec;
    ROS_INFO_STREAM("Started direct-pose validation for RViz generation "
                    << request.generation << " using height keyframe "
                    << seed.height_keyframe_id);
    return 1;
  }

  // The point-to-plane alignment for odometry
  bool lio_state_estimation(PVecPtr pptr, TrackingQuality *quality = nullptr,
                            double minimum_normal_eigenvalue = 14.0,
                            std::shared_ptr<const SpatialMapSnapshot>
                                spatial_snapshot = nullptr,
                            std::ostream *iteration_log = nullptr)
  {
    TrackingQuality candidate_quality;
    IMUST x_prop = x_curr;

    const int num_max_iter = 4;
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();
    int rematch_num = 0;
    int match_num = 0;
    double absolute_residual_sum = 0.0;
    bool solve_succeeded = true;

    int psize = pptr->size();
    vector<OctoTree*> octos;
    octos.resize(psize, nullptr);
    vector<StaticMapNodeHint> static_hints(psize);
    const bool use_spatial_map =
        spatial_snapshot != nullptr && spatial_map_matcher != nullptr;

    Eigen::Matrix3d nnt; 
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();
    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
      Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);
      match_num = 0;
      candidate_quality.map_query_points = 0U;
      candidate_quality.missing_block_points = 0U;
      absolute_residual_sum = 0.0;
      nnt.setZero();

      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        double sigma_d = 0;
        Plane* pla = nullptr;
        int flag = 0;
        Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
        Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
        if(use_spatial_map)
        {
          ++candidate_quality.map_query_points;
          try
          {
            const BlockIndex block_index =
                world_to_block(wld, localization_spatial_config);
            const auto block = spatial_snapshot->blocks.find(block_index);
            if(block != spatial_snapshot->blocks.end() && block->second)
            {
              StaticPlaneMatch static_match;
              const StaticMapNodeHint *hint =
                  static_hints[i].node_index == kInvalidSpatialIndex
                      ? nullptr : &static_hints[i];
              if(spatial_map_matcher->match(*block->second, wld, var_world,
                                            hint, static_match))
              {
                flag = 1;
                plane_center = static_match.center;
                plane_normal = static_match.normal;
                sigma_d = static_match.sigma_d;
                static_hints[i].block_index = static_match.block_index;
                static_hints[i].node_index = static_match.node_index;
              }
            }
            else
              ++candidate_quality.missing_block_points;
          }
          catch(const std::exception&)
          {
            flag = 0;
          }
        }
        else
        {
          if(octos[i] != nullptr && octos[i]->inside(wld))
          {
            double max_prob = 0;
            flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d,
                                   octos[i]);
          }
          else
          {
            flag = match(surf_map, wld, pla, var_world, sigma_d, octos[i]);
          }
          if(flag && pla != nullptr)
          {
            plane_center = pla->center;
            plane_normal = pla->normal;
          }
        }

        if(flag)
        {
          double R_inv = 1.0 / (0.0005 + sigma_d);
          double resi = plane_normal.dot(wld - plane_center);
          absolute_residual_sum += std::abs(resi);

          Eigen::Matrix<double, 6, 1> jac;
          jac.head(3) = phat * x_curr.R.transpose() * plane_normal;
          jac.tail(3) = plane_normal;
          HTH += R_inv * jac * jac.transpose();
          HTz -= R_inv * jac * resi;
          nnt += plane_normal * plane_normal.transpose();
          match_num++;
        }

      }

      if(iteration_log)
        *iteration_log << iterCount << ',' << psize << ',' << match_num << ','
                       << candidate_quality.missing_block_points << ','
                       << absolute_residual_sum << ',' << x_curr.p.x()
                       << ',' << x_curr.p.y() << ',' << x_curr.p.z()
                       << '\n';
      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      if(!K_1.array().isFinite().all() ||
         !solution.array().isFinite().all())
      {
        solve_succeeded = false;
        break;
      }

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      EKF_stop_flg = false;
      flg_EKF_converged = false;

      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) 
        flg_EKF_converged = true;

      if(flg_EKF_converged || ((rematch_num==0) && (iterCount==num_max_iter-2)))
      {       
        rematch_num++;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
    Eigen::Vector3d evalue = saes.eigenvalues();
    // printf("eva %d: %lf\n", match_num, evalue[0]);
    candidate_quality.matched_points = match_num;
    candidate_quality.mean_abs_point_plane =
        match_num > 0
            ? absolute_residual_sum / static_cast<double>(match_num)
            : std::numeric_limits<double>::infinity();
    candidate_quality.min_normal_eigenvalue = evalue[0];
    candidate_quality.finite =
        solve_succeeded && x_curr.R.array().isFinite().all() &&
        x_curr.p.array().isFinite().all() &&
        x_curr.v.array().isFinite().all() &&
        x_curr.cov.array().isFinite().all() &&
        std::isfinite(candidate_quality.mean_abs_point_plane) &&
        std::isfinite(candidate_quality.min_normal_eigenvalue);
    candidate_quality.solve_succeeded =
        solve_succeeded && match_num > 0;
    if(quality != nullptr)
      *quality = candidate_quality;

    return candidate_quality.solve_succeeded &&
           candidate_quality.finite &&
           candidate_quality.min_normal_eigenvalue >=
               minimum_normal_eigenvalue;
  }

  void record_preparation_diagnostic(const char *phase)
  {
    const char *directory = std::getenv("VOXEL_SLAM_MATCH_DIAGNOSTIC_DIR");
    if(!directory || !*directory) return;
    std::filesystem::create_directories(directory);
    const auto path = std::filesystem::path(directory) / "preparation_states.csv";
    const bool header = !std::filesystem::exists(path);
    std::ofstream out(path, std::ios::app);
    out << std::setprecision(17);
    if(header)
      out << "phase,sensor_time,clock_time,px,py,pz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,gx,gy,gz,r00,r01,r02,r10,r11,r12,r20,r21,r22,cov_rx,cov_ry,cov_rz,cov_px,cov_py,cov_pz\n";
    out << phase << ',' << x_curr.t << ',' << ros::Time::now().toSec();
    for(const auto &v : {x_curr.p, x_curr.v, x_curr.bg, x_curr.ba, x_curr.g})
      for(int i = 0; i < 3; ++i) out << ',' << v[i];
    for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c) out << ',' << x_curr.R(r,c);
    for(int i = 0; i < 6; ++i) out << ',' << x_curr.cov(i,i);
    out << '\n';
  }

  void run_frozen_match_diagnostic(
      PVecPtr points, const IMUST &input,
      const std::shared_ptr<const SpatialMapSnapshot> &loaded,
      const TrackingQuality &live_quality)
  {
    const char *directory = std::getenv("VOXEL_SLAM_MATCH_DIAGNOSTIC_DIR");
    static bool captured = false;
    if(captured || !directory || !*directory || !loaded) return;
    captured = true;
    // Freeze inputs and restore the live estimator even if diagnostic I/O fails.
    const IMUST live_state = x_curr;
    struct StateRestore {
      IMUST &target;
      IMUST saved;
      ~StateRestore() { target = saved; }
    } restore{x_curr, live_state};
    try
    {
      const std::filesystem::path root(directory);
      std::filesystem::create_directories(root);
      std::ofstream state(root / "input_state.txt");
      state << std::setprecision(17) << "t\n" << input.t << "\nR\n"
            << input.R << "\np\n" << input.p << "\nv\n" << input.v
            << "\nbg\n" << input.bg << "\nba\n" << input.ba
            << "\ng\n" << input.g << "\ncov\n" << input.cov << '\n';
      std::ofstream cloud(root / "input_points.csv");
      cloud << std::setprecision(17) << "x,y,z,var00,var01,var02,var10,var11,var12,var20,var21,var22\n";
      for(const auto &point : *points)
      {
        cloud << point.pnt.x() << ',' << point.pnt.y() << ',' << point.pnt.z();
        for(int r = 0; r < 3; ++r)
          for(int c = 0; c < 3; ++c) cloud << ',' << point.var(r,c);
        cloud << '\n';
      }
      SpatialMapCatalog catalog;
      string error;
      if(!SpatialMapIO::load_catalog(localization_map_path, catalog, error))
        throw std::runtime_error(error);
      auto complete = std::make_shared<SpatialMapSnapshot>();
      complete->generation = loaded->generation;
      std::ofstream blocks(root / "blocks.csv");
      blocks << "x,y,z,loaded_at_capture\n";
      for(const auto &entry : catalog.entries)
      {
        auto block = std::make_shared<StaticMapBlock>();
        if(!SpatialMapIO::load_block(localization_map_path, catalog,
                                    entry.index, *block, error))
          throw std::runtime_error(error);
        complete->blocks.emplace(entry.index, block);
        blocks << entry.index.x << ',' << entry.index.y << ',' << entry.index.z
               << ',' << loaded->blocks.count(entry.index) << '\n';
      }
      std::ofstream summary(root / "comparison.csv");
      summary << std::setprecision(17)
              << "map,blocks,solver_good,queries,missing_blocks,matched,mean_abs_residual,min_normal_eigenvalue,finite,px,py,pz\n";
      const auto compare = [&](const char *name,
                               std::shared_ptr<const SpatialMapSnapshot> snapshot,
                               const IMUST &initial) {
        x_curr = initial;
        std::ofstream iterations(root / (string(name) + "_iterations.csv"));
        iterations << std::setprecision(17)
                   << "iteration,queries,matched,missing_blocks,absolute_residual_sum,px,py,pz\n";
        TrackingQuality quality;
        const bool good = lio_state_estimation(points, &quality,
            tracking_min_normal_eigenvalue, snapshot, &iterations);
        summary << name << ',' << snapshot->blocks.size() << ',' << good << ','
                << quality.map_query_points << ',' << quality.missing_block_points
                << ',' << quality.matched_points << ',' << quality.mean_abs_point_plane
                << ',' << quality.min_normal_eigenvalue << ',' << quality.finite
                << ',' << x_curr.p.x() << ',' << x_curr.p.y() << ',' << x_curr.p.z() << '\n';
        if(string(name) == "loaded" &&
           (quality.matched_points != live_quality.matched_points ||
            quality.missing_block_points != live_quality.missing_block_points ||
            (x_curr.p - live_state.p).norm() > 1e-10 ||
            (x_curr.R - live_state.R).norm() > 1e-10 ||
            (x_curr.cov - live_state.cov).norm() > 1e-10))
          throw std::runtime_error("Frozen loaded-map replay differs from live match");
      };
      compare("loaded", loaded, input);
      compare("complete", complete, input);
      // Same-bag mapping poses are a diagnostic reference, not independent GT.
      std::vector<const SavedKeyframe *> ordered;
      for(const auto &keyframe : localization_map.keyframes)
        ordered.push_back(&keyframe);
      std::sort(ordered.begin(), ordered.end(), [](const auto *a, const auto *b) {
        return a->pose.stamp < b->pose.stamp;
      });
      auto upper = std::lower_bound(ordered.begin(), ordered.end(), input.t,
          [](const auto *keyframe, double time) { return keyframe->pose.stamp < time; });
      if(upper != ordered.begin() && upper != ordered.end())
      {
        const SavedPose &a = (*(upper - 1))->pose;
        const SavedPose &b = (*upper)->pose;
        const double span = b.stamp - a.stamp;
        if(span > 0.0 && span <= 3.0)
        {
          const double alpha = (input.t - a.stamp) / span;
          IMUST reference = input;
          reference.p = (1.0 - alpha) * a.t + alpha * b.t;
          reference.R = Eigen::Quaterniond(a.R).slerp(alpha,
              Eigen::Quaterniond(b.R)).toRotationMatrix();
          std::ofstream ref(root / "mapping_reference.txt");
          ref << std::setprecision(17) << "bracket_times " << a.stamp << ' '
              << b.stamp << "\nalpha " << alpha << "\np\n" << reference.p
              << "\nR\n" << reference.R << "\nposition_difference_m "
              << (input.p - reference.p).norm() << "\nrotation_difference_deg "
              << Eigen::AngleAxisd(reference.R.transpose() * input.R).angle()
                   * 180.0 / std::acos(-1.0) << '\n';
          compare("loaded_reference_pose", loaded, reference);
          compare("complete_reference_pose", complete, reference);
        }
      }
      std::ofstream done(root / "DONE");
      done << "frozen_match_pose_control_v1: frozen-input comparison complete; subsequent online timing is diagnostic-contaminated.\n";
      ROS_INFO_STREAM("Frozen first-match diagnostic saved to " << directory);
    }
    catch(const std::exception &error)
    {
      std::ofstream failure(std::filesystem::path(directory) / "ERROR");
      failure << error.what() << '\n';
      ROS_ERROR_STREAM("Frozen match diagnostic failed: " << error.what());
    }
  }

  // The point-to-plane alignment for initialization
  pcl::PointCloud<PointType>::Ptr pl_tree;
  void lio_state_estimation_kdtree(PVecPtr pptr)
  {
    static pcl::KdTreeFLANN<PointType> kd_map;
    if(pl_tree->size() < 100)
    {
      for(pointVar pv: *pptr)
      {
        PointType pp;
        pv.pnt = x_curr.R * pv.pnt + x_curr.p;
        pp.x = pv.pnt[0]; pp.y = pv.pnt[1]; pp.z = pv.pnt[2];
        pl_tree->push_back(pp);
      }
      kd_map.setInputCloud(pl_tree);
      return;
    }

    const int num_max_iter = 4;
    IMUST x_prop = x_curr;
    int psize = pptr->size();
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();

    double max_dis = 2*2;
    vector<float> sqdis(NMATCH); vector<int> nearInd(NMATCH);
    PLV(3) vecs(NMATCH);
    int rematch_num = 0;
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

    Eigen::Matrix<double, NMATCH, 1> b;
    b.setOnes();
    b *= -1.0f;

    vector<double> ds(psize, -1);
    PLV(3) directs(psize);
    bool refind = true;

    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      int valid = 0;
      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        if(refind)
        {
          PointType apx;
          apx.x = wld[0]; apx.y = wld[1]; apx.z = wld[2];
          kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);

          Eigen::Matrix<double, NMATCH, 3> A;
          for(int i=0; i<NMATCH; i++)
          {
            PointType &pp = pl_tree->points[nearInd[i]];
            A.row(i) << pp.x, pp.y, pp.z;
          }
          Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
          bool check_flag = false;
          for(int i=0; i<NMATCH; i++)
          {
            if(fabs(direct.dot(A.row(i)) + 1.0) > 0.1) 
              check_flag = true;
          }

          if(check_flag) 
          {
            ds[i] = -1;
            continue;
          }
          
          double d = 1.0 / direct.norm();
          // direct *= d;
          ds[i] = d;
          directs[i] = direct * d;
        }

        if(ds[i] >= 0)
        {
          double pd2 = directs[i].dot(wld) + ds[i];
          Eigen::Matrix<double, 6, 1> jac_s;
          jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
          jac_s.tail(3) = directs[i];

          HTH += jac_s * jac_s.transpose();
          HTz += jac_s * (-pd2);
          valid++;
        }
      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      refind = false;
      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
      {
        refind = true;
        flg_EKF_converged = true;
        rematch_num++;
      }

      if(iterCount == num_max_iter-2 && !flg_EKF_converged)
      {
        refind = true;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    double tt1 = ros::Time::now().toSec();
    for(pointVar pv: *pptr)
    {
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType ap;
      ap.x = pv.pnt[0]; ap.y = pv.pnt[1]; ap.z = pv.pnt[2];
      pl_tree->push_back(ap);
    }
    down_sampling_voxel(*pl_tree, 0.5);
    kd_map.setInputCloud(pl_tree);
    double tt2 = ros::Time::now().toSec();
  }

  // After detecting loop closure, refine current map and states
  void loop_update()
  {
    printf("loop update: %zu\n", sws[0].size());
    double t1 = ros::Time::now().toSec();

    string correction_error;
    if(!apply_frontend_global_correction(dx, correction_error))
    {
      ROS_ERROR("global correction rejected: %s", correction_error.c_str());
      loop_detect = 0;
      return;
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      // octos_release.push_back(iter->second);
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second; iter->second = nullptr;
    }
    surf_map.clear(); surf_map_slide.clear();
    surf_map = map_loop;
    map_loop.clear();

    printf("scanPoses: %zu %zu %zu %d %d %zu\n", scanPoses->size(), buf_lba2loop.size(), x_buf.size(), win_base, win_count, sws[0].size());
    int blsize = scanPoses->size();
    PointType ap = pcl_path[0];
    pcl_path.clear();
    
    for(int i=0; i<blsize; i++)
    {
      ap.x = scanPoses->at(i)->x.p[0];
      ap.y = scanPoses->at(i)->x.p[1];
      ap.z = scanPoses->at(i)->x.p[2];
      pcl_path.push_back(ap);
    }

    for(ScanPose *bl: buf_lba2loop)
    {
      ap.x = bl->x.p[0];
      ap.y = bl->x.p[1];
      ap.z = bl->x.p[2];
      pcl_path.push_back(ap);
    }
    
    for(int i=0; i<win_count; i++)
    {
      IMUST &x = x_buf[i];
      // PointType ap;
      ap.x = x.p[0]; ap.y = x.p[1]; ap.z = x.p[2];
      pcl_path.push_back(ap);
    }

    pub_pl_func(pcl_path, pub_curr_path);

    x_curr.R = x_buf[win_count-1].R;
    x_curr.p = x_buf[win_count-1].p;
    x_curr.g = x_buf[win_count-1].g;
    
    for(int i=0; i<win_size; i++)
      mp[i] = i;

    for(ScanPose *bl: buf_lba2loop)
    {
      IMUST xx = bl->x;
      PVec pvec_tem = *(bl->pvec);
      for(pointVar &pv: pvec_tem)
        pv.pnt = xx.R * pv.pnt + xx.p;
      cut_voxel(surf_map, pvec_tem, win_size, 0);
    }
    
    PLV(3) pwld;
    for(int i=0; i<win_count; i++)
    {
      pwld.clear();
      for(pointVar &pv: *pvec_buf[i])
        pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      iter->second->recut(win_count, x_buf, sws[0]);

    if(g_update == 1) g_update = 2;
    loop_detect = 0;
    double t2 = ros::Time::now().toSec();
    printf("loop head: %lf %zu\n", t2 - t1, sws[0].size());
  }

  // load the previous keyframe in the local voxel map
  void keyframe_loading(double jour)
  {
    if(history_kfsize <= 0) return;
    double tt1 = ros::Time::now().toSec();
    PointType ap_curr;
    ap_curr.x = x_curr.p[0];
    ap_curr.y = x_curr.p[1];
    ap_curr.z = x_curr.p[2];
    vector<int> vec_idx;
    vector<float> vec_dis;
    kd_keyframes.radiusSearch(ap_curr, 10, vec_idx, vec_dis);

    for(int id: vec_idx)
    {
      int ord_kf = pl_kdmap->points[id].curvature;
      if(keyframes->at(id)->exist)
      {
        Keyframe &kf = *(keyframes->at(id));
        IMUST &xx = kf.x0;
        PVec pvec; pvec.reserve(kf.plptr->size());

        pointVar pv; pv.var.setZero();
        int plsize = kf.plptr->size();
        // for(int j=0; j<plsize; j+=2)
        for(int j=0; j<plsize; j++)
        {
          PointType ap = kf.plptr->points[j];
          pv.pnt << ap.x, ap.y, ap.z;
          pv.pnt = xx.R * pv.pnt + xx.p;
          pvec.push_back(pv);
        }

        cut_voxel(surf_map, pvec, win_size, jour);
        kf.exist = 0;
        history_kfsize--;
        break;
      }
    }
    
  }

  int initialization(deque<sensor_msgs::msg::Imu::SharedPtr> &imus, Eigen::MatrixXd &hess, LidarFactor &voxhess, PLV(3) &pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
  {
    auto &pl_origs = initialization_clouds;
    auto &beg_times = initialization_times;
    auto &vec_imus = initialization_imus;

    pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));
    if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      return 0;

    if(win_count == 0)
      imupre_scale_gravity = odom_ekf.scale_gravity;

    PVecPtr pptr(new PVec);
    double downkd = down_size >= 0.5 ? down_size : 0.5;
    down_sampling_voxel(*pcl_curr, downkd);
    var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
    lio_state_estimation_kdtree(pptr);

    pwld.clear();
    pvec_update(pptr, x_curr, pwld);

    win_count++;
    x_buf.push_back(x_curr);
    pvec_buf.push_back(pptr);
    publish_localtraj(pwld, 0, x_curr, sessionNames.size()-1);

    if(win_count > 1)
    {
      imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
      imu_pre_buf[win_count-2]->push_imu(imus);
    }

    pcl::PointCloud<PointType> pl_mid = *orig;
    down_sampling_close(*orig, down_size);
    if(orig->size() < 1000)
    {
      *orig = pl_mid;
      down_sampling_close(*orig, down_size / 2);
    }

    sort(orig->begin(), orig->end(), [](PointType &x, PointType &y)
    {return x.curvature < y.curvature;});

    if(needs_details()) {
      initialization_details.push_back(pl_mid.makeShared());
      std::sort(initialization_details.back()->begin(),initialization_details.back()->end(),
                [](const PointType &a,const PointType &b) { return a.curvature<b.curvature; });
    }
    pl_origs.push_back(orig);
    beg_times.push_back(odom_ekf.pcl_beg_time);
    vec_imus.push_back(imus);

    int is_success = 0;
    if(win_count >= win_size)
    {
      auto detail_imus=vec_imus;
      auto detail_times=beg_times;
      is_success = Initialization::instance().motion_init(pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map, surf_map_slide, pvec_buf, win_size, sws, x_curr, imu_pre_buf, extrin_para);
      if(is_success && needs_details()) {
        detail_window.clear();
        for(int i=0;i<win_size;++i) {
          PVec dense;
          Initialization::instance().motion_blur(*initialization_details.at(i), dense,
              x_buf[i], x_buf[i ? i-1 : 0], detail_imus[i], detail_times[i], extrin_para);
          pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
          for(const auto &p : dense) {
            PointType q{}; q.x=p.pnt.x(); q.y=p.pnt.y(); q.z=p.pnt.z(); cloud->push_back(q);
          }
          detail_window.push_back(cloud);
          export_scan(*cloud,x_buf[i]);
        }
      }
      initialization_details.clear();

      if(is_success == 0)
        return -1;
      return 1;
    }
    return 0;
  }

  void system_reset(deque<sensor_msgs::msg::Imu::SharedPtr> &imus, bool reset_imu_history = false)
  {
    reset_imu_rate_predictor();
    string epoch_error;
    if(!frontend_session_epoch.advance(epoch_error))
    {
      ROS_FATAL_STREAM("Cannot reset frontend session: " << epoch_error);
      ros::shutdown();
      return;
    }
    if(dual_tf)
    {
      dual_alignment.reset(frontend_session_epoch.capture());
      dual_initializing.store(true);
      dual_prediction_wall.store(-1);
      lock_guard<mutex> lock(mtx_loop);
      for(auto *pose : buf_lba2loop) delete pose;
      buf_lba2loop.clear();
      dual_scans.clear();
      dual_latest_scans.clear();
      publish_dual_status();
    }
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    surf_map.clear(); surf_map_slide.clear();

    x_curr.setZero();
    x_curr.p = Eigen::Vector3d(0, 0, 30);
    odom_ekf.mean_acc.setZero();
    odom_ekf.init_num = 0;
    if(dual_tf && (reset_imu_history || imu_orientation_ready))
    {
      odom_ekf.init_flag = false;
      odom_ekf.last_imu.reset();
      odom_ekf.mean_gyr.setZero();
      odom_ekf.angvel_last.setZero();
      odom_ekf.acc_s_last.setZero();
      odom_ekf.imu_poses.clear();
      odom_ekf.last_pcl_end_time = -1;
    }
    if(!imus.empty()) odom_ekf.IMU_init(imus);
    x_curr.g = -odom_ekf.mean_acc * imupre_scale_gravity;
    imu_orientation_ready = false;

    for(int i=0; i<imu_pre_buf.size(); i++)
      delete imu_pre_buf[i];
    x_buf.clear(); pvec_buf.clear(); imu_pre_buf.clear();
    detail_window.clear(); initialization_details.clear();
    initialization_clouds.clear();
    initialization_times.clear();
    initialization_imus.clear();
    pl_tree->clear();

    for(int i=0; i<win_size; i++)
      mp[i] = i;
    win_base = 0; win_count = 0; pcl_path.clear();
    pub_pl_func(pcl_path, pub_cmap);
    ROS_WARN("Reset");
  }

  // After local BA, update the map and marginalize the points of oldest scan
  // multi means multiple thread
  void multi_margi(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, double jour, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<SlideWindow*> &sw)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    // {
    //   iter->second->jour = jour;
    //   iter->second->margi(win_count, 1, xs, voxopt);
    //   if(iter->second->isexist)
    //     iter++;
    //   else
    //   {
    //     iter->second->clear_slwd(sw);
    //     feat_map.erase(iter++);
    //   }
    // }
    // return;

    int thd_num = thread_num;
    vector<vector<OctoTree*>*> octs;
    for(int i=0; i<thd_num; i++) 
      octs.push_back(new vector<OctoTree*>());

    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      iter->second->jour = jour;
      octs[cnt]->push_back(iter->second);
      if(octs[cnt]->size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto margi_func = [](int win_cnt, vector<OctoTree*> *oct, vector<IMUST> xxs, LidarFactor &voxhess)
    {
      for(OctoTree *oc: *oct)
      {
        oc->margi(win_cnt, 1, xxs, voxhess);
      }
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
    }
    
    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        margi_func(win_count, octs[i], xs, voxopt);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    {
      if(iter->second->isexist)
        iter++;
      else
      {
        iter->second->clear_slwd(sw);
        feat_map.erase(iter++);
      }
    }

    for(int i=0; i<thd_num; i++)
      delete octs[i];

  }

  // Determine the plane and recut the voxel map in octo-tree
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<vector<SlideWindow*>> &sws)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    // {
    //   iter->second->recut(win_count, xs, sws[0]);
    //   iter->second->tras_opt(voxopt);
    // }

    int thd_num = thread_num;
    vector<vector<OctoTree*>> octss(thd_num);
    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      octss[cnt].push_back(iter->second);
      if(octss[cnt].size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto recut_func = [](int win_count, vector<OctoTree*> &oct, vector<IMUST> xxs, vector<SlideWindow*> &sw)
    {
      for(OctoTree *oc: oct)
        oc->recut(win_count, xxs, sw);
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));
    }

    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        recut_func(win_count, octss[i], xs, sws[i]);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(int i=1; i<sws.size(); i++)
    {
      sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
      sws[i].clear();
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
      iter->second->tras_opt(voxopt);

  }

  // The main thread of odometry and local mapping
  void thd_odometry_localmapping(ros::NodeHandle &n)
  {
    PLV(3) pwld;
    double down_sizes[3] = {0.1, 0.2, 0.4};
    Eigen::Vector3d last_pos(0, 0 ,0);
    double jour = 0;
    int counter = 0;

    pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
    int motion_init_flag = 1;
    pl_tree.reset(new pcl::PointCloud<PointType>());
    vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    vector<double> beg_times;
    vector<deque<sensor_msgs::msg::Imu::SharedPtr>> vec_imus;
    bool release_flag = false;
    int degrade_cnt = 0;
    LidarFactor voxhess(win_size);
    const int mgsize = 1;
    Eigen::MatrixXd hess;
    while(n.ok())
    {
      ros::spinOnce();
      if(!n.ok()) break;
      if(loop_detect == 1)
      {
        loop_update(); last_pos = x_curr.p; jour = 0;
      }
      
      bool finish_requested = false;
      n.param<bool>("finish", finish_requested, false);
      bool no_ba_requested = false;
      n.param<bool>("finish_no_ba", no_ba_requested, false);
      if(no_ba_requested && operation_mode == "mapping") {
        finish_without_ba.store(true);
        stop_gba_worker.store(true);
        finish_requested = true;
      }
      if(finish_requested && scan_map_writer && !scan_export_failed) {
        for(const auto &x : x_buf) {
          const auto found=scan_export_ids.find(x.t);
          if(found!=scan_export_ids.end()) scan_map_writer->set_pose(found->second,saved_pose(x));
        }
      }
      is_finish.store(finish_requested);
      if(finish_requested)
      {
        shutdown_imu_rate_output();
        break;
      }

      // Idle recycling shares map and slide-window ownership with BTC alignment.
      unique_lock<mutex> localization_pipeline_lock;
      if(operation_mode == "localization")
        localization_pipeline_lock =
            unique_lock<mutex>(frontend_pipeline_mutex);
      if(dual_tf && frontend_reset_requested.exchange(false))
      {
        {
          lock_guard<mutex> lock(mBuf);
          pcl_buf.clear();
          time_buf.clear();
        }
        deque<sensor_msgs::msg::Imu::SharedPtr> reset_seed;
        sync_packages(pcl_curr, reset_seed, odom_ekf, true);
        system_reset(reset_seed, true);
        motion_init_flag = 1;
        reset_flag = 1;
        degrade_cnt = 0;
        last_pos = x_curr.p;
        jour = 0;
        continue;
      }
      deque<sensor_msgs::msg::Imu::SharedPtr> imus;
      if(!sync_packages(pcl_curr, imus, odom_ekf))
      {
        if(octos_release.size() != 0)
        {
          int msize = octos_release.size();
          if(msize > 1000) msize = 1000;
          for(int i=0; i<msize; i++)
          {
            delete octos_release.back();
            octos_release.pop_back();
          }
          malloc_trim(0);
        }
        else if(release_flag)
        {
          release_flag = false;
          vector<OctoTree*> octos;
          for(auto iter=surf_map.begin(); iter!=surf_map.end();)
          {
            int dis = jour - iter->second->jour;
            if(dis < 700)
            // if(dis < 200)
            {
              iter++;
            }
            else
            {
              octos.push_back(iter->second);
              iter->second->tras_ptr(octos);
              surf_map.erase(iter++);
            }
          }
          int ocsize = octos.size();
          for(int i=0; i<ocsize; i++)
            delete octos[i];
          octos.clear();
          malloc_trim(0);
        }
        else if(sws[0].size() > 10000)
        {
          for(int i=0; i<500; i++)
          {
            delete sws[0].back();
            sws[0].pop_back();
          }
          malloc_trim(0);
        }

        sleep(0.001);
        continue;
      }

      if(frontend_reset_requested.exchange(false))
      {
        system_reset(imus);
        motion_init_flag = 1;
        reset_flag = 1;
        continue;
      }
      if(lost_recovery_pending)
      {
        handle_localization_event(LocalizationEvent::RECOVERY_STARTED);
        lost_recovery_pending = false;
      }

      static int first_flag = 1;
      if (first_flag)
      {
        pcl::PointCloud<PointType> pl;
        pub_pl_func(pl, pub_pmap);
        pub_pl_func(pl, pub_prev_path);
        first_flag = 0;
      }

      double t0 = ros::Time::now().toSec();
      const auto frame_steady_started = std::chrono::steady_clock::now();
      double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0, t7=0, t8=0;

      if(motion_init_flag)
      {
        int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

        if(init == 1)
        {
          motion_init_flag = 0;
          if(dual_tf) dual_initializing.store(false);
          imu_orientation_ready = true;
          refresh_imu_rate_predictor_snapshot();
        }
        else
        {
          if(init == -1)
            system_reset(imus);
          continue;
        }
      }
      else
      {
        if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
          continue;

        string direct_pose_error;
        const int direct_start = dual_tf ? 0 : try_start_direct_pose_validation(
            ros::Time::now().toSec(), direct_pose_error);
        if(direct_start < 0)
        {
          ROS_WARN_STREAM("Direct-pose initialization failed: "
                          << direct_pose_error);
          direct_validation_active = false;
          active_direct_generation = 0;
          reset_spatial_map_session();
          system_reset(imus);
          motion_init_flag = 1;
          reset_flag = 1;
          continue;
        }

        auto detail = needs_details() ? imu_cloud(*pcl_curr) : pcl::PointCloud<PointType>::Ptr();
        pcl::PointCloud<PointType> pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size);

        if(pl_down.size() < 500)
        {
          pl_down = *pcl_curr;
          down_sampling_voxel(pl_down, down_size / 2);
        }

        PVecPtr pptr(new PVec);
        var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

        TrackingQuality scan_quality;
        LocalizationState current_localization_state =
            localization_state_snapshot();
        if(!dual_tf && operation_mode == "localization" && prior_map_ready.load() &&
           (current_localization_state ==
                LocalizationState::PREPARING_LOCAL_MAP ||
            current_localization_state ==
                LocalizationState::IN_MAP_TRACKING))
        {
          const IMUST propagated_state = x_curr;
          SpatialFrameMetrics frame_metrics;
          frame_metrics.sensor_time = odom_ekf.pcl_end_time;
          SpatialFrameLogScope metrics_scope;
          if(spatial_map_manager && spatial_map_manager->metrics_enabled())
          {
            metrics_scope.write = [&] {
              frame_metrics.state = localization_state_name(localization_state_snapshot());
              frame_metrics.valid = localization_valid.load();
              frame_metrics.query_points = scan_quality.map_query_points;
              frame_metrics.missing_block_points = scan_quality.missing_block_points;
              frame_metrics.matched_points = scan_quality.matched_points;
              frame_metrics.residual = scan_quality.mean_abs_point_plane;
              frame_metrics.frame_compute_ms = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - frame_steady_started).count();
              spatial_map_manager->record_frame(frame_metrics);
            };
          }
          const auto request_started = std::chrono::steady_clock::now();
          string map_request_error;
          if(!spatial_map_manager ||
             !spatial_map_manager->request(x_curr.p, x_curr.v,
                                           spatial_map_generation,
                                           &map_request_error))
          {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "Spatial map request failed: " << map_request_error);
          }
          frame_metrics.schedule_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - request_started).count();
          SpatialMapStatus map_status = spatial_map_manager
                                            ? spatial_map_manager->status()
                                            : SpatialMapStatus();
          if(map_status.failed_core_block_count > 0U)
          {
            ROS_WARN_STREAM("Required spatial map block failed to load: "
                            << map_status.last_error);
            if(current_localization_state ==
               LocalizationState::IN_MAP_TRACKING)
            {
              handle_localization_event(
                  LocalizationEvent::MAP_GAP_TIMEOUT);
              lost_recovery_pending = true;
            }
            else if(alignment_source_snapshot() ==
                    AlignmentSource::RVIZ_INITIAL_POSE)
            {
              handle_localization_event(
                  LocalizationEvent::DIRECT_POSE_FAILED);
            }
            else
            {
              handle_localization_event(
                  LocalizationEvent::LOCAL_MAP_FAILED);
            }
            direct_validation_active = false;
            active_direct_generation = 0;
            reset_spatial_map_session();
            system_reset(imus);
            motion_init_flag = 1;
            reset_flag = 1;
            continue;
          }
          if(map_status.failed_block_count > 0U)
          {
            ROS_WARN_STREAM_THROTTLE(
                2.0, "Non-core spatial map block is unavailable for this "
                     "session: " << map_status.last_error);
          }
          if(current_localization_state ==
                 LocalizationState::PREPARING_LOCAL_MAP &&
             map_status.core_ready &&
             !localization_local_map_ready_snapshot())
          {
            if(handle_localization_event(
                   LocalizationEvent::LOCAL_MAP_READY).accepted &&
               alignment_source_snapshot() ==
                   AlignmentSource::RVIZ_INITIAL_POSE)
              direct_validation_started_sec = ros::Time::now().toSec();
            current_localization_state = localization_state_snapshot();
          }

          const bool waiting_for_first_map =
              current_localization_state ==
                  LocalizationState::PREPARING_LOCAL_MAP &&
              !localization_local_map_ready_snapshot() &&
              !map_status.core_ready;
          if(waiting_for_first_map)
          {
            const double waiting_sec = ros::Time::now().toSec() -
                                       local_map_prepare_started_sec;
            if(waiting_sec > spatial_runtime_config.preload_timeout_sec ||
               (map_status.idle && !map_status.last_error.empty()))
            {
              ROS_WARN_STREAM("Spatial local-map preparation failed: "
                              << map_status.last_error);
              handle_localization_event(LocalizationEvent::LOCAL_MAP_FAILED);
              direct_validation_active = false;
              active_direct_generation = 0;
              reset_spatial_map_session();
              system_reset(imus);
              motion_init_flag = 1;
              reset_flag = 1;
              continue;
            }

            // IMU-only propagation while the first immutable core is loading.
            record_preparation_diagnostic("imu_wait");
            pwld.clear();
            pvec_update(pptr, x_curr, pwld);
            publish_localtraj(
                pwld, jour, x_curr, sessionNames.size()-1);
            continue;
          }

          const std::shared_ptr<const SpatialMapSnapshot> spatial_snapshot =
              spatial_map_manager ? spatial_map_manager->snapshot() : nullptr;
          bool tracking_good = false;
          double spatial_match_ms = 0.0;
          if(spatial_snapshot && !spatial_snapshot->blocks.empty())
          {
            record_preparation_diagnostic("match_input");
            const auto match_started = std::chrono::steady_clock::now();
            const bool solver_good =
                lio_state_estimation(pptr, &scan_quality,
                                     tracking_min_normal_eigenvalue,
                                     spatial_snapshot);
            spatial_match_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - match_started).count();
            frame_metrics.match_ms = spatial_match_ms;
            tracking_good =
                solver_good &&
                scan_quality.matched_points >= tracking_min_matched_points &&
                scan_quality.mean_abs_point_plane <=
                    tracking_max_mean_abs_point_plane &&
                scan_quality.min_normal_eigenvalue >=
                    tracking_min_normal_eigenvalue;
            run_frozen_match_diagnostic(pptr, propagated_state,
                                        spatial_snapshot, scan_quality);
          }
          last_tracking_quality = scan_quality;

          const bool loading_required_core =
              !tracking_good && !map_status.core_ready &&
              map_status.pending_block_count > 0U;
          if(loading_required_core)
          {
            x_curr = propagated_state;
            if(map_gap_started_sec < 0.0)
              map_gap_started_sec = ros::Time::now().toSec();
            if(ros::Time::now().toSec() - map_gap_started_sec >
               spatial_runtime_config.map_gap_timeout_sec)
            {
              const LocalizationTransition gap_transition =
                  handle_localization_event(
                      LocalizationEvent::MAP_GAP_TIMEOUT);
              if(gap_transition.cleanup_requested)
              {
                direct_validation_active = false;
                active_direct_generation = 0;
                reset_spatial_map_session();
                system_reset(imus);
                motion_init_flag = 1;
                reset_flag = 1;
                lost_recovery_pending = true;
                continue;
              }
            }
          }
          else
          {
            map_gap_started_sec = -1.0;
          }

          if(!loading_required_core && scan_quality.solve_succeeded &&
             scan_quality.finite)
            refresh_imu_rate_predictor_snapshot();

          if(tracking_good)
          {
            if(degrade_cnt > 0) degrade_cnt--;
          }
          else if(!loading_required_core)
          {
            degrade_cnt++;
          }

          // Do not score or time out an RViz pose while a newly required
          // core is still being fetched.  This scan was deliberately rolled
          // back to IMU propagation and therefore carries no map evidence.
          if(direct_validation_active && !loading_required_core)
          {
            if(tracking_good)
              direct_validation_streak++;
            else
              direct_validation_streak = 0;

            if(direct_validation_streak >= direct_pose_validation_scans)
            {
              direct_validation_active = false;
              handle_localization_event(
                  LocalizationEvent::DIRECT_POSE_VALIDATED);
              ROS_INFO_STREAM(
                  "RViz direct pose validated after "
                  << direct_validation_streak << " consecutive scans.");
            }
            else if(ros::Time::now().toSec() -
                        direct_validation_started_sec >
                    direct_pose_validation_timeout_sec)
            {
              ROS_WARN("RViz direct-pose validation timed out; resetting temporary LIO.");
              direct_validation_active = false;
              active_direct_generation = 0;
              handle_localization_event(
                  LocalizationEvent::DIRECT_POSE_FAILED);
              reset_spatial_map_session();
              system_reset(imus);
              motion_init_flag = 1;
              reset_flag = 1;
              continue;
            }
          }

          if(!direct_validation_active && localization_valid.load() &&
             !loading_required_core)
          {
            LocalizationEvent tracking_event =
                !scan_quality.finite
                    ? LocalizationEvent::TRACKING_NONFINITE
                    : (tracking_good ? LocalizationEvent::TRACKING_GOOD
                                     : LocalizationEvent::TRACKING_BAD);
            const LocalizationTransition tracking_transition =
                handle_localization_event(tracking_event);
            if(tracking_transition.cleanup_requested)
            {
              ROS_WARN("Localization entered LOST; clearing current-session state.");
              active_direct_generation = 0;
              direct_validation_active = false;
              reset_spatial_map_session();
              system_reset(imus);
              motion_init_flag = 1;
              reset_flag = 1;
              lost_recovery_pending = true;
              continue;
            }
          }

          pwld.clear();
          pvec_update(pptr, x_curr, pwld);
          publish_localtraj(
              pwld, jour, x_curr, sessionNames.size()-1);

          const SpatialMapStatus diagnostic_status =
              spatial_map_manager->status();
          ROS_INFO_STREAM("SPATIAL_MAP active="
                          << (spatial_snapshot
                                  ? spatial_snapshot->blocks.size() : 0U)
                          << " pending="
                          << diagnostic_status.pending_block_count
                          << " core="
                          << (diagnostic_status.core_ready ? 1 : 0)
                          << " load_seq="
                          << diagnostic_status.load_sequence
                          << " load_ms="
                          << diagnostic_status.last_load_ms
                          << " match_ms=" << spatial_match_ms
                          << " frame_ms="
                          << ((ros::Time::now().toSec() - t0) * 1000.0)
                          << " memory_mb="
                          << spatialSnapshotMemoryMb(spatial_snapshot));

          // Historical localization is read-only: the current scan is never
          // inserted into surf_map or any sliding-window map.
          continue;
        }

        const IMUST local_prediction = x_curr;
        const bool scan_update_good =
            lio_state_estimation(pptr, &scan_quality);
        if(dual_tf && (!x_curr.R.allFinite() || !x_curr.p.allFinite() ||
                       !x_curr.v.allFinite() || !x_curr.bg.allFinite() ||
                       !x_curr.ba.allFinite() || !x_curr.g.allFinite() || !x_curr.cov.allFinite()))
        {
          x_curr = local_prediction;
          ROS_ERROR("Local LIO state became nonfinite; starting a new odometry session.");
          system_reset(imus);
          motion_init_flag = 1;
          reset_flag = 1;
          degrade_cnt = 0;
          continue;
        }
        if(scan_update_good)
        {
          if(degrade_cnt > 0) degrade_cnt--;
        }
        else
          degrade_cnt++;
        last_tracking_quality = scan_quality;
        if(!scan_update_good &&
           (degrade_cnt == 1 || degrade_cnt % 25 == 0))
          ROS_WARN_STREAM("Mapping tracking degraded: stamp="
                          << std::fixed << odom_ekf.pcl_end_time
                          << " count=" << degrade_cnt << "/" << degrade_bound
                          << " points=" << pptr->size()
                          << " matched=" << scan_quality.matched_points
                          << " normal_min_eigen="
                          << scan_quality.min_normal_eigenvalue
                          << " required=14 residual="
                          << scan_quality.mean_abs_point_plane
                          << " finite=" << scan_quality.finite
                          << " solved=" << scan_quality.solve_succeeded);
        if(scan_quality.solve_succeeded && scan_quality.finite)
          refresh_imu_rate_predictor_snapshot();

        pwld.clear();
        pvec_update(pptr, x_curr, pwld);
        publish_localtraj(pwld, jour, x_curr, sessionNames.size()-1);

        t1 = ros::Time::now().toSec();

        win_count++;
        x_buf.push_back(x_curr);
        pvec_buf.push_back(pptr);
        if(detail) { detail_window.push_back(detail); export_scan(*detail,x_curr); }
        if(dual_tf && (direct_options.enabled || relocalization_backend != "btc"))
        {
          lock_guard<mutex> lock(mtx_loop);
          PVecPtr global_points(new PVec);
          for(const auto &p : *detail_window.back()) {
            pointVar q; q.pnt=p.getVector3fMap().cast<double>(); q.var.setZero(); global_points->push_back(q);
          }
          dual_latest_scans.push_back({frontend_session_epoch.capture(), x_curr, global_points});
          while(dual_latest_scans.size() > static_cast<size_t>(dual_queue_capacity)) dual_latest_scans.pop_front();
        }
        if(win_count > 1)
        {
          imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
          imu_pre_buf[win_count-2]->push_imu(imus);
        }
        
        keyframe_loading(jour);
        voxhess.clear(); voxhess.win_size = win_size;

        // cut_voxel(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws[0]);
        cut_voxel_multi(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws);
        t2 = ros::Time::now().toSec();

        multi_recut(surf_map_slide, win_count, x_buf, voxhess, sws);
        t3 = ros::Time::now().toSec();

        if(degrade_cnt > degrade_bound)
        {
          ROS_WARN_STREAM("Mapping reset triggered by accumulated tracking "
                          "degradation: stamp=" << std::fixed
                          << odom_ekf.pcl_end_time << " count=" << degrade_cnt
                          << " bound=" << degrade_bound
                          << " matched=" << scan_quality.matched_points
                          << " normal_min_eigen="
                          << scan_quality.min_normal_eigenvalue);
          degrade_cnt = 0;
          system_reset(imus);

          last_pos = x_curr.p; jour = 0;

          mtx_loop.lock();
          buf_lba2loop_tem.swap(buf_lba2loop);
          mtx_loop.unlock();
          reset_flag = 1;

          motion_init_flag = 1;
          history_kfsize = 0;

          continue;
        }
      }

      if(win_count >= win_size)
      {
        t4 = ros::Time::now().toSec();
        
        if(dual_tf || !local_ba_enabled)
        {
          // Keep the sliding map without optimizing its local poses.
        }
        else if(g_update == 2)
        {
          LI_BA_OptimizerGravity opt_lsv;
          vector<double> resis;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, &hess, 5);
          printf("g update: %lf %lf %lf: %lf\n", x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm());
          g_update = 0;
          x_curr.g = x_buf[win_count-1].g;
        }
        else
        {
          LI_BA_Optimizer opt_lsv;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        }

        if(!dual_tf)
        {
          ScanPose *bl = new ScanPose(x_buf[0], pvec_buf[0]);
          if(scan_map_writer && !scan_export_failed) {
            const auto found=scan_export_ids.find(bl->x.t);
            if(found==scan_export_ids.end()) scan_export_failed=true;
            else scan_pose_links.emplace_back(found->second,bl);
          }
          if(local_ba_enabled) {
            bl->v6 = hess.block<6, 6>(0, DIM).diagonal();
            for(int i=0; i<6; i++) bl->v6[i] = 1.0 / fabs(bl->v6[i]);
          } else {
            bl->v6 = bl->x.cov.block<6, 6>(0, 0).diagonal();
            for(int i=0; i<6; i++)
              bl->v6[i] = std::isfinite(bl->v6[i]) ? std::max(1e-9, bl->v6[i]) : 1.0;
          }
          lock_guard<mutex> lock(mtx_loop);
          buf_lba2loop.push_back(bl);
        }

        x_curr.R = x_buf[win_count-1].R;
        x_curr.p = x_buf[win_count-1].p;
        refresh_imu_rate_predictor_snapshot();
        t5 = ros::Time::now().toSec();

        ResultOutput::instance().pub_localmap(mgsize, sessionNames.size()-1, pvec_buf, x_buf, pcl_path, win_base, win_count, dual_tf ? odom_frame : "camera_init");

        multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
        t6 = ros::Time::now().toSec();

        if((win_base + win_count) % 10 == 0)
        {
          double spat = (x_curr.p - last_pos).norm();
          if(spat > 0.5)
          {
            jour += spat;
            last_pos = x_curr.p;
            release_flag = true;
          }
        }

        if(is_save_map)
        {
          for(int i=0; i<mgsize; i++)
            FileReaderWriter::instance().save_pcd(pvec_buf[i], x_buf[i], win_base + i, savepath + bagname);
        }

        for(int i=0; i<win_size; i++)
        {
          mp[i] += mgsize;
          if(mp[i] >= win_size) mp[i] -= win_size;
        }

        if(!detail_window.empty())
          detail_window.erase(detail_window.begin(), detail_window.begin()+mgsize);
        for(int i=mgsize; i<win_count; i++)
        {
          x_buf[i-mgsize] = x_buf[i];
          PVecPtr pvec_tem = pvec_buf[i-mgsize];
          pvec_buf[i-mgsize] = pvec_buf[i];
          pvec_buf[i] = pvec_tem;
        }

        for(int i=win_count-mgsize; i<win_count; i++)
        {
          x_buf.pop_back();
          pvec_buf.pop_back();

          delete imu_pre_buf.front();
          imu_pre_buf.pop_front();
        }

        win_base += mgsize; win_count -= mgsize;
      }
      
      double t_end = ros::Time::now().toSec();
      double mem = get_memory();
      // printf("%d: %.4lf: %.4lf %.4lf %.4lf %.4lf %.4lf %.2lfGb %.1lf\n", win_base+win_count, t_end-t0, t1-t0, t2-t1, t3-t2, t5-t4, t6-t5, mem, jour);

      // printf("%d: %lf %lf %lf\n", win_base + win_count, x_curr.p[0], x_curr.p[1], x_curr.p[2]);
    }

    unique_lock<mutex> localization_cleanup_lock;
    if(operation_mode == "localization")
      localization_cleanup_lock = unique_lock<mutex>(frontend_pipeline_mutex);
    vector<OctoTree *> octos;
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    surf_map.clear();
    surf_map_slide.clear();

    for(int i=0; i<octos.size(); i++)
      delete octos[i];
    octos.clear();

    for(int i=0; i<sws[0].size(); i++)
      delete sws[0][i];
    sws[0].clear();
    malloc_trim(0);
  }

  const SavedKeyframe *find_localization_keyframe(int id) const
  {
    for(const SavedKeyframe &keyframe: localization_map.keyframes)
      if(keyframe.id == id) return &keyframe;
    return nullptr;
  }



  void thd_dual_tf_sc(ros::NodeHandle &n, ConfigSetting &config)
  {
    const bool use_sc = true;
    const char *recovery_name = "SC_QN";
    std::unique_ptr<ScanContextRelocalizer> sc_relocalizer;
    if(use_sc) {
      const auto started = steady_seconds();
      sc_relocalizer.reset(new ScanContextRelocalizer(descriptor_history_map, config, scancontext_options,
          [&](int id) { return scan_map_store->fuse(id,history_scans); }));
      ROS_INFO_STREAM("SC_QN history ready: keyframes=" << descriptor_history_map.keyframes.size()
          << " query_scans=" << scancontext_options.query_scans
          << " build_ms=" << (steady_seconds()-started)*1000);
    }
    string error;
    DirectMatchSchedule direct_schedule(direct_options);
    std::unique_ptr<DirectLocalMap> direct_map;
    if(direct_options.enabled) direct_map.reset(new DirectLocalMap(localization_map, config, direct_options,
        [&](int id) { return scan_map_store->scan(id); },
        [&](int id) { return scan_map_store->planes(id,config); }));
    deque<DualScan> latest;
    int direct_failures = 0;
    bool recovery_active = false, previous_was_direct = false;
    std::uint64_t epoch = frontend_session_epoch.capture();
    auto manual_revision = dual_alignment.view(dual_sensor_time.load()).manual_revision;
    int query_id = 0;
    double last_query_wall = -1;
    bool query_pending = false;
    int sc_since_query = 0;
    struct PendingAudit {
      bool active = false;
      int history_id = -1;
      int anchor_id = -1;
      double score = 0;
      bool direct = false;
      bool scancontext = false;
      STDescManager::PlaneCloud::Ptr target;
      Eigen::Isometry3d map_from_target = Eigen::Isometry3d::Identity();
      IMUST anchor;
      Eigen::Isometry3d map_from_query = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d odom_from_query = Eigen::Isometry3d::Identity();
      deque<DualScan> fresh;
    } pending;
    auto fuse = [](const deque<DualScan> &scans) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      const auto &anchor = scans.back().state;
      for(const auto &scan : scans)
      {
        const Eigen::Matrix3d R = anchor.R.transpose() * scan.state.R;
        const Eigen::Vector3d t = anchor.R.transpose() * (scan.state.p - anchor.p);
        for(const auto &point : *scan.points)
        {
          const Eigen::Vector3d p = R * point.pnt + t;
          pcl::PointXYZI out;
          out.x = p.x(); out.y = p.y(); out.z = p.z(); out.intensity = 0;
          cloud->push_back(out);
        }
      }
      return cloud;
    };
    auto audit_log = [](const char *stage, double stamp, const GlobalMatchAudit &a) {
      ROS_INFO_STREAM("DUAL_TF audit " << stage << " stamp=" << std::fixed << stamp
                      << " matches=" << a.matches << " overlap=" << a.overlap
                      << " rmse=" << a.rmse << " p95=" << a.p95
                      << " normal_eigen=" << a.normal_eigenvalue
                      << " spatial_eigen=" << a.spatial_eigenvalue << " pass=" << a.passed);
    };
    while(n.ok() && !is_finish.load())
    {
      deque<DualScan> incoming, latest_incoming;
      {
        lock_guard<mutex> lock(mtx_loop);
        incoming.swap(dual_scans);
        latest_incoming.swap(dual_latest_scans);
      }
      if(!frontend_session_epoch.is_current(epoch))
      {
        query_pending = false; pending = PendingAudit();
        latest.clear(); direct_schedule.reset(); direct_failures = 0; recovery_active = false;
        sc_since_query = 0;
        epoch = frontend_session_epoch.capture();
      }
      const auto current_revision = dual_alignment.view(dual_sensor_time.load()).manual_revision;
      if(current_revision != manual_revision) {
        pending = PendingAudit();
        direct_schedule.reset(); direct_failures = 0; recovery_active = false;
        previous_was_direct = false; last_query_wall = -1;
        manual_revision = current_revision;
      }
      if(incoming.empty() && latest_incoming.empty() && !query_pending && !pending.active)
      { ros::Duration(0.005).sleep(); continue; }
      for(const auto &scan : latest_incoming)
      {
        if(!frontend_session_epoch.is_current(scan.epoch)) continue;
        if(pending.active && scan.state.t > pending.anchor.t &&
           (pending.fresh.empty() || scan.state.t > pending.fresh.back().state.t))
          pending.fresh.push_back(scan);
        latest.push_back(scan);
        while(latest.size() > 30U) latest.pop_front();
        if(use_sc && ++sc_since_query >= scancontext_options.query_stride_scans &&
           latest.size() >= std::size_t(scancontext_options.query_scans)) query_pending = true;
      }

      if(pending.active)
      {
        const double now = dual_sensor_time.load();
        if(now - pending.anchor.t > dual_options.max_result_age_sec)
        {
          ROS_WARN("DUAL_TF audit candidate expired without enough fresh evidence");
          if(pending.direct) ++direct_failures;
          pending = PendingAudit();
        }
        else if(pending.fresh.size() >= std::size_t(dual_options.audit_scans) &&
                pending.fresh.back().state.t >= pending.anchor.t + dual_options.audit_min_delay_sec)
        {
          while(pending.fresh.size() > std::size_t(dual_options.audit_scans)) pending.fresh.pop_front();
          STDescManager audit_query(config);
          auto fresh_cloud = fuse(pending.fresh);
          if(pending.direct)
            fresh_cloud->erase(std::remove_if(fresh_cloud->begin(), fresh_cloud->end(), [&](const auto &p) {
              return !p.getVector3fMap().allFinite() || p.getVector3fMap().norm() > direct_options.query_max_range_m;
            }), fresh_cloud->end());
          const auto fresh_planes = audit_query.ExtractPlanes(fresh_cloud);
          bool committed = false;
          bool awaiting_confirmation = false;
          if(pending.target && !fresh_planes->empty())
          {
            const auto &map_from_history = pending.map_from_target;
            Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
            local.linear() = pending.fresh.back().state.R;
            local.translation() = pending.fresh.back().state.p;
            const auto candidate = pending.map_from_query * pending.odom_from_query.inverse();
            const Eigen::Isometry3d candidate_pose = candidate * local;
            const auto target = pending.target;
            const auto &planes = *fresh_planes;
            const auto quality = audit_global_match(planes, *target,
                map_from_history.inverse() * candidate_pose, dual_options);
            audit_log("fresh", pending.fresh.back().state.t, quality);
            bool approved = quality.passed;
            const auto incumbent = dual_alignment.view(now);
            if(approved && incumbent.aligned)
            {
              const Eigen::Isometry3d prior_pose = incumbent.map_from_odom * local;
              const auto prior_quality = audit_global_match(planes, *target,
                  map_from_history.inverse() * prior_pose, dual_options);
              audit_log("incumbent", pending.fresh.back().state.t, prior_quality);
              const double delta = (candidate_pose.translation() - prior_pose.translation()).norm();
              const double angle = Eigen::AngleAxisd(prior_pose.linear().transpose() * candidate_pose.linear()).angle() * 180 / M_PI;
              approved = audit_improves_prior(quality, prior_quality, delta, angle, dual_options);
              if(!approved) ROS_WARN_STREAM("DUAL_TF audit rejected: no clear improvement over incumbent, displacement=" << delta);
            }
            if(approved)
            {
              if(dual_alignment.accept(epoch, pending.anchor.t, now, pending.map_from_query,
                                       pending.odom_from_query, error, pending.fresh.back().state.t))
              {
                committed = true;
                direct_failures = 0; recovery_active = false;
                ROS_INFO_STREAM("DUAL_TF accepted " << (pending.direct ? "DIRECT" : pending.scancontext ? "SC_QN" : "BTC") << " correction: anchor=" << std::fixed << pending.anchor.t
                                << " history=" << pending.history_id << " score=" << pending.score
                                << " fresh_evidence=" << pending.fresh.back().state.t);
              }
              else
              {
                awaiting_confirmation = dual_alignment.view(now).confirmation_pending;
                ROS_WARN_STREAM("DUAL_TF rejected " << (pending.direct ? "DIRECT" : pending.scancontext ? "SC_QN" : "BTC") << " correction: " << error);
              }
            }
          }
          if(pending.direct && !committed && !awaiting_confirmation) ++direct_failures;
          pending = PendingAudit();
        }
        if(pending.active) { ros::Duration(0.005).sleep(); continue; }
      }
      const double wall = steady_seconds();
      const auto alignment = dual_alignment.view(dual_sensor_time.load());
      if(direct_options.enabled && alignment.aligned &&
         (direct_failures >= direct_options.recovery_after_failures ||
          alignment.age >= direct_options.recovery_after_sec))
      {
        if(!recovery_active) ROS_WARN_STREAM("DUAL_TF entering " << recovery_name << " recovery: direct_failures="
            << direct_failures << " correction_age=" << alignment.age);
        recovery_active = true;
      }
      const bool btc_due = query_pending && latest.size() >= std::size_t(scancontext_options.query_scans) &&
          wall - last_query_wall >= dual_options.query_period_sec &&
          (!direct_options.enabled || !alignment.aligned || recovery_active);
      Eigen::Isometry3d direct_local = Eigen::Isometry3d::Identity();
      if(!latest.empty()) { direct_local.linear() = latest.back().state.R; direct_local.translation() = latest.back().state.p; }
      const bool direct_due = direct_options.enabled && alignment.aligned &&
          latest.size() >= std::size_t(direct_options.query_scans) &&
          direct_schedule.ready(wall, latest.back().state.t, direct_local, alignment.confirmation_pending);
      if(direct_due && !(btc_due && previous_was_direct))
      {
        previous_was_direct = true;
        const auto started = steady_seconds();
        direct_schedule.submitted(wall, latest.back().state.t, direct_local);
        deque<DualScan> scans(latest.end() - direct_options.query_scans, latest.end());
        const IMUST direct_anchor = scans.back().state;
        auto cloud = fuse(scans);
        cloud->erase(std::remove_if(cloud->begin(), cloud->end(), [&](const auto &p) {
          return !p.getVector3fMap().allFinite() || p.getVector3fMap().norm() > direct_options.query_max_range_m;
        }), cloud->end());
        cloud=downsample_localization_cloud(cloud,direct_options.query_leaf_size_m);
        if(cloud->size() < std::size_t(direct_options.min_query_points))
        {
          ROS_WARN_STREAM("DIRECT_MATCH insufficient query points stamp=" << std::fixed << direct_anchor.t);
          continue;
        }
        const Eigen::Isometry3d predicted = alignment.map_from_odom * direct_local;
        const auto target = direct_map->target(*cloud, predicted, error);
        if(!target.planes)
        {
          ROS_WARN_STREAM("DIRECT_MATCH map unavailable: " << error);
          continue;
        }
        STDescManager extractor(config);
        auto planes = extractor.ExtractPlanes(cloud);
        ROS_INFO_STREAM("DIRECT_MATCH query stamp=" << std::fixed << direct_anchor.t
            << " cache_resident_bytes=" << scan_map_store->resident_bytes()
            << " cache_entries=" << scan_map_store->cache_entries()
            << " cache_reads=" << scan_map_store->disk_reads()
            << " cache_evictions=" << scan_map_store->evictions()
            << " backend=" << direct_options.registration_backend
            << " scans=" << scans.size() << " points=" << cloud->size() << " planes=" << planes->size()
            << " target_points=" << (target.points ? target.points->size() : 0)
            << " target_planes=" << target.planes->size() << " keyframes=" << target.keyframes
            << " target_voxels=" << target.voxels << " cache_hit=" << target.cache_hit);
        if(planes->size() < std::size_t(dual_options.audit_min_matches))
        {
          ROS_WARN("DIRECT_MATCH insufficient query plane geometry");
          continue;
        }
        const Eigen::Isometry3d initial = target.map_from_target.inverse() * predicted;
        const auto initial_quality = audit_global_match(*planes, *target.planes, initial, dual_options);
        audit_log("direct_initial", direct_anchor.t, initial_quality);
        Eigen::Isometry3d refined;
        const bool registered = direct_options.registration_backend == "nano_gicp"
            ? (target.points && register_direct_gicp(*cloud, *target.points, initial, direct_options, refined))
            : register_direct_scan(*cloud, *target.planes, initial, direct_options, refined);
        if(!registered)
        {
          ++direct_failures;
          ROS_WARN("DIRECT_MATCH ICP failed");
          continue;
        }
        const auto quality = audit_global_match(*planes, *target.planes, refined, dual_options);
        audit_log("direct_final_icp", direct_anchor.t, quality);
        if(!quality.passed) { ++direct_failures; continue; }
        const Eigen::Isometry3d map_pose = target.map_from_target * refined;
        const Eigen::Isometry3d correction = map_pose * direct_local.inverse();
        if(!correction.matrix().allFinite() || correction.translation().norm() > alignment_max_translation_m ||
           Eigen::AngleAxisd(correction.linear()).angle() * 180 / M_PI > alignment_max_rotation_deg)
        { ++direct_failures; ROS_WARN("DIRECT_MATCH exceeds absolute alignment limits"); continue; }
        pending.active = true; pending.direct = true; pending.anchor = direct_anchor;
        pending.map_from_query = map_pose; pending.odom_from_query = direct_local;
        pending.target = target.planes; pending.map_from_target = target.map_from_target;
        pending.score = quality.overlap;
        ROS_INFO_STREAM("DUAL_TF staged DIRECT candidate: anchor=" << std::fixed << direct_anchor.t
                        << " compute_ms=" << (steady_seconds() - started) * 1000);
        continue;
      }
      if(!btc_due) { ros::Duration(0.005).sleep(); continue; }
      previous_was_direct = false;
      if(latest.size() < std::size_t(scancontext_options.query_scans) || !query_pending) continue;
      if(steady_seconds() - last_query_wall < dual_options.query_period_sec)
      {
        ros::Duration(0.005).sleep();
        continue;
      }
      query_pending = false;
      last_query_wall = steady_seconds();
      if(use_sc)
      {
        sc_since_query = 0;
        const auto started = steady_seconds();
        deque<DualScan> scans(latest.end() - scancontext_options.query_scans, latest.end());
        const IMUST anchor = scans.back().state;
        auto cloud = fuse(scans);
        Eigen::Isometry3d odom_from_query = Eigen::Isometry3d::Identity();
        odom_from_query.linear()=anchor.R; odom_from_query.translation()=anchor.p;
        const Eigen::Vector3d predicted_position =
            (alignment.map_from_odom * odom_from_query).translation();
        const auto result = sc_relocalizer->match(*cloud, anchor.R,
            alignment.aligned ? &predicted_position : nullptr);
        ROS_INFO_STREAM("SC_QN query " << query_id++ << " scans=" << scans.size()
            << " anchor=" << std::fixed << anchor.t << " candidate=" << result.keyframe_id
            << " distance=" << result.distance
            << " candidate_position_distance_m=" << result.candidate_position_distance_m << " descriptor_ms=" << result.descriptor_ms
            << " search_ms=" << result.search_ms << " quatro_ms=" << result.quatro_ms
            << " gicp_ms=" << result.gicp_ms << " target_ms=" << result.target_ms
            << " compute_ms=" << (steady_seconds()-started)*1000 << " reason=" << result.reason);
        if(!result.success || !frontend_session_epoch.is_current(epoch)) continue;
        const auto correction = result.map_from_query * odom_from_query.inverse();
        if(!correction.matrix().allFinite() ||
           correction.translation().norm()>alignment_max_translation_m ||
           Eigen::AngleAxisd(correction.linear()).angle()*180.0/M_PI>alignment_max_rotation_deg) continue;
        if(dual_alignment.accept_scancontext(epoch, anchor.t, dual_sensor_time.load(),
                                             result.map_from_query, odom_from_query, error)) {
          direct_failures=0; recovery_active=false;
          ROS_INFO_STREAM("DUAL_TF accepted SC_QN correction: anchor=" << std::fixed << anchor.t
              << " history=" << result.keyframe_id << " score=" << 1.0-result.distance);
        } else {
          ROS_WARN_STREAM("DUAL_TF rejected SC_QN correction: " << error);
        }
        continue;
      }

    }
  }

  // Build the pose graph in loop closure
  void build_graph(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, int cur_id, PGO_Edges &lp_edges, gtsam::noiseModel::Diagonal::shared_ptr default_noise, vector<int> &ids, vector<int> &stepsizes, int lpedge_enable)
  {
    initial.clear(); graph = gtsam::NonlinearFactorGraph();
    ids.clear();
    lp_edges.connect(cur_id, ids);

    stepsizes.clear(); stepsizes.push_back(0);
    for(int i=0; i<ids.size(); i++)
      stepsizes.push_back(stepsizes.back() + multimap_scanPoses[ids[i]]->size());
    
    for(int ii=0; ii<ids.size(); ii++)
    {
      int bsize = stepsizes[ii], id = ids[ii];
      for(int j=bsize; j<stepsizes[ii+1]; j++)
      {
        IMUST &xc = multimap_scanPoses[id]->at(j-bsize)->x;
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        initial.insert(j, pose3);
        if(j > bsize)
        {
          gtsam::Vector samv6(6);
          samv6 = multimap_scanPoses[ids[ii]]->at(j-1-bsize)->v6;
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
          add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, v6_noise);
          // add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, default_noise);
        }
      }
    }

    if(multimap_scanPoses[ids[0]]->size() != 0)
    {
      int ceil = multimap_scanPoses[ids[0]]->size();
      // if(ceil > 10) ceil = 10;
      ceil = 1;
      for(int i=0; i<ceil; i++)
      {
        Eigen::Matrix<double, 6, 1> v6_fixd;
        v6_fixd << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9;
        gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
        IMUST xf = multimap_scanPoses[ids[0]]->at(i)->x;
        gtsam::Pose3 pose3 = gtsam::Pose3(gtsam::Rot3(xf.R), gtsam::Point3(xf.p));
        graph.addPrior(i, pose3, fixd_noise);
      }
    }

    if(lpedge_enable == 1)
    for(PGO_Edge &edge: lp_edges.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, default_noise);
        }
      }
    }
    
  }

  // The main thread of loop clousre
  // The topDownProcess of HBA is also run here
  void thd_loop_closure(ros::NodeHandle &n)
  {
    pl_kdmap.reset(new pcl::PointCloud<PointType>);
    vector<STDescManager*> std_managers;
    PGO_Edges lp_edges;


    double jud_default = 0.45, icp_eigval = 14;
    double ratio_drift = 0.05;
    int curr_halt = 10, prev_halt = 30;
    int isHighFly = 0;
    n.param<double>("Loop/jud_default", jud_default, 0.45);
    n.param<double>("Loop/icp_eigval", icp_eigval, 14);
    n.param<double>("Loop/ratio_drift", ratio_drift, 0.05);
    n.param<int>("Loop/curr_halt", curr_halt, 10);
    n.param<int>("Loop/prev_halt", prev_halt, 30);
    n.param<int>("Loop/isHighFly", isHighFly, 0);
    ConfigSetting config_setting;
    read_parameters(n, config_setting, isHighFly);
    n.param<int>("Loop/skip_near_num", config_setting.skip_near_num_, config_setting.skip_near_num_);

    if(operation_mode == "localization")
    {
      try {
        scan_map_store=std::make_shared<ScanMapStore>(localization_map_path,
            std::size_t(map_cache_mb)*1024*1024);
        localization_map=scan_map_store->map();
        descriptor_history_map=scan_map_store->history(history_scans,history_distance,history_angle);
        ROS_INFO_STREAM("Loaded SC scan map: scans=" << localization_map.keyframes.size()
                        << " history_anchors=" << descriptor_history_map.keyframes.size()
                        << " history_window=" << history_scans << " cache_mb=" << map_cache_mb);
        if(scan_map_store->leaf()>scancontext_options.registration_leaf_size_m ||
           scan_map_store->leaf()>direct_options.query_leaf_size_m)
          ROS_WARN("Saved map is coarser than requested registration sampling.");
        if(localization_publish_map_cloud) {
          // Display files are already in map coordinates; never reconstruct or
          // downsample the registration map just to visualize it.
          const auto display_path = localization_map_cloud_path.empty()
              ? std::filesystem::path(localization_map_path) / "merged_0.2m.pcd"
              : std::filesystem::path(localization_map_cloud_path);
          try {
            pcl::PCLPointCloud2 display;
            if(pcl::io::loadPCDFile(display_path.string(), display) < 0 ||
               display.width == 0 || display.height == 0)
              throw std::runtime_error("missing, unreadable or empty PCD");
            sensor_msgs::PointCloud2 message;
            pcl_conversions::moveFromPCL(display, message);
            message.header.frame_id = map_frame;
            message.header.stamp = ros::Time::now();
            localization_map_cloud_pub=n.advertise<sensor_msgs::PointCloud2>("/localization/map_cloud",1,true);
            localization_map_cloud_pub.publish(message);
            ROS_INFO_STREAM("Published display PCD: " << display_path.string()
                << " points=" << std::uint64_t(message.width)*message.height);
          } catch(const std::exception &e) {
            ROS_WARN_STREAM("Display PCD unavailable: " << display_path.string()
                << "; continuing localization without map display: " << e.what());
          }
        }
        prior_map_ready.store(true);
        ROS_INFO_STREAM("Loaded localization map package with " << localization_map.keyframes.size()
                        << " scans; SC-only localization.");
        thd_dual_tf_sc(n,config_setting);
      } catch(const std::exception &e) {
        ROS_FATAL_STREAM("SC scan-map localization failed: " << e.what());
        ros::shutdown();
      }
      return;
    }

    vector<double> juds;
    FileReaderWriter::instance().previous_map_names(n, sessionNames, juds);
    FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 0, savepath, bagname);
    FileReaderWriter::instance().previous_map_read(std_managers, multimap_scanPoses, multimap_keyframes, config_setting, lp_edges, n, sessionNames, juds, savepath, win_size);
    
    STDescManager *std_manager = new STDescManager(config_setting);
    sessionNames.push_back(bagname);
    std_managers.push_back(std_manager);
    multimap_scanPoses.push_back(scanPoses);
    multimap_keyframes.push_back(keyframes);
    juds.push_back(jud_default);
    vector<double> jours(std_managers.size(), 0);

    vector<int> relc_counts(std_managers.size(), prev_halt);
    
    deque<ScanPose*> bl_local;
    Eigen::Matrix<double, 6, 1> v6_init, v6_fixd;
    v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    v6_fixd << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    vector<int> ids(1, std_managers.size() - 1), stepsizes(2, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plbtc(new pcl::PointCloud<pcl::PointXYZI>);
    IMUST x_key;
    int buf_base = 0;

    while(n.ok())
    {
      if(reset_flag.exchange(0) == 1)
      {
        scanPoses->insert(scanPoses->end(), buf_lba2loop_tem.begin(), buf_lba2loop_tem.end());
        for(ScanPose *bl: buf_lba2loop_tem) bl->pvec = nullptr;
        buf_lba2loop_tem.clear();

        keyframes = new vector<Keyframe*>();
        multimap_keyframes.push_back(keyframes);
        scanPoses = new vector<ScanPose*>();
        multimap_scanPoses.push_back(scanPoses);

        bl_local.clear(); buf_base = 0; 
        std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);
        std_manager = new STDescManager(config_setting);
        std_managers.push_back(std_manager);
        relc_counts.push_back(prev_halt);
        sessionNames.push_back(bagname + to_string(sessionNames.size()));
        juds.push_back(jud_default);
        jours.push_back(0);

        bagname = sessionNames.back();
        string cmd = "mkdir " + savepath + bagname + "/";
        int ss = system(cmd.c_str());

        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids, pub_pmap);

        initial.clear(); graph = gtsam::NonlinearFactorGraph();
        ids.clear(); ids.push_back(std_managers.size()-1); 
        stepsizes.clear(); stepsizes.push_back(0); stepsizes.push_back(0);
      }

      if(is_finish.load() && buf_lba2loop.empty())
      {
        break;
      }

      if(buf_lba2loop.empty() || loop_detect == 1)
      {
        sleep(0.01); continue;
      }
      ScanPose *bl_head = nullptr;
      mtx_loop.lock();
      if(!buf_lba2loop.empty()) 
      {
        bl_head = buf_lba2loop.front();
        buf_lba2loop.pop_front();
      }
      mtx_loop.unlock();
      if(bl_head == nullptr) continue;

      int cur_id = std_managers.size() - 1;
      scanPoses->push_back(bl_head);
      bl_local.push_back(bl_head);
      IMUST xc = bl_head->x;
      gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
      int g_pos = stepsizes.back();
      initial.insert(g_pos, pose3);

      if(g_pos > 0)
      {
        gtsam::Vector samv6(scanPoses->at(buf_base-1)->v6);
        gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
        add_edge(g_pos-1, g_pos, scanPoses->at(buf_base-1)->x, xc, graph, v6_noise);
      }
      else
      {
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        graph.addPrior(0, pose3, fixd_noise);
      }

      if(buf_base == 0) x_key = xc;
      buf_base++; stepsizes.back() += 1;

      if(bl_local.size() < win_size) continue;
      double ang = Log(x_key.R.transpose() * xc.R).norm() * 57.3;
      double len = (xc.p - x_key.p).norm();
      if(ang < 5 && len < 0.1 && buf_base > win_size)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
        continue;
      }
      for(double &jour: jours)
        jour += len;
      x_key = xc;

      PVecPtr pptr(new PVec);
      for(int i=0; i<win_size; i++)
      {
        ScanPose &bl = *bl_local[i];
        Eigen::Vector3d delta_p = xc.R.transpose() * (bl.x.p - xc.p);
        Eigen::Matrix3d delta_R = xc.R.transpose() *  bl.x.R;
        for(pointVar pv: *(bl.pvec))
        {
          pv.pnt = delta_R * pv.pnt + delta_p;
          pptr->push_back(pv);
        }
      }
      for(int i=0; i<win_size; i++)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
      }

      Keyframe *smp = new Keyframe(xc);
      smp->id = buf_base - 1;
      smp->jour = jours[cur_id];
      down_sampling_pvec(*pptr, voxel_size/10, *(smp->plptr));

      plbtc->clear();
      pcl::PointXYZI ap;
      for(pointVar &pv: *pptr)
      {
        Eigen::Vector3d &wld = pv.pnt;
        ap.x = wld[0]; ap.y = wld[1]; ap.z = wld[2];
        plbtc->push_back(ap);
      }
      mtx_keyframe.lock();
      keyframes->push_back(smp);
      mtx_keyframe.unlock();

      vector<STD> stds_vec;
      std_manager->GenerateSTDescs(plbtc, stds_vec, buf_base-1);
      pair<int, double> search_result(-1, 0);
      pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      vector<pair<STD, STD>> loop_std_pair;

      bool isGraph = false, isOpt = false;
      int match_num = 0;
      for(int id=0; id<=cur_id; id++)
      {
        std_managers[id]->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair, std_manager->plane_cloud_vec_.back());

        if(search_result.first >= 0)
        {
          printf("Find Loop in session%d: %d %d\n", id, buf_base, search_result.first);
          printf("score: %lf\n", search_result.second);
        }

        if(search_result.first >= 0 && search_result.second > juds[id])
        {
          if(icp_normal(*(std_manager->plane_cloud_vec_.back()), *(std_managers[id]->plane_cloud_vec_[search_result.first]), loop_transform, icp_eigval))
          {
            int ord_bl = std_managers[id]->plane_cloud_vec_[search_result.first]->header.seq;

            IMUST &xx = multimap_scanPoses[id]->at(ord_bl)->x;
            double drift_p = (xx.R * loop_transform.first + xx.p - xc.p).norm();

            bool isPush = false;
            int step = -1;
            if(id == cur_id)
            {
              double span = smp->jour - keyframes->at(search_result.first)->jour;
              printf("drift: %lf %lf\n", drift_p, span);

              if(drift_p / span < ratio_drift)
              {
                isPush = true;
                step = stepsizes.size() - 2;

                if(relc_counts[id] > curr_halt && drift_p > 0.10)
                {
                  isOpt = true;
                  for(int &cnt: relc_counts) cnt = 0;
                }
              }
            }
            else
            {
              for(int i=0; i<ids.size(); i++)
                if(id == ids[i]) 
                  step = i;
              
              printf("drift: %lf %lf\n", drift_p, jours[id]);

              if(step == -1)
              {
                isGraph = true;
                isOpt = true;
                relc_counts[id] = 0;
                g_update = 1;
                isPush = true;
                jours[id] = 0;
              }
              else
              {
                if(drift_p / jours[id] < 0.05)
                {
                  jours[id] = 1e-6; // set to 0
                  isPush = true;
                  if(relc_counts[id] > prev_halt && drift_p > 0.25)
                  {
                    isOpt = true;
                    for(int &cnt: relc_counts) cnt = 0;
                  }
                }
              }

            }

            if(isPush)
            {
              match_num++;
              lp_edges.push(id, cur_id, ord_bl, buf_base-1, loop_transform.second, loop_transform.first, v6_init);
              if(step > -1)
              {
                int id1 = stepsizes[step] + ord_bl;
                int id2 = stepsizes.back() - 1;
                add_edge(id1, id2, loop_transform.second, loop_transform.first, graph, odom_noise);
                printf("addedge: (%d %d) (%d %d)\n", id, cur_id, ord_bl, buf_base-1);
              }
            }

            // if(isPush)
            // {
            //   icp_check(*(smp->plptr), *(std_managers[id]->plane_cloud_vec_[search_result.first]), pub_test, pub_init, loop_transform, multimap_scanPoses[id]->at(ord_bl)->x);
            // }

          }
        }
        
      }
      for(int &it: relc_counts) it++;
      std_manager->AddSTDescs(stds_vec);
  
      if(isGraph)
      {
        build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 1);
      }

      if(isOpt)
      {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);
        isam.update(graph, initial);

        for(int i=0; i<5; i++) isam.update();
        gtsam::Values results = isam.calculateEstimate();
        int resultsize = results.size();
        
        IMUST x1 = scanPoses->at(buf_base-1)->x;
        int idsize = ids.size();

        history_kfsize = 0;
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
          {
            int ord = j - stepsizes[ii];
            multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
          }
        }
        mtx_keyframe.lock();
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(Keyframe *kf: *multimap_keyframes[tip])
            kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
        }
        mtx_keyframe.unlock();

        initial.clear();
        for(int i=0; i<resultsize; i++)
          initial.insert(i, results.at(i).cast<gtsam::Pose3>());
        
        IMUST x3 = scanPoses->at(buf_base-1)->x;
        dx.p = x3.p - x3.R * x1.R.transpose() * x1.p;
        dx.R = x3.R * x1.R.transpose();
        x_key = x3;

        PVec pvec_tem;
        int subsize = keyframes->size();
        int init_num = 5;
        for(int i=subsize-init_num; i<subsize; i++)
        {
          if(i < 0) continue;
          Keyframe &sp = *(keyframes->at(i));
          sp.exist = 0;
          pvec_tem.reserve(sp.plptr->size());
          pointVar pv; pv.var.setZero();
          for(PointType &ap: sp.plptr->points)
          {
            pv.pnt << ap.x, ap.y, ap.z;
            pv.pnt = sp.x0.R * pv.pnt + sp.x0.p;
            for(int j=0; j<3; j++)
              pv.var(j, j) = ap.normal[j];
            pvec_tem.push_back(pv);
          }
          cut_voxel(map_loop, pvec_tem, win_size, 0);
        }

        if(subsize > init_num)
        {
          pl_kdmap->clear();
          for(int i=0; i<subsize-init_num; i++)
          {
            Keyframe &kf = *(keyframes->at(i));
            kf.exist = 1;
            PointType pp;
            pp.x = kf.x0.p[0]; pp.y = kf.x0.p[1]; pp.z = kf.x0.p[2];
            pp.intensity = cur_id; pp.curvature = i;
            pl_kdmap->push_back(pp);
          }

          kd_keyframes.setInputCloud(pl_kdmap);
          history_kfsize = pl_kdmap->size();
        }
        loop_detect = 1;

        vector<int> ids2 = ids; ids2.pop_back();
        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids2);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
        ids2.clear(); ids2.push_back(ids.back());
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);

      }

    }

    for(int i=0; i<std_managers.size(); i++)
      delete std_managers[i];
    malloc_trim(0);

    if(is_finish.load())
    {
      // The background HBA worker can still reference keyframes. Drain it before
      // exporting current poses or freeing any map/session data.
      if(finish_without_ba.load()) {
        while(!gba_worker_stopped.load()) ros::Duration(0.005).sleep();
        ROS_INFO("finish_no_ba: skipping final global BA; saving current optimized poses.");
      }
      if(keyframes->empty())
      {
        sessionNames.pop_back();
        std_managers.pop_back();
        multimap_scanPoses.pop_back();
        multimap_keyframes.pop_back();
        juds.pop_back();
        jours.pop_back();
        relc_counts.pop_back();
      }

      if(multimap_keyframes.empty()) 
      {
        printf("no data\n"); return;
      }

      if(!finish_without_ba.load()) {
        int cur_id = std_managers.size() - 1;
        build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 0);
      }

      if(scan_map_writer && !scan_export_failed)
        for(const auto &link : scan_pose_links) scan_map_writer->set_pose(link.first,saved_pose(link.second->x));
      if(!finish_without_ba.load()) topDownProcess(initial, graph, ids, stepsizes);
    }

    if(is_finish.load() && scan_map_writer) {
      try {
        if(scan_export_failed) throw std::runtime_error("scan capture failed");
        vector<pair<int,SavedPose>> final;
        for(const auto &link : scan_pose_links) final.emplace_back(link.first,saved_pose(link.second->x));
        scan_map_writer->finish(final);
        ROS_INFO_STREAM("Saved optimized SC scan map to " << map_export_path
                        << " with " << scan_export_ids.size() << " scans; final_global_ba="
                        << (finish_without_ba.load() ? "skipped" : "enabled") << ".");
      } catch(const std::exception &e) {
        ROS_ERROR_STREAM("SC scan-map export failed; incomplete map is not usable: " << e.what());
      }
    }

    if(is_save_map)
    {
      for(int i=0; i<ids.size(); i++)
        FileReaderWriter::instance().save_pose(*(multimap_scanPoses[ids[i]]), sessionNames[ids[i]], "/alidarState.txt", savepath);

      FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 1, savepath, bagname);
    }

    for(int i=0; i<multimap_scanPoses.size(); i++)
    {
      for(int j=0; j<multimap_scanPoses[i]->size(); j++)
        delete multimap_scanPoses[i]->at(j);
    }
    for(int i=0; i<multimap_keyframes.size(); i++)
    {
      for(int j=0; j<multimap_keyframes[i]->size(); j++)
        delete multimap_keyframes[i]->at(j);
    }
    
    malloc_trim(0);
  }

  // The top down process of HBA
  void topDownProcess(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, vector<int> &ids, vector<int> &stepsizes)
  {
    cnct_map = ids;
    gba_size = multimap_keyframes.back()->size();
    gba_flag = 1;

    pcl::PointCloud<PointType> pl0;
    pub_pl_func(pl0, pub_pmap);
    pub_pl_func(pl0, pub_cmap);
    pub_pl_func(pl0, pub_curr_path);
    pub_pl_func(pl0, pub_prev_path);
    pub_pl_func(pl0, pub_scan);

    double t0 = ros::Time::now().toSec();
    while(gba_flag);
    
    for(PGO_Edge &edge: gba_edges1.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    for(PGO_Edge &edge: gba_edges2.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    for(int i=0; i<5; i++) isam.update();
    gtsam::Values results = isam.calculateEstimate();
    int resultsize = results.size();

    int idsize = ids.size();
    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
      {
        int ord = j - stepsizes[ii];
        multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
      }
    }

    Eigen::Quaterniond qq(multimap_scanPoses[0]->at(0)->x.R);

    double t1 = ros::Time::now().toSec();
    printf("GBA opt: %lfs\n", t1 - t0);

    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(Keyframe *smp: *multimap_keyframes[tip])
        smp->x0 = multimap_scanPoses[tip]->at(smp->id)->x;
    }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
    vector<int> ids2 = ids; ids2.pop_back();
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
    ids2.clear(); ids2.push_back(ids.back());
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
  }

  // The bottom up to add edge in HBA
  void HBA_add_edge(vector<IMUST> &p_xs, vector<Keyframe*> &p_smps, PGO_Edges &gba_edges, vector<int> &maps, int max_iter, int thread_num, pcl::PointCloud<PointType>::Ptr plptr = nullptr)
  {
    bool is_display = false;
    if(plptr == nullptr) is_display = true;

    double t0 = ros::Time::now().toSec();
    vector<Keyframe*> smps;
    vector<IMUST> xs;
    int last_mp = -1, isCnct = 0;
    for(int i=0; i<p_smps.size(); i++)
    {
      Keyframe *smp = p_smps[i];
      if(smp->mp != last_mp)
      {
        isCnct = 0;
        for(int &m: maps)
        if(smp->mp == m)
        {
          isCnct = 1; break;
        }
        last_mp = smp->mp;
      }

      if(isCnct)
      {
        smps.push_back(smp);
        xs.push_back(p_xs[i]);
      }
    }
    
    int wdsize = smps.size();
    Eigen::MatrixXd hess;
    vector<double> gba_eigen_value_array_orig = gba_eigen_value_array;
    double gba_min_eigen_value_orig = gba_min_eigen_value;
    double gba_voxel_size_orig = gba_voxel_size;

    int up = 4;
    int converge_flag = 0;
    double converge_thre = 0.05;

    for(int iterCnt = 0; iterCnt < max_iter; iterCnt++)
    {
      if(converge_flag == 1 || iterCnt == max_iter-1)
      {
        // if(plptr == nullptr)
        // {
        //   break;
        // }

        gba_voxel_size = voxel_size;
        gba_eigen_value_array = plane_eigen_value_thre;
        gba_min_eigen_value = min_eigen_value;
      }

      unordered_map<VOXEL_LOC, OctreeGBA*> oct_map;
      for(int i=0; i<wdsize; i++)
        OctreeGBA::cut_voxel(oct_map, xs[i], smps[i]->plptr, i, wdsize);

      LidarFactor voxhess(wdsize);
      OctreeGBA_multi_recut(oct_map, voxhess, thread_num);

      Lidar_BA_Optimizer opt_lsv;
      opt_lsv.thd_num = thread_num;
      vector<double> resis;
      bool is_converge = opt_lsv.damping_iter(xs, voxhess, &hess, resis, up, is_display);
      if(is_display)
        printf("%lf\n", fabs(resis[0] - resis[1]) / resis[0]);
      if((fabs(resis[0] - resis[1]) / resis[0] < converge_thre && is_converge) || (iterCnt == max_iter-2 && converge_flag == 0))
      {
        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          converge_flag = 1;
        }
        else if(converge_flag == 1)
        {
          break;
        }
      }
    }

    gba_eigen_value_array = gba_eigen_value_array_orig;
    gba_min_eigen_value = gba_min_eigen_value_orig;
    gba_voxel_size = gba_voxel_size_orig;

    for(int i=0; i<wdsize - 1; i++)
    for(int j=i+1; j<wdsize; j++)
    {
      bool isAdd = true;
      Eigen::Matrix<double, 6, 1> v6;
      for(int k=0; k<6; k++)
      {
        double hc = fabs(hess(6*i+k, 6*j+k));
        if(hc < 1e-6) // 1e-6
        {
          isAdd = false; break;
        }
        v6[k] = 1.0 / hc;
      }

      if(isAdd)
      {
        Keyframe &s1 = *smps[i]; Keyframe &s2 = *smps[j];
        Eigen::Vector3d tra = xs[i].R.transpose() * (xs[j].p - xs[i].p);
        Eigen::Matrix3d rot = xs[i].R.transpose() *  xs[j].R;
        gba_edges.push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
      }
    }

    if(plptr != nullptr)
    {
      pcl::PointCloud<PointType> pl;
      IMUST xc = xs[0];
      for(int i=0; i<wdsize; i++)
      {
        Eigen::Vector3d dp = xc.R.transpose() * (xs[i].p - xc.p);
        Eigen::Matrix3d dR = xc.R.transpose() *  xs[i].R;
        for(PointType ap: smps[i]->plptr->points)
        {
          Eigen::Vector3d v3(ap.x, ap.y, ap.z);
          v3 = dR * v3 + dp;
          ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
          ap.intensity = smps[i]->mp;
          pl.push_back(ap);
        }
      }
      
      down_sampling_voxel(pl, voxel_size / 8);
      plptr->clear(); plptr->reserve(pl.size());
      for(PointType &ap: pl.points)
        plptr->push_back(ap);
    }
    else
    {
      // pcl::PointCloud<PointType> pl, path;
      // pub_pl_func(pl, pub_test);
      // for(int i=0; i<wdsize; i++)
      // {
      //   PointType pt;
      //   pt.x = xs[i].p[0]; pt.y = xs[i].p[1]; pt.z = xs[i].p[2];
      //   path.push_back(pt);
      //   for(int j=1; j<smps[i]->plptr->size(); j+=2)
      //   {
      //     PointType ap = smps[i]->plptr->points[j];
      //     Eigen::Vector3d v3(ap.x, ap.y, ap.z);
      //     v3 = xs[i].R * v3 + xs[i].p;
      //     ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
      //     ap.intensity = smps[i]->mp;
      //     pl.push_back(ap);

      //     if(pl.size() > 1e7)
      //     {
      //       pub_pl_func(pl, pub_test);
      //       pl.clear();
      //       sleep(0.05);
      //     }
      //   }
      // }
      // pub_pl_func(pl, pub_test);
      // return;
    }

  }

  // The main thread of bottom up in global mapping
  void thd_globalmapping(ros::NodeHandle &n)
  {
    // Localization consumes an immutable historical map and never creates GBA
    // submaps. Do not start the mapping-only polling loop in this mode.
    struct WorkerStopped {
      std::atomic<bool> &flag;
      ~WorkerStopped() { flag.store(true); }
    } stopped{gba_worker_stopped};
    if(operation_mode == "localization") return;
    n.param<double>("GBA/voxel_size", gba_voxel_size, 1.0);
    n.param<double>("GBA/min_eigen_value", gba_min_eigen_value, 0.01);
    n.param<vector<double>>("GBA/eigen_value_array", gba_eigen_value_array, vector<double>());
    for(double &iter: gba_eigen_value_array) iter = 1.0 / iter;
    int total_max_iter = 1;
    n.param<int>("GBA/total_max_iter", total_max_iter, 1);

    vector<Keyframe*> gba_submaps;
    deque<int> localID;

    int smp_mp = 0;
    int buf_base = 0;
    int wdsize = 10;
    int mgsize = 5;
    int thread_num = 5;

    while(n.ok() && !stop_gba_worker.load())
    {
      if(multimap_keyframes.empty())
      {
        sleep(0.1); continue;
      }

      int smp_flag = 0;
      if(smp_mp+1 < multimap_keyframes.size() && !multimap_keyframes.back()->empty())
        smp_flag = 1;

      vector<Keyframe*> &smps = *multimap_keyframes[smp_mp];
      int total_ba = 0;
      if(gba_flag == 1 && smp_mp >= cnct_map.back() && gba_size <= buf_base)
      {
        printf("gba_flag enter: %d\n", gba_flag);
        total_ba = 1;
      }
      else if(smps.size() <= buf_base)
      {
        if(smp_flag == 0)
        {
          sleep(0.1); continue;
        }
      }
      else
      {
        smps[buf_base]->mp = smp_mp;
        localID.push_back(buf_base);

        buf_base++;
        if(localID.size() < wdsize)
        {
          sleep(0.1); continue;
        }
      }

      vector<IMUST> xs;
      vector<Keyframe*> smp_local;
      mtx_keyframe.lock();
      for(int i: localID)
      {
        xs.push_back(multimap_keyframes[smp_mp]->at(i)->x0);
        smp_local.push_back(multimap_keyframes[smp_mp]->at(i));
      }
      mtx_keyframe.unlock();

      double tg1 = ros::Time::now().toSec();

      Keyframe *gba_smp = new Keyframe(smp_local[0]->x0);
      vector<int> mps{smp_mp};
      HBA_add_edge(xs, smp_local, gba_edges1, mps, 1, 2, gba_smp->plptr);
      gba_smp->id = smp_local[0]->id;
      gba_smp->mp = smp_mp;
      gba_submaps.push_back(gba_smp);

      if(total_ba == 1)
      {
        printf("GBAsize: %d\n", gba_size);
        vector<IMUST> xs;
        mtx_keyframe.lock();
        for(Keyframe *smp: gba_submaps)
        {
          xs.push_back(multimap_scanPoses[smp->mp]->at(smp->id)->x);
        }
        mtx_keyframe.unlock();
        gba_edges2.edges.clear(); gba_edges2.mates.clear();
        HBA_add_edge(xs, gba_submaps, gba_edges2, cnct_map, total_max_iter, thread_num);

        if(is_finish.load())
        {
          for(int i=0; i<gba_submaps.size(); i++)
            delete gba_submaps[i];
        }
        gba_submaps.clear();

        malloc_trim(0);
        gba_flag = 0;
      }
      else if(smp_flag == 1 && multimap_keyframes[smp_mp]->size() <= buf_base)
      {
        smp_mp++; buf_base = 0; localID.clear();
        // printf("switch: %d\n", smp_mp);
      }
      else
      {
        for(int i=0; i<mgsize; i++)
          localID.pop_front();
      }
  
    }

    for(Keyframe *submap : gba_submaps) delete submap;
    gba_submaps.clear();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmn_voxel");
  ros::NodeHandle n;

  pub_cmap = n.advertise<sensor_msgs::PointCloud2>("/map_cmap", 100);
  pub_pmap = n.advertise<sensor_msgs::PointCloud2>("/map_pmap", 100);
  pub_scan = n.advertise<sensor_msgs::PointCloud2>("/map_scan", 100);
  pub_init = n.advertise<sensor_msgs::PointCloud2>("/map_init", 100);
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
  pub_curr_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_prev_path = n.advertise<sensor_msgs::PointCloud2>("/map_true", 100);
  
  VOXEL_SLAM vs(n);
  mp = new int[vs.win_size];
  for(int i=0; i<vs.win_size; i++)
    mp[i] = i;
  
  thread thread_loop(&VOXEL_SLAM::thd_loop_closure, &vs, ref(n));
  thread thread_gba(&VOXEL_SLAM::thd_globalmapping, &vs, ref(n));
  vs.thd_odometry_localmapping(n);

  thread_loop.join();
  thread_gba.join();
  if(n.ok()) ros::spin();
  // Global ROS1-style topic handles must release DDS entities before static teardown.
  sub_pcl.shutdown();
  pub_cmap.shutdown();
  pub_pmap.shutdown();
  pub_scan.shutdown();
  pub_init.shutdown();
  pub_test.shutdown();
  pub_curr_path.shutdown();
  pub_prev_path.shutdown();
  return 0;
}
