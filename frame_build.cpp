#include "frame_build.h"
#include <map>
#include <algorithm>

namespace proto_input
{
    std::list<std::tuple<double, FileTag, std::string> > buildSensorInputSequence(boost::filesystem::path data_dir, const char* data_name, FileTag tag)
    {
        std::list<std::tuple<double, FileTag, std::string> > data_seq;
        if (!boost::filesystem::exists(data_dir) || !boost::filesystem::is_directory(data_dir))
        {
            std::cout << data_name  << " data directory does not exist!\n";
            abort();
        }
        for (auto const& dir_entry : boost::filesystem::directory_iterator{ data_dir })
            if (!boost::filesystem::is_directory(dir_entry.path()))
                data_seq.emplace_back(std::stod(dir_entry.path().stem().string()), tag, dir_entry.path().string());
        return data_seq;
    }

    std::vector<std::tuple<double, FileTag, std::string> > frameTimeMatch(std::list<std::tuple<double, FileTag, std::string> >&& data, size_t type_num)
    {
        std::map<FileTag, std::list<std::tuple<double, FileTag, std::string> >::iterator > frame;
        std::vector<std::tuple<double, FileTag, std::string> > res;
        res.reserve(data.size());
        for (auto it = data.begin(); it != data.end(); ++it)
        {
            frame.emplace(std::get<1>(*it), it);
            if (frame.size() == type_num)
            {
                for (auto& pair : frame)
                    res.push_back(std::move(*(pair.second)));
                frame.clear();
            }
        }
        return res;
    }

    std::vector<std::tuple<double, FileTag, std::string> > mergeInputSequence(std::vector<std::list<std::tuple<double, FileTag, std::string> > >&& data)
    {
        auto comp = [](auto& l, auto& r) { return std::get<0>(l) < std::get<0>(r); };
        //size_t frame_num = std::min_element(data.begin(), data.end(), [](auto& l, auto& r) { return l.size() < r.size(); })->size();
        std::list<std::tuple<double, FileTag, std::string> > merge_list = std::move(data.front());
        merge_list.sort(comp);
        for (size_t i = 1; i < data.size(); i++)
        {
            data[i].sort(comp);
            merge_list.merge(std::move(data[i]), comp);
        }
        return frameTimeMatch(std::move(merge_list), data.size());
    }

    kit::perception::fusion::LiDARObjectPtr makeLiDARObjectPtr(LidarObject raw_lo, double time)
    {
        auto lo = std::make_shared<kit::perception::fusion::LiDARObject>();
        lo->time_ns = time;
        lo->id = raw_lo.id;
        lo->x = raw_lo.anchor_point.x;
        lo->y = raw_lo.anchor_point.y;
        lo->z = raw_lo.anchor_point.z;
        lo->length = raw_lo.length;
        lo->width = raw_lo.width;
        lo->height = raw_lo.height;
        lo->theta = raw_lo.theta;
        lo->velo_x = raw_lo.velocity.x;
        lo->velo_y = raw_lo.velocity.y;
        lo->velo_z = raw_lo.velocity.z;
        return lo;
    }
    
    kit::perception::fusion::CameraObjectPtr makeCameraObjectPtr(CameraObject raw_co, double time)
    {
        auto co = std::make_shared<kit::perception::fusion::CameraObject>();
        co->time_ns = time;
        co->id = raw_co.id;
        co->x = raw_co.anchor_point.x;
        co->y = raw_co.anchor_point.y;
        co->z = raw_co.anchor_point.z;
        co->length = raw_co.length;
        co->width = raw_co.width;
        co->height = raw_co.height;
        co->theta = raw_co.theta;
        co->velo_x = raw_co.velocity.x;
        co->velo_y = raw_co.velocity.y;
        co->velo_z = raw_co.velocity.z;
        co->ux = (raw_co.bbox2d.xmax + raw_co.bbox2d.xmin) / 2;
        co->vy = (raw_co.bbox2d.ymax + raw_co.bbox2d.ymin) / 2;
        co->width_2d = raw_co.bbox2d.xmax - raw_co.bbox2d.xmin;
        co->height_2d = raw_co.bbox2d.ymax - raw_co.bbox2d.ymin;
        return co;
    }

    Eigen::Affine3d Pose2Affine(const proto_input::Location &pose) {
        Eigen::Translation3d trans(pose.pose.position.x,
                                   pose.pose.position.y,
                                   pose.pose.position.z);
        Eigen::Quaterniond quat(pose.pose.orientation.qw,
                                pose.pose.orientation.qx,
                                pose.pose.orientation.qy,
                                pose.pose.orientation.qz);
        Eigen::Affine3d affine = trans * quat;
        return affine;
    }

    void TransformLiDAR2Baselink(
        const proto_input::Location &pose, LidarObject& raw_lo) {
        Eigen::Affine3d affine = Pose2Affine(pose);
        Eigen::Vector3d anchor(raw_lo.anchor_point.x, 
                               raw_lo.anchor_point.y,
                               raw_lo.anchor_point.z);
        anchor = affine.inverse() * anchor;
        raw_lo.anchor_point.x = anchor(0);
        raw_lo.anchor_point.y = anchor(1);
        raw_lo.anchor_point.z = anchor(2);
        Eigen::Vector3d vel(raw_lo.velocity.x,
                            raw_lo.velocity.y,
                            raw_lo.velocity.z);
        vel = affine.inverse().rotation() * vel;
        raw_lo.velocity.x = vel(0);
        raw_lo.velocity.y = vel(1);
        raw_lo.velocity.z = vel(2);

        Eigen::Vector3d eulers = affine.rotation().matrix().eulerAngles(2,1,0);
        raw_lo.theta -= eulers(0);
    }

    void TransformCamera2Baselink(
        const proto_input::Location &pose, CameraObject& raw_co) {
        Eigen::Affine3d affine = Pose2Affine(pose);
        Eigen::Vector3d anchor(raw_co.anchor_point.x, 
                               raw_co.anchor_point.y,
                               raw_co.anchor_point.z);
        anchor = affine.inverse() * anchor;
        raw_co.anchor_point.x = anchor(0);
        raw_co.anchor_point.y = anchor(1);
        raw_co.anchor_point.z = anchor(2);

        Eigen::Vector3d vel(raw_co.velocity.x,
                            raw_co.velocity.y,
                            raw_co.velocity.z);
        vel = affine.inverse().rotation() * vel;
        raw_co.velocity.x = vel(0);
        raw_co.velocity.y = vel(1);
        raw_co.velocity.z = vel(2);

        Eigen::Vector3d eulers = affine.rotation().matrix().eulerAngles(2,1,0);
        raw_co.theta -= eulers(0);
    }
}
