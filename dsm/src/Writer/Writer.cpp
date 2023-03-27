# include <iostream>
# include <vector>
# include <memory>
# include "Writer.h"
# include "DataStructures/ActivePoint.h"
# include "DataStructures/Frame.h"


namespace dsm {

    PointCloudWriter::PointCloudWriter(std::string pointcloud_filename, std::string keyframes_filename) : p_filename(pointcloud_filename), k_filename(keyframes_filename) {
        // open and close to delete all the previous information
        this->p_file.open(p_filename); 
        this->p_file.close();
        this->k_file.open(k_filename);
        this->k_file.close();      
        std::cout << "writer initialized!" << std::endl;       
    }

    bool PointCloudWriter::output(const std::vector<std::unique_ptr<ActivePoint>>& points, const std::shared_ptr<Frame>& frame){
        this->p_file.open(p_filename, std::ios::app);
        if (!this->p_file) {
            std::cout << "can not open pointcloud txt" << std::endl;
            return false;
        }
        for (const auto& it : points) {
            Eigen::Vector3f pt_w = frame.get()->camToWorld() * (it.get())->pt3d();
            Eigen::Vector3f raw_color = it.get()->colors(0);
            Eigen::Vector3f color_cam = it.get()->get_rgb();
            Eigen::Vector3f base;
            base << 1.0, 1.0, 1.0;
            Eigen::Vector3f color_global = (frame.get()->affineLight().a()) * color_cam + frame.get()->affineLight().b() * base;
            Eigen::Vector3f ray = pt_w - frame.get()->camToWorld().translation();
            // take the center pixel brightness from Pattern DSO
            //float cam_brightness = it.get()->colors(0)(4);
            //unsigned int global_brightness = static_cast<unsigned int>((frame.get()->affineLight().a()) * cam_brightness + frame.get()->affineLight().b());
            
            //file << pt_w.transpose() << " " << global_brightness << " " << global_brightness << " " << global_brightness << std::endl;
            p_file << pt_w.transpose() << " " << static_cast<int>(color_global[0]) << " " << static_cast<int>(color_global[1]) << " " << static_cast<int>(color_global[2]) << " " << ray.transpose() << std::endl;
        }
        this->p_file.close();

        return true;
    }

    bool PointCloudWriter::output_keyframes(const std::shared_ptr<Frame>& frame) {
        this->k_file.open(k_filename, std::ios::app);
        if (!this->k_file) {
            std::cout << "can not open keyframes txt" << std::endl;
            return false;
        }

        k_file << frame.get()->frameID() << std::endl;
        k_file.close();
        return true;
    }
}