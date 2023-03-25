# include <iostream>
# include <vector>
# include <memory>
# include "Writer.h"
# include "DataStructures/ActivePoint.h"
# include "DataStructures/Frame.h"


namespace dsm {

    PointCloudWriter::PointCloudWriter(std::string filename_) : filename(filename_) {
        this->file.open(filename);       
        std::cout << "writer initialized!" << std::endl;
        this->file.close();
    }

    bool PointCloudWriter::output(const std::vector<std::unique_ptr<ActivePoint>>& points, const std::shared_ptr<Frame>& frame){
        this->file.open(filename, std::ios::app);
        if(!this->file) {
            std::cout << "can not open pointcloud txt" << std::endl;
            return false;
        }
        for (const auto& it : points) {
            Eigen::Vector3f pt_w = frame.get()->camToWorld() * (it.get())->pt3d();
            // Eigen::Vector3f raw_color = it.get()->colors(0);
            // Eigen::Matrix<unsigned int, 3, 1> colors;
            // colors(0) = static_cast<unsigned int>(raw_color(0));
            // colors(1) = static_cast<unsigned int>(raw_color(1));
            // colors(2) = static_cast<unsigned int>(raw_color(2));
            file << pt_w.transpose() << std::endl;
        }
        this->file.close();
        return true;
    }
}