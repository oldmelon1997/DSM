# pragma once

# include <fstream>

namespace dsm {

    class ActivePoint;
    class Frame;

    class PointCloudWriter {
    public:
        PointCloudWriter(std::string pointcloud_filename, std::string keyframes_filename);
        bool output(const std::vector<std::unique_ptr<ActivePoint>>& points, const std::shared_ptr<Frame>& frame);
        bool output_keyframes(const std::shared_ptr<Frame>& frame);

    private:
        std::ofstream p_file;
        std::ofstream k_file;
        std::string p_filename;
        std::string k_filename;

    };
}