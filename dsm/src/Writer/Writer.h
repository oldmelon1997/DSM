# pragma once

# include <fstream>

namespace dsm {

    class ActivePoint;
    class Frame;

    class PointCloudWriter {
    public:
        PointCloudWriter(std::string file_name);
        bool output(const std::vector<std::unique_ptr<ActivePoint>>& points, const std::shared_ptr<Frame>& frame);

    private:
        int x=100;
        std::ofstream file;
        std::string filename;

    };
}