#include <iostream>
#include <thread>
#include <memory>

#include "glog/logging.h"

#include "opencv2/imgproc.hpp"

#include "QtVisualizer.h"
#include "FullSystem/FullSystem.h"
#include "Utils/Undistorter.h"
#include "Utils/InteriorNetReader.h"


// Use the best GPU available for rendering (visualization)
#ifdef WIN32
extern "C"
{
	__declspec(dllexport) uint32_t NvOptimusEnablement = 0x00000001;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace dsm
{
	class Processor
	{
	public:

		inline Processor() { this->shouldStop = false; }
		inline ~Processor() { this->join(); }

		inline void run(InteriorNetReader& reader, Undistorter& undistorter, QtVisualizer& visualizer, std::string& settingsFile)
		{
			this->processThread = std::make_unique<std::thread>(&Processor::doRun, this,
				std::ref(reader), std::ref(undistorter), std::ref(visualizer), std::ref(settingsFile));
		}

		inline void join()
		{
			this->shouldStop = true;

			// wait the thread to exit
			if (this->processThread->joinable())
			{
				std::cout << "Waiting Processor to finish.." << std::endl;

				this->processThread->join();

				std::cout << " .. Processor has finished." << std::endl;
			}
		}

	private:

		inline void doRun(InteriorNetReader& reader, Undistorter& undistorter, QtVisualizer& visualizer, std::string& settingsFile)
		{
			int id = 0;
			cv::Mat image;
			cv::Mat image_bgr;
			double timestamp;

			const double fps = reader.fps();

			const cv::Mat& cvK = undistorter.getK();
			const Eigen::Matrix3f K((Eigen::Matrix3f() << cvK.at<double>(0, 0), cvK.at<double>(0, 1), cvK.at<double>(0, 2),
				cvK.at<double>(1, 0), cvK.at<double>(1, 1), cvK.at<double>(1, 2),
				cvK.at<double>(2, 0), cvK.at<double>(2, 1), cvK.at<double>(2, 2)).finished());

			// create DSM
			std::unique_ptr<FullSystem> DSM;

			while (!this->shouldStop)
			{
				// reset
				if (visualizer.getDoReset())
				{
					// reset slam system
					DSM.reset();

					// reset variables
					id = 0;
					timestamp = 0;
					image.release();
					image_bgr.release();

					// reset visualizer
					visualizer.reset();

					// reset dataset reader
					reader.reset();
				}

				// capture
				if (visualizer.getDoProcessing() && reader.read(image_bgr, timestamp))
				{
					double time = (double)cv::getTickCount();

					//gray image from source
					if (image_bgr.channels() == 3)
					{
						cv::cvtColor(image_bgr, image, cv::COLOR_BGR2GRAY);
					} else {
						image = image_bgr.clone();
						cv::cvtColor(image_bgr, image_bgr, cv::COLOR_GRAY2BGR);
						
					}

					// undistort
					undistorter.undistort(image, image);
					undistorter.undistort(image_bgr, image_bgr);

					if (DSM == nullptr)
					{
						DSM = std::make_unique<FullSystem>(undistorter.getOutputWidth(),
														   undistorter.getOutputHeight(),
														   K, settingsFile,
														   &visualizer);
					}

					// process
					DSM->trackFrame(id, timestamp, image.data, image_bgr.data);

					// visualize image
					visualizer.publishLiveFrame(image);

					// increase counter
					++id;

					// wait 
					time = 1000.0*(cv::getTickCount() - time) / cv::getTickFrequency();
					const double delay = (1000.0 / fps) - time;

					if (delay > 0.0)
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(delay)));
					}
				}
				else
				{
					// allow other threads to run
					std::this_thread::yield();
				}
			}

			// print log
			if (DSM)
			{
				DSM->printLog();
			}
		}

	private:

		bool shouldStop;

		std::unique_ptr<std::thread> processThread;
	};
}

int main(int argc, char *argv[])
{
	// input arguments
	std::string imageFolder, timestampFile, calibFile, settingsFile;

	// Configuration
	if (argc == 5)
	{
		imageFolder = argv[1];
		timestampFile = argv[2];
		calibFile = argv[3];
		settingsFile = argv[4];
	}
	else if (argc == 4)
	{
		imageFolder = argv[1];
		timestampFile = argv[2];
		calibFile = argv[3];
	}
	else
	{
		std::cout << "The InteriorNetExample requires at least 3 arguments: imageFolder, timestampFile, calibFile, settingsFile (optional)\n";
		return 0;
	}

	// Initialize logging
	google::InitGoogleLogging(argv[0]);
	//FLAGS_logtostderr = 1;
	//FLAGS_v = 9;

	std::cout << "\n";

	// Create the application before the window always!
	// create visualizer in the main thread
	QApplication app(argc, argv);
	dsm::QtVisualizer visualizer(app);

	std::cout << "\n";

	// read calibration
	dsm::Undistorter undistorter(calibFile);
	if (!undistorter.isValid())
	{
		std::cout << "need camera calibration file..." << std::endl;
		return 0;
	}

	std::cout << "\n";

	// read sequence
	dsm::InteriorNetReader reader(imageFolder, timestampFile, false);
	if (!reader.open())
	{
		std::cout << "no images found ..." << std::endl;
		return 0;
	}

	std::cout << "\n";

	// add image size to the visualizer
	visualizer.setImageSize(undistorter.getOutputWidth(), undistorter.getOutputHeight());

	// run processing in a second thread
	dsm::Processor processor;
	processor.run(reader, undistorter, visualizer, settingsFile);

	// run main window
	// it will block the main thread until closed
	visualizer.run();

	// join processing thread
	processor.join();

	std::cout << "Finished!" << std::endl;

	return 1;
}