#include "InteriorNetReader.h"

#include <iostream>
#include <fstream>

#include "opencv2/highgui.hpp"

namespace dsm
{
	InteriorNetReader::InteriorNetReader(const std::string &imageFolder, const std::string &timestampFile, bool reverse) :
		imagePath(imageFolder), timestampPath(timestampFile), id(0), inc(reverse ? -1 : 1)
	{}

	InteriorNetReader::~InteriorNetReader()
	{}

	bool InteriorNetReader::open()
	{
		bool readOK = this->readImageNames();

		if (readOK)
		{
			//sequence length in seconds
			double diff = this->timestamps.back() - this->timestamps.front();

			//fps
			this->fps_ = this->timestamps.size() / diff;

			// reset
			this->reset();

			std::cout << "InteriorNet sequence found!" << std::endl;

			return true;
		}

		return false;
	}

	void InteriorNetReader::reset()
	{
		if (this->inc > 0) this->id = 0;
		else this->id = (int)this->files.size() - 1;
	}

	bool InteriorNetReader::isOpened() const
	{
		return (this->files.size() > 0);
	}

	bool InteriorNetReader::read(cv::Mat &img, double &timestamp)
	{
		if (this->id < this->files.size() && this->id >= 0)
		{
			img = cv::imread(this->files[this->id], cv::IMREAD_UNCHANGED);
			timestamp = this->timestamps[this->id];

			this->id += this->inc;

			return true;
		}

		return false;
	}

	double InteriorNetReader::fps() const
	{
		return this->fps_;
	}

	bool InteriorNetReader::readImageNames()
	{
		//clear all data
		this->timestamps.clear();
		this->files.clear();

		// read timestamps and images with names equal to timestamps
		std::ifstream infile;
		infile.open(this->timestampPath);

		bool firstline = true;

		while (!infile.eof() && infile.good())
		{
			std::string line;
			std::getline(infile, line);
			if (firstline) {
				firstline = false;
				continue;
			}

			if (!line.empty())
			{
				
				std::size_t pos = line.find(",");
				line = line.substr(0, pos);
				
				this->files.push_back(imagePath + "/" + line + ".png");
				this->timestamps.push_back(std::atof(line.c_str()) / 1e9);		// transform to seconds
			}
		}

		if (this->timestamps.size() > 0 && this->timestamps.size() == this->files.size())
		{
			return true;
		}
		else
		{
			this->timestamps.clear();
			this->files.clear();
		}

		return false;
	}
}