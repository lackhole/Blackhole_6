#pragma once

#include <iostream>
#include <math.h>

#include <direct.h>

#include "opencv2\highgui\highgui.hpp" 
#include "opencv2\core\core.hpp" 
#include "opencv2\opencv.hpp"

#include "singleton.h"

using std::string;

const int nstep = 20;
const int nstep_safe = nstep - 1;
const double nstep_inverse = 1. / (double)nstep;
const double time_coefficient = 1;


const int buffer_count = nstep * 2 + 2;

class _Properties;
class _Directory;
class _Image;

class _Properties : _Singleton<_Properties> {
public:
	_Directory& getDirectory() { return *directory; };
	_Image& getImage() { return *image; };

private:

	_Directory* directory;
	_Image* image;
};

class _Directory : public _Singleton<_Directory> {
private:
	struct _Seperator {
		string WinOS, Linux, Short;
	};

public:

	static const _Seperator seperator;
	static string outDirectory, outDirectory_image, outDirectory_video;
	static string inDirectory;

	void MakeOutDirectories() {
		try {
			if (!(
				outDirectory.length() |
				outDirectory_image.length() |
				outDirectory_video.length()
				)) {
				throw std::length_error("Length of some out directory(ies) is 0");
			}
		}
		catch (const std::length_error e) {
			std::cout << e.what() << std::endl;
			std::cin.get();
			exit(EXIT_FAILURE);
		}

		string directory = "";
		string outFolder = std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
		for (const char& c : outDirectory) {
			switch (c) {
			case '/':
			case '\\':
				directory += '\\';
			default:
				directory += c;
			}
		}

		_mkdir(directory.c_str());
		_mkdir((directory + this->seperator.WinOS + outFolder).c_str());
		_mkdir((directory + this->seperator.WinOS + outFolder + seperator.WinOS + outDirectory_image).c_str());
		_mkdir((directory + this->seperator.WinOS + outFolder + seperator.WinOS + outDirectory_video).c_str());
	}

};

const _Directory::_Seperator _Directory::seperator = { "\\","/","/" };
string _Directory::inDirectory = "C:/Users/cosge/Desktop/";
string _Directory::outDirectory = "C:/Users/cosge/Desktop/blackhole";
string _Directory::outDirectory_image = "image";
string _Directory::outDirectory_video = "video";

class _Image {
public:
	static string name, extension;

	static cv::Mat image, backgroundImage, backgroundImage_copy;

	static unsigned int backgroundImage_width, backgroundImage_height;
};
