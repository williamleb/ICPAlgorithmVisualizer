/**
 * @file main.cpp  
 *
 * @brief GTI320 Labo 2 - Lance une fenêtre NanoGUI
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <nanogui/common.h>
#include <nanogui/object.h>

#include <iostream>
#include <string>
#include <ctime>

#include "IcpApplication.h"
#include "IcpDataPath.h"

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

void usage(const char* cmd)
{
	std::cout << "Usage : " << cmd << " [options] [OBJFOLDER]\n\n";
	std::cout << "  Loads .obj files from folder OBJFOLDER. Default is the current working directory.\n\n";
	std::cout << "  Options:\n";
	std::cout << "    -h, --help   Display this help.\n" << std::endl;
}

int main(int argc, char** argv)
{
	const char* objFolder = OBJ_DEFAULT_PATH;
	for (int i = 1; i < argc; ++i)
	{
		if (std::string("-h").compare(argv[i]) == 0)
		{
			usage(argv[0]);
			exit(0);
		}
		else if (std::string("--help").compare(argv[i]) == 0)
		{
			usage(argv[0]);
			exit(0);
		}
		else
		{
			objFolder = argv[1];
		}
	}
	std::srand(std::time(0));

	nanogui::init();
	nanogui::ref<IcpApplication> app = new IcpApplication(objFolder);
	app->drawAll();
	app->setVisible(true);
	nanogui::mainloop();
	nanogui::shutdown();

	return 0;
}
