#include "leica_scanstation_utils/LeicaUtils.h"


std::string LeicaUtils::getDefaultPointcloudFolder()
{
	return _default_pc_path;
}

std::string LeicaUtils::findPointcloudFolder()
{
	std::string pkg_path = ros::package::getPath("leica_scanstation_utils");

	std::size_t i = pkg_path.find("leica_scanstation_utils"); 
	pkg_path = pkg_path.substr(0,i-1);

	_pc_path = pkg_path + "/leica_scanstation_utils/pointclouds/";
	return _pc_path;
}

std::string LeicaUtils::getPointCloudFolder()
{
	return _pc_path;
}

void LeicaUtils::setPointCloudFolder(std::string pc_path)
{
	_pc_path = pc_path;
}

std::string LeicaUtils::getFilePath(std::string file_name, std::string extension)
{
	std::string file_path = _pc_path;
	
	file_path += file_name + extension;

	return file_path;
}

std::string LeicaUtils::getFilePath(std::string file_name, std::string extension, int counter)
{
	std::string file_path = _pc_path;
	std::string scan_number = std::to_string(counter);

	file_path += file_name + scan_number + extension;

	return file_path;
}

void LeicaUtils::ptx2pcd(std::string file_name)
{
	ptx_2_pcd converter = ptx_2_pcd();
    converter(getFilePath(file_name,".ptx"));
}

// Initialization
std::string LeicaUtils::_pc_path = "";
std::string LeicaUtils::_default_pc_path = DEFAULT_PATH;