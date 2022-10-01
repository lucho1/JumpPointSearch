#include "p2Log.h"
#include <string>
#include <vector>
#include <fstream>
#include <xiosbase>
#include <filesystem>
#include <iostream>


std::vector<std::string> logs;


void log(const char file[], int line, const char* format, ...)
{
	static char tmp_string[4096];
	static char tmp_string2[4096];
	static va_list  ap;

	// Construct the string from variable arguments
	va_start(ap, format);
	vsprintf_s(tmp_string, 4096, format, ap);
	va_end(ap);

	sprintf_s(tmp_string2, 4096, "\n%s(%d) : %s", file, line, tmp_string);
	OutputDebugString(tmp_string2);

	logs.push_back(tmp_string2);
}

void saveLogFile()
{
	const std::string projectDir = std::filesystem::current_path().u8string() + "\\output";
	const std::string filepath = projectDir + "\\logs.txt";

	std::ofstream ofs(filepath.c_str(), std::ios_base::out | std::ios_base::trunc);
	for (const std::string& message : logs)
		ofs << message.c_str() << "\n";

	ofs.close();
}