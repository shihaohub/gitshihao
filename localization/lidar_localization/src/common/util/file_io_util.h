//
// Created by zhou on 9/27/19.
//

#ifndef SRC_FILE_IO_UTIL_H
#define SRC_FILE_IO_UTIL_H

#include <dirent.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <cstring>

namespace localization {
    namespace util {
        std::vector<std::string> GetFilesInFolder(const std::string &cate_dir);
    } // namespace util
} // namespace localization

#endif //SRC_FILE_IO_UTIL_H
