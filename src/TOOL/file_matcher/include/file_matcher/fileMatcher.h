/*
 * @Author: Jixuan Lee
 * @Date: 2025-02-27 12:08:06
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-02-27 14:31:53
 * @FilePath: /OnlineLTSlam/src/TOOL/file_matcher/include/file_matcher/fileMatcher.h
 * @Description: 
 * @Logs: 
 */

#include <string>
#include <vector>
#include <unordered_map>
#include <utility> // for pair
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>


class fileMatcher {
    public:

        fileMatcher(const std::string& file1, const std::string& file2);
        std::vector<std::pair<std::string, std::string>> processBtcNdtFiles();
    
    private:
        std::string file1_; // Path to the first file
        std::string file2_; // Path to the second file
};
    