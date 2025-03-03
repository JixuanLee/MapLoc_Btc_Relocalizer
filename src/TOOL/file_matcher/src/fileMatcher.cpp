/*
 * @Author: Jixuan Lee
 * @Date: 2025-02-27 12:08:11
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-03-03 11:22:43
 * @FilePath: /OnlineLTSlam/src/TOOL/file_matcher/src/fileMatcher.cpp
 * @Description: 
 * @Logs: 
 */

#include "file_matcher/fileMatcher.h"

fileMatcher::fileMatcher(const std::string& file1, const std::string& file2)
    : file1_(file1), file2_(file2) {}

std::vector<std::pair<std::string, std::string>> fileMatcher::processBtcNdtFiles() 
{
    std::vector<std::pair<std::string, std::string>> matchedPairs;
    std::unordered_map<int, std::string> file1Data;

    std::ifstream inputFile1(file1_);
    if (!inputFile1.is_open()) {
        std::cerr << "Error opening file: " << file1_ << std::endl;
        return matchedPairs;
    }

    std::string line;
    // Skip the header line for the first file of BTC / NDT
    if (std::getline(inputFile1, line)) {
        std::cout << "Skipped header line from file1: " << line << std::endl;
    }

    while (std::getline(inputFile1, line)) 
    {
        std::istringstream iss(line);
        int id;
        if (iss >> id) {
            file1Data[id] = line;
            std::cout << "Read ID from file1: " << id << std::endl;
        } else {
            std::cerr << "Invalid line in file1: " << line << std::endl;
        }
    }
    inputFile1.close();

    // Read the second file and check for matching IDs
    std::ifstream inputFile2(file2_);
    if (!inputFile2.is_open()) {
        std::cerr << "Error opening file: " << file2_ << std::endl;
        return matchedPairs;
    }

    if (std::getline(inputFile2, line)) {
        std::cout << "Skipped header line from file2: " << line << std::endl;
    }

    while (std::getline(inputFile2, line)) {
        std::istringstream iss(line);
        int id;
        if (iss >> id) {
            auto it = file1Data.find(id);
            if (it != file1Data.end()) {
                matchedPairs.emplace_back(it->second, line);
                std::cout << "Matched ID: " << id << std::endl;
            }
        } else {
            std::cerr << "Invalid line in file2: " << line << std::endl;
        }
    }
    inputFile2.close();

    return matchedPairs;
}