/*
 * @Author: Jixuan Lee
 * @Date: 2025-02-27 12:08:48
 * @LastEditors: Jixuan Lee
 * @LastEditTime: 2025-03-03 11:21:42
 * @FilePath: /OnlineLTSlam/src/TOOL/file_matcher/src/resultOfBtcNdtCompare.cpp
 * @Description: 输入2个txt，分别为BTC与NDT的回环重定位的精度提升；本节点析出二者交集（同currID）并对其比较。
 * @Logs: 
 */

#include <iostream>
#include "file_matcher/fileMatcher.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::pair<std::vector<int>,std::vector<double>> splitToFloatsAndInts(const std::string& line) {
    std::istringstream iss(line);
    std::vector<int> intParts;
    std::vector<double> doubleParts;

    // 读取前两个整数
    int num1, num2;
    if (!(iss >> num1 >> num2)) {
        std::cerr << "Error: Unable to read the first two integers." << std::endl;
        return std::pair<std::vector<int>,std::vector<double>>(intParts, doubleParts);
    }
    intParts.push_back(num1);
    intParts.push_back(num2);

    // 读取剩余的浮点数
    double value;
    while (iss >> value) {
        doubleParts.push_back(value);
    }

    return std::pair<std::vector<int>,std::vector<double>>(intParts, doubleParts);

    // // 输出结果
    // std::cout << "Integers:" << std::endl;
    // std::cout << "CurrID: " << intParts[0] << std::endl;
    // std::cout << "LoopID: " << intParts[1] << std::endl;

    // std::cout << "\nFloats:" << std::endl;
    // for (size_t i = 0; i < doubleParts.size(); ++i) {
    //     std::cout << "Value " << i + 1 << ": " << doubleParts[i] << std::endl;
    // }
}

int main() 
{
    std::string cpoFileBtcPath = "/home/jixuanlee/cpoFileBTC-3V3.txt";
    std::string cpoFileNdtPath = "/home/jixuanlee/cpoFileNDT-5V9-skip2.txt";
    std::string resultPath = "/home/jixuanlee/resultBtcNdtMatch-010111.txt";
    fileMatcher matcher(cpoFileBtcPath, cpoFileNdtPath);
    auto pairs = matcher.processBtcNdtFiles();

    std::cout << "Matched Btc-NDt pairs count: " << pairs.size() << std::endl;

    std::ofstream matchedFile;

    std::vector<double> btcCpoImprovePrecent;
    std::vector<double> ndtCpoImprovePrecent;
    for (const auto& pair : pairs) {
        auto btcInfo = splitToFloatsAndInts(pair.first);
        auto ndtInfo = splitToFloatsAndInts(pair.second);

        double btcInfoCpoImprovePrecent = (btcInfo.second[1] - btcInfo.second[0]) / btcInfo.second[0] * 100;
        double ndtInfoCpoImprovePrecent = (ndtInfo.second[1] - ndtInfo.second[0]) / ndtInfo.second[0] * 100;

        btcCpoImprovePrecent.push_back(btcInfoCpoImprovePrecent);
        ndtCpoImprovePrecent.push_back(ndtInfoCpoImprovePrecent);

        static bool isFirst = true;
        if (isFirst)
        {
            isFirst = false;
            matchedFile.open(resultPath);
            if (!matchedFile.is_open())
                std::cout << "matchedFile Cant be Open!" << std::endl;
            std::string lineOut = "currIDofBoth btcCpoImprovePrecent ndtCpoImprovePrecent";
            matchedFile << lineOut <<"\n";
            lineOut = std::to_string(ndtInfo.first[0]) + " " + std::to_string(btcInfoCpoImprovePrecent) + " " + std::to_string(ndtInfoCpoImprovePrecent);
            matchedFile << lineOut <<"\n";
        }
        std::string lineOut = std::to_string(ndtInfo.first[0]) + " " + std::to_string(btcInfoCpoImprovePrecent) + " " + std::to_string(ndtInfoCpoImprovePrecent);
        matchedFile << lineOut <<"\n";

    }
    matchedFile.close();

    std::vector<int> indices(btcCpoImprovePrecent.size());
    for (int i = 0; i < btcCpoImprovePrecent.size(); ++i) {
        indices[i] = i;
    }
    plt::plot(indices, btcCpoImprovePrecent, "r-"); // 红色曲线
    plt::plot(indices, ndtCpoImprovePrecent, "b-"); // 蓝色曲线

    plt::xlim(0, int(btcCpoImprovePrecent.size())-1);
    plt::title("BTC and NDT CPO Improve Percent Over Index");
    plt::xlabel("Index");
    plt::ylabel("Improve Percent");

    plt::show();
    
    return 0;
}