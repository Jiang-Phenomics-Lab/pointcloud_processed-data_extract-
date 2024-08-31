#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main() {
    std::ifstream infile("E:\\0723&0718\\predict_result\\0718_WL82_2.txt");
    std::ofstream outfile("E:\\0723&0718\\0718_WL82_2_leave.txt");

    if (!infile.is_open() || !outfile.is_open()) {
        std::cerr << "Error opening files." << std::endl;
        return -1;
    }

    std::string line;
    int line_number = 0;
    while (std::getline(infile, line)) {
        line_number++;
        if (line.empty()) continue; // 跳过空行

        std::istringstream iss(line);
        std::string token;
        int label_value = 0;
        int field_count = 0;

        // 从字符串流中读取每个字段
        while (iss >> token) {
            field_count++;
            if (field_count == 7) { // 假设 label 是第七个字段
                label_value = std::stoi(token); // 将字符串转换为整数
                break;
            }
        }

        // 如果 label 为 1，则将整行写入输出文件
        if (label_value == 1) {
            outfile << line << std::endl;
        }
    }

    infile.close();
    outfile.close();

    std::cout << "Extracted points with label 1 from line " << line_number << " to output file." << std::endl;
    return 0;
}