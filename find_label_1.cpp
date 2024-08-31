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
        if (line.empty()) continue; // ��������

        std::istringstream iss(line);
        std::string token;
        int label_value = 0;
        int field_count = 0;

        // ���ַ������ж�ȡÿ���ֶ�
        while (iss >> token) {
            field_count++;
            if (field_count == 7) { // ���� label �ǵ��߸��ֶ�
                label_value = std::stoi(token); // ���ַ���ת��Ϊ����
                break;
            }
        }

        // ��� label Ϊ 1��������д������ļ�
        if (label_value == 1) {
            outfile << line << std::endl;
        }
    }

    infile.close();
    outfile.close();

    std::cout << "Extracted points with label 1 from line " << line_number << " to output file." << std::endl;
    return 0;
}