#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main() {
    std::ifstream infile("0603_zhongdou41_6.txt");
    std::ofstream outfile("0603_zhongdou41_6_label2.txt");

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
        if (label_value == 2) {
            outfile << line << std::endl;
        }
    }

    infile.close();
    outfile.close();

    std::cout << "Extracted points with label 2 from line " << line_number << " to output file." << std::endl;
    return 0;
}