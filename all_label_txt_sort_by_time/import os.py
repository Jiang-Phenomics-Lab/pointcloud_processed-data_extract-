import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import matplotlib.ticker as ticker

# 指定顶层文件夹路径
root_directory = "D:\\pcl\\PCL1\\all_single_ply_sort_by_time"  # 请替换为实际路径

# 定义要查找的文件名称模式
for i in ["liaodou50","kedou35","kedou46","heihe43","zhongdou41","WL82","jidou19"]:
# for i in ["heihe43"]:
    for j in range(1,7):
        target_filename = "{}_{}.txt".format(i,j)
        plant_name = target_filename.split('.')[0]

    # 存储结果的列表
        dates = []
        total_points_list = []
        label_1_ratios = []
        label_1_counts = []

        # 遍历文件夹及其子文件夹
        for root, dirs, files in os.walk(root_directory):
            for file in files:
                # 检查文件名是否符合日期 + "_jidou19_1.txt"的格式
                if file.endswith(target_filename) and len(file.split("_")) == 3:
                    # 提取日期部分
                    date_str = file.split("_")[0]
                    try:
                        # 将日期转换为datetime对象
                        date = datetime.strptime(date_str, "%m%d")  # 修改为 "%m%d" 适应日期格式
                    except ValueError:
                        print(f"跳过无法解析的日期格式文件: {file}")
                        continue
                    
                    # 读取点云数据
                    file_path = os.path.join(root, file)
                    data = np.loadtxt(file_path)

                    # 统计点云总数
                    total_points = data.shape[0]

                    # 统计第七列 (标签) 为 1 的点云数量
                    # label_1_count = np.sum(data[:, 6] == 1)  # 确保标签列的索引正确

                    # 计算标签为 1 的点云占总点云数的比例
                    # label_1_ratio = label_1_count / total_points if total_points > 0 else 0

                    # 存储数据
                    dates.append(date)
                    total_points_list.append(total_points)
                    # label_1_ratios.append(label_1_ratio)
                    # label_1_counts.append(label_1_count)

        # 将数据按日期排序
        sorted_indices = np.argsort(dates)
        dates = np.array(dates)[sorted_indices]
        total_points_list = np.array(total_points_list)[sorted_indices]
        # label_1_ratios = np.array(label_1_ratios)[sorted_indices]
        # label_1_counts = np.array(label_1_counts)[sorted_indices]

        # 创建图像并保存


        # 图像1: 日期 vs 总点云数
        plt.figure(figsize=(6, 6))
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.gca().xaxis.set_major_locator(mdates.DayLocator())
        plt.subplot(3, 1, 1)
        plt.plot(dates, total_points_list, marker='o')
        plt.title('{}总点云数随日期变化'.format(plant_name), fontdict={'fontsize': 14, 'fontweight': 'bold'})
        plt.xlabel('日期')
        plt.ylabel('总点云数')
        # plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(200000))  # 设置y轴刻度间隔
        plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
        plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(dates)))  # 仅显示数据中的日期
        plt.xticks(dates)
        plt.grid(True)
        plt.savefig('D:\\pcl\\PCL1\\all_single_ply_sort_by_time\\{}_total_points_over_time.png'.format(plant_name))
        plt.show()

        # # 图像2: 日期 vs 标签为1的点云数目比例
        # plt.figure(figsize=(6, 6))
        # plt.rcParams['font.sans-serif'] = ['SimHei']
        # plt.gca().xaxis.set_major_locator(mdates.DayLocator())
        # plt.subplot(3, 1, 2)
        # plt.plot(dates, label_1_ratios, marker='o', color='orange')
        # plt.title('{}标签为1的点云数目占比随日期变化'.format(plant_name), fontdict={'fontsize': 14, 'fontweight': 'bold'})
        # plt.xlabel('日期')
        # plt.ylabel('标签为1的点云占比')
        # plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
        # plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(dates)))  # 仅显示数据中的日期
        # plt.xticks(dates)
        # plt.grid(True)
        # plt.savefig('D:\\pcl\\PCL1\\all_label_txt_sort_by_time\\{}_label_1_ratio_over_time.png'.format(plant_name))
        # plt.show()


        # # 图像3: 日期 vs 标签为1的点云数目
        # plt.figure(figsize=(8, 6))
        # plt.rcParams['font.sans-serif'] = ['SimHei']
        # plt.gca().xaxis.set_major_locator(mdates.DayLocator())
        # plt.subplot(3, 1, 3)
        # plt.plot(dates, label_1_counts, marker='o', color='green')
        # plt.title('{}标签为1的点云数目随日期变化'.format(plant_name), fontdict={'fontsize': 14, 'fontweight': 'bold'})
        # plt.xlabel('日期')
        # plt.ylabel('标签为1的点云数目')
        # plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
        # plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(dates)))  # 仅显示数据中的日期
        # plt.xticks(dates)
        # plt.grid(True)
        # plt.savefig('{}_label_1_count_over_time.png'.format(plant_name))
        # plt.show()
