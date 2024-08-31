import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import matplotlib.ticker as ticker

# 指定顶层文件夹路径
root_directory = "D:\\pcl\\PCL1\\all_single_ply_sort_by_time"  # 请替换为实际路径

# 定义要查找的品种列表
plant_types = ["liaodou50", "kedou35", "kedou46", "heihe43", "zhongdou41", "WL82", "jidou19"]

for plant_type in plant_types:
    # 存储每个盆栽的数据
    all_dates = []
    all_total_points_list = []
    all_modified_total_points_list = []  # 新增列表用于存储调整后的点云数量
    
    for j in range(1, 7):
        target_filename = "{}_{}.txt".format(plant_type, j)
        dates = []
        total_points_list = []

        # 遍历文件夹及其子文件夹
        for root, dirs, files in os.walk(root_directory):
            for file in files:
                # 检查文件名是否符合日期 + "_plant_type_盆号.txt"的格式
                if file.endswith(target_filename) and len(file.split("_")) == 3:
                    # 提取日期部分
                    date_str = file.split("_")[0]
                    try:
                        # 将日期转换为datetime对象
                        date = datetime.strptime(date_str, "%m%d")
                    except ValueError:
                        print(f"跳过无法解析的日期格式文件: {file}")
                        continue
                    
                    # 读取点云数据
                    file_path = os.path.join(root, file)
                    data = np.loadtxt(file_path)

                    # 统计点云总数
                    total_points = data.shape[0]
                    if date.month > 8 or (date.month == 8 and date.day >= 2):
                        total_points = total_points * 1.879
                    # 存储数据
                    dates.append(date)
                    total_points_list.append(total_points)

        # 将数据按日期排序
        sorted_indices = np.argsort(dates)
        dates = np.array(dates)[sorted_indices]
        total_points_list = np.array(total_points_list)[sorted_indices]

        all_dates.append(dates)
        all_total_points_list.append(total_points_list)

        

    # 绘制图像
    plt.figure(figsize=(10, 6))
    plt.rcParams['font.sans-serif'] = ['SimHei']
    
    # 设置日期格式
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
    plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(np.unique(np.concatenate(all_dates)))))

    # 绘制6条曲线
    for i in range(6):
        plt.plot(all_dates[i], all_total_points_list[i], marker='o', label='盆 {}'.format(i + 1))

    plt.title('{} 总点云数随日期变化'.format(plant_type), fontdict={'fontsize': 14, 'fontweight': 'bold'})
    plt.xlabel('日期')
    plt.ylabel('总点云数')
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.legend(loc='upper right')

    # 保存图像
    plt.savefig('D:\\pcl\\PCL1\\all_single_ply_sort_by_time\\{}_total_points_over_time_1.879.png'.format(plant_type))
    # plt.show()
