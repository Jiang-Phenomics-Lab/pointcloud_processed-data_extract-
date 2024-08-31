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

# 存储每个品种的日期和点云平均值
plant_dates = {}
plant_averages = {}

for plant_type in plant_types:
    # 用于存储所有盆的数据
    combined_dates = []
    combined_total_points = []

    for j in range(1, 7):
        # 生成文件名模式
        target_filename_pattern = f"{{}}_{plant_type}_{j}.txt"

        dates = []
        total_points_list = []

        # 遍历文件夹及其子文件夹
        for root, dirs, files in os.walk(root_directory):
            for file in files:
                # 检查文件名是否符合模式
                if file.startswith(f"{file.split('_')[0]}_{plant_type}_{j}.txt") and len(file.split("_")) == 3:
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

                    # 存储数据
                    dates.append(date)
                    total_points_list.append(total_points)

        # 将数据按日期排序
        if dates:
            sorted_indices = np.argsort(dates)
            dates = np.array(dates)[sorted_indices]
            total_points_list = np.array(total_points_list)[sorted_indices]

            # 记录所有盆的日期和点云数
            combined_dates.append(dates)
            combined_total_points.append(total_points_list)

    # 计算相同日期下的六盆点云数的平均值（不去极值）
    if combined_dates:
        unique_dates = sorted(set(np.concatenate(combined_dates)))
        averaged_points = []

        for date in unique_dates:
            # 找出每盆在该日期的数据
            points_on_date = []
            for i in range(len(combined_dates)):
                if date in combined_dates[i]:
                    idx = np.where(combined_dates[i] == date)[0][0]
                    points_on_date.append(combined_total_points[i][idx])

            # 检查日期是否在0802及之后
            if date.month > 8 or (date.month == 8 and date.day >= 2):
                # 如果是，则乘以1.234
                points_on_date = [point * 1.879 for point in points_on_date]
                average = np.mean(points_on_date)
            else:
                # 如果不是，则直接计算平均值
                average = np.mean(points_on_date)
            averaged_points.append(average)
        # 存储该品种的日期和平均点云数
        if averaged_points:  # 确保有数据
            plant_dates[plant_type] = unique_dates
            plant_averages[plant_type] = averaged_points

# 绘制图像
if plant_dates and plant_averages:
    plt.figure(figsize=(10, 6))
    plt.rcParams['font.sans-serif'] = ['SimHei']

    # 绘制每个品种的曲线
    for plant_type in plant_types:
        if plant_type in plant_dates:
            plt.plot(plant_dates[plant_type], plant_averages[plant_type], marker='o', label=plant_type)

    # 设置日期格式
    all_dates = np.unique(np.concatenate([plant_dates[p] for p in plant_types if p in plant_dates]))
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
    plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(all_dates)))

    plt.title('不同品种点云数量随日期的变化', fontdict={'fontsize': 16, 'fontweight': 'bold'})
    plt.xlabel('日期')
    plt.ylabel('平均点云数')
    plt.xticks(rotation=45)
    plt.grid(True)
    plt.legend(loc='upper right')

    # 保存图像
    plt.savefig('D:\\pcl\\PCL1\\all_single_ply_sort_by_time\\all_plants_average_points_over_time_no_outlier_removal_1.879.png')
    plt.show()
else:
    print("没有足够的数据进行绘图。")
