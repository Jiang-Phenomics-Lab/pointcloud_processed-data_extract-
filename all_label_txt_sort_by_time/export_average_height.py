import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import matplotlib.dates as mdates
import matplotlib.ticker as ticker

# 设置文件夹路径
folder_path = r'C:\Users\74773\Desktop\大豆真实值'

# 品种信息
varieties = ["冀豆19", "科豆35", "科豆46", "辽豆50", "WL82", "中豆41", "黑河43"]

# 遍历文件夹中的Excel文件
data = {variety: [] for variety in varieties}  # 初始化每个品种的空数据列表

for filename in os.listdir(folder_path):
    # 跳过临时文件或不符合日期格式的文件
    if not filename.endswith(".xlsx") or filename.startswith("~$"):
        continue
    
    try:
        # 从文件名提取日期信息
        date = datetime.strptime(filename.split('.')[0], '%Y%m%d').strftime('%m-%d')
    except ValueError:
        print(f"跳过无法解析的文件: {filename}")
        continue
    
    # 读取Excel文件
    file_path = os.path.join(folder_path, filename)
    df = pd.read_excel(file_path)
    
    # 提取每个品种的平均高度
    for variety in varieties:
        # 筛选当前品种的数据
        variety_data = df[df.iloc[:, 1] == variety]
        
        if not variety_data.empty:
            # 计算该品种在该日期的平均高度
            avg_height = variety_data.iloc[:, 4].mean()  # 假设高度在第5列
            data[variety].append((date, avg_height))

# 绘制包含所有品种平均高度的图
plt.figure(figsize=(10, 6))
for variety in varieties:
    if data[variety]:
        # 按日期排序
        data[variety].sort(key=lambda x: datetime.strptime(x[0], '%m-%d'))
        
        # 提取日期和平均高度
        dates = [datetime.strptime(v[0], '%m-%d') for v in data[variety]]
        print(dates)
        avg_heights = [v[1] for v in data[variety]]
        
        # 绘制每个品种的平均高度曲线
        plt.plot(dates, avg_heights, marker='o', label=variety)

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.title('各品种平均高度随时间的变化')
plt.xlabel('日期 (月-日)')
plt.ylabel('平均高度 (cm)')
plt.xticks(rotation=45)
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
# print(dates)
plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(dates)))  # 仅显示数据中的日期
plt.grid(True)
plt.legend()
plt.tight_layout()

# 保存图像
# plt.savefig(folder_path + '\\all_varieties_average_growth.png')
plt.show()
