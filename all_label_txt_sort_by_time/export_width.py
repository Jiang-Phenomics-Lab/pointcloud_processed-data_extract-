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
data = {variety: {i: [] for i in range(1, 7)} for variety in varieties}  # 初始化每个品种每株的空数据列表

def calculate_canopy_width(canopy_str):
    try:
        # 将冠宽的a*b格式数据转换为 (a + b) / 2
        a, b = map(float, canopy_str.split('*'))
        return (a + b) / 2
    except:
        return None  # 如果格式错误，返回None

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
    
    # 提取每个品种的每株数据
    for variety in varieties:
        # 筛选当前品种的数据
        variety_data = df[df.iloc[:, 1] == variety]
        
        if not variety_data.empty:
            for i in range(1, 7):  # 遍历六株
                plant_data = variety_data[variety_data.iloc[:, 2] == i]  # 根据序号筛选单株数据
                if not plant_data.empty:
                    # 获取冠宽数据并计算
                    canopy_width_str = plant_data.iloc[0, 5]  # 假设冠宽在第6列
                    canopy_width = calculate_canopy_width(canopy_width_str)
                    
                    if canopy_width is not None:
                        data[variety][i].append((date, canopy_width))

# 绘制每个品种的图
for variety in varieties:
    plt.figure(figsize=(10, 6))
    for i in range(1, 7):
        if data[variety][i]:
            # 按日期排序
            data[variety][i].sort(key=lambda x: datetime.strptime(x[0], '%m-%d'))
            
            # 提取日期和冠宽
            dates = [datetime.strptime(v[0], '%m-%d')for v in data[variety][i]]
            canopy_widths = [v[1] for v in data[variety][i]]
            
            # 绘制每株的曲线
            plt.plot(dates, canopy_widths, marker='o', label=f'{variety} 株{i}')
    
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.title(f'{variety} 品种的冠宽变化趋势')
    plt.xlabel('日期 (月-日)')
    plt.ylabel('冠宽 (cm)')
    plt.xticks(rotation=45)
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))
    plt.gca().xaxis.set_major_locator(ticker.FixedLocator(mdates.date2num(dates)))  # 仅显示数据中的日期
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # 保存图像到指定文件夹路径
    save_path = os.path.join(folder_path, f'{variety}_canopy_width_trend.png')
    plt.savefig(save_path)
    plt.close()  # 关闭图像以节省内存
