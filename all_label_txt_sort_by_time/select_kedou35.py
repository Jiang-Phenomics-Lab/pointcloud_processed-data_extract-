import os
import shutil

# 原始文件夹路径
source_folder = r'D:\pcl\PCL1\all_label_txt_sort_by_time'

# 目标文件夹路径
destination_folder = r'D:\pcl\PCL1\all_label_txt_sort_by_time\kedou35'

# 创建目标文件夹，如果不存在的话
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

for root, dirs, files in os.walk(source_folder):
    for filename in files:
        # 检查文件名是否包含 '_kedou35_' 并以 '.txt' 结尾
        if '_kedou35_' in filename and filename.endswith('.txt'):
            # 构造原文件的完整路径
            file_path = os.path.join(root, filename)
            # 复制文件到目标文件夹
            shutil.copy(file_path, destination_folder)

print('文件复制完成！')