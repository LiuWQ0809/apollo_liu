import os
import subprocess

def process_images_with_tool(image_directory, tool_path):
    """
    使用指定的可执行文件批量处理图像目录中的所有图像文件。
    
    参数:
    - image_directory: 包含图像文件的目录路径。
    - tool_path: main_offline_tool 可执行文件的路径。
    """
    # 确保提供的目录存在并且是一个目录
    if not os.path.isdir(image_directory):
        print(f"错误：'{image_directory}' 不是一个有效的目录。")
        return
    
    # 遍历指定目录中的所有文件
    for filename in os.listdir(image_directory):
        file_path = os.path.join(image_directory, filename)
        
        # 检查文件是否为图像文件（这里可以根据需要添加更多的图像格式）
        if os.path.isfile(file_path) and filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            # 构建命令行参数
            command = [tool_path, file_path]
            
            # 执行命令并捕获输出
            try:
                result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                print(f"成功处理文件 '{file_path}': {result.stdout.decode()}")
            except subprocess.CalledProcessError as e:
                print(f"处理文件 '{file_path}' 时发生错误: {e.stderr.decode()}")

if __name__ == "__main__":
    # 替换为你的main_offline_tool可执行文件的实际路径
    tool_executable_path = "/apollo/dev/install/apollo/modules/perception/camera_detection_occupancy/main_offline_tool"
    
    # 替换为你想要处理的图像文件所在的目录路径
    images_directory = "/apollo_workspace/data/camera_data/test1/0.png"
    img1 = os.path.join(images_directory, "0.png")
    img2 = os.path.join(images_directory, "1.png")
    img3 = os.path.join(images_directory, "2.png")
    img4 = os.path.join(images_directory, "3.png")
    img5 = os.path.join(images_directory, "4.png")
    img6 = os.path.join(images_directory, "5.png")
    output_path = "/home/liuwq/work/apollo/modules/perception/camera_detection_occupancy/data/occ_results"

    command = [tool_executable_path, img1, img2, img3, img4, img5, img6, output_path]
            
    # 执行命令并捕获输出
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"成功处理文件 '{file_path}': {result.stdout.decode()}")
    except subprocess.CalledProcessError as e:
        print(f"处理文件 '{file_path}' 时发生错误: {e.stderr.decode()}")
    
    # 调用函数处理图像
    # process_images_with_tool(images_directory, tool_executable_path)