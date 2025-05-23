import os

from src.conf.configs import Configs
from src.utils.logging_engine import logger


# Output logs through console and files
def ini_logger(file_name, level='info'):
    log_folder = os.path.join(Configs.output_folder, 'log')
    if not os.path.exists(log_folder):
        os.makedirs(log_folder)
    delete_files(log_folder, Configs.MAX_LOG_FILE_NUM)
    log_file = os.path.join(log_folder, file_name)
    logger.add_file_output(log_file, level)


def remove_file_handler_of_logging(file_name: str):
    log_folder = os.path.join(Configs.output_folder, 'log')
    file_path = os.path.join(log_folder, file_name)
    try:
        logger.remove_file_handler(file_path)
    except Exception as e:
        print(f"Failed to remove file handler {file_path}, reason: {e}")


def delete_files(file_folder, max_num):
    """
    :param file_folder: 目标文件夹, 绝对路径
    :param max_num: 最大文件数量
    """
    num = count_file(file_folder)
    if num > max_num:
        delete_num = max_num // 2
        total_files_and_dirs = os.listdir(file_folder)
        total_files = []
        for item in total_files_and_dirs:
            if not os.path.isdir(os.path.join(file_folder, item)):
                total_files.append(item)
        total_files.sort()
        for i in range(delete_num):
            os.remove(os.path.join(file_folder, total_files[i]))


# 计算目标文件夹下的文件数量, 不递归文件夹
def count_file(directory):
    file_num = 0
    if not os.path.exists(directory):
        os.makedirs(directory)
    for item in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, item)):
            file_num += 1
    return file_num
