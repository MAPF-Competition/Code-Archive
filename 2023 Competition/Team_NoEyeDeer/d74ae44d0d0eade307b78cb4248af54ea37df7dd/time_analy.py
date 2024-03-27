import json
import numpy as np
import matplotlib.pyplot as plt
import sys

def plot_histogram(data):
    # 定义bins的范围为0到10，间隔为1
    bins = list(range(0, 11, 1))
    
    # 使用matplotlib绘制直方图
    plt.hist(data, bins=bins, edgecolor='black', align='left')
    
    # 单独统计大于10的频次
    count_above_10 = sum(1 for d in data if d > 10)
    plt.bar(10.5, count_above_10, width=1, align='center', color=plt.cm.get_cmap()(0.5), edgecolor='black')
    
    # plt.title("Histogram of plannerTimes")
    # plt.xlabel("Value")
    # plt.ylabel("Frequency")
    # plt.xticks(list(range(0, 12)), labels=[str(i) for i in range(0, 10)] + ['>10'])
    plt.show()

def read_planner_times_from_json(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
        return data['plannerTimes']

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide a JSON filename as an argument.")
        sys.exit(1)
    
    filename = sys.argv[1]
    planner_times = read_planner_times_from_json(filename)
    print(max(planner_times))
    print(min(planner_times))
    plot_histogram(planner_times)
