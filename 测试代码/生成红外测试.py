import pandas as pd
from itertools import product

def generate_infrared_rules_table():
    # 定义传感器名称
    sensor_names = ["Far Left", "Left", "Center Left", "Center Right", "Right", "Far Right"]
    
    # 生成所有可能的传感器组合 (2^6 = 64种)
    combinations = list(product([0, 1], repeat=6))
    
    # 创建DataFrame
    df = pd.DataFrame(combinations, columns=sensor_names)
    
    # 添加二进制编码列
    df['Binary Code'] = df.apply(lambda row: ''.join(map(str, row)), axis=1)
    
    # 添加十进制编码列
    df['Decimal Code'] = df['Binary Code'].apply(lambda x: int(x, 2))
    
    # 添加状态描述列
    df['Sensor State'] = df.apply(lambda row: describe_sensor_state(row), axis=1)
    
    # 添加默认动作列（初始为空，可手动填写）
    df['Action'] = ""
    
    # 添加动作描述列（初始为空，可手动填写）
    df['Action Description'] = ""
    
    # 重新排序列
    df = df[['Decimal Code', 'Binary Code', 'Sensor State'] + sensor_names + ['Action', 'Action Description']]
    
    # 保存为Excel文件
    df.to_excel('infrared_tracking_rules.xlsx', index=False)
    print("红外循迹规则表已生成到 infrared_tracking_rules.xlsx")

def describe_sensor_state(row):
    sensors = ["Far Left", "Left", "Center Left", "Center Right", "Right", "Far Right"]
    black = [sensors[i] for i in range(6) if row[i] == 1]
    white = [sensors[i] for i in range(6) if row[i] == 0]
    
    if len(black) == 0:
        return "全部在白区"
    elif len(white) == 0:
        return "全部在黑线"
    else:
        return f"黑线在: {', '.join(black)} | 白区在: {', '.join(white)}"

if __name__ == "__main__":
    generate_infrared_rules_table()