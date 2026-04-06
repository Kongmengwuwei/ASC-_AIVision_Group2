#ifndef __SERIAL_H
#define __SERIAL_H

//主控发送的命令枚举
typedef enum {
    Get_map,		//获取地图
	Get_car,		//获取小车信息
	Get_number,		//获取箱子或目标编号
} Order;


//以下为数据包包头，包头格式：$+三个大写字符+\n
#define MapTitle "$MAP\n"
#define CarPose "$CAR\n"


//包尾为统一格式
#define PackEnd "$END\n"






#endif
