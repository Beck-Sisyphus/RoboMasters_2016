# 视觉目标检测算法说明
## 功能
通过安装在战车上的摄像头,检测视野范围内的敌方战车。
## 算法
### 目标检测与识别
1. 颜色检测
采集大量敌方机器人的图片数据,并进行训练,得到对方机器人的颜色区间, 并以此为阈值对整幅图像进行颜色检测,找到疑似敌方机器人的区域,量化 成二值图。
2. 滤除噪声点 对得到的二值图像进行开运算处理,滤除颜色检测结果中的噪声点。
3. 连通区域检测
对图像中的疑似区域进行连通区域检测,计算出每个疑似区域的外部轮廓, 用矩形近似表示。
4. 连通区域合并
根据连通区域之间的距离和颜色相似性,将距离接近且相似性高的连通区域 进行合并。
5. 形状和大小过滤
对大量敌方机器人图片进行训练,得到对方机器人的形状信息(例如长宽比)
和大小信息(面积),并以此为依据将不符合的区域过滤掉。 经过以上五步的处理,可以初步得到敌方机器人的位置、大小和形状信息。

### 目标运动跟踪
对上步中的检测结果进行运动跟踪。

1. 状态估计
根据上一时刻地方机器人的运动状态(包括位置和速度),估算当前时刻机 器人的运动状态。
2. 轨迹关联 根据位置和颜色信息,对当前时刻机器人的估计状态和检测结果进行关联。
3. 状态更新
若上一步中关联成功,更新当前时刻的运动状态。 通过对检测结果进行运动跟踪,可以计算出当前时刻敌方机器人的运动速度和方 向。
2.3 预估提前量
1. 评估延迟时间
根据己方机器人实际的调试情况,通过多次试验和统计的方法,估算己方 机器人从接收命令到炮弹(或子弹)击中目标区域的时间延时(包括图像处理 时间、落弹时间和炮弹飞行时间)。
2. 计算提前量
根据延迟时间和敌方机器人的运动速度,计算炮弹发射的提前量,补偿到 敌方机器人的运动状态中。
## 总结
对于机器人战车中的敌方目标检问题,有很多种方法可以实现,视觉检测只是其中 的一种方法,而基于颜色识别的目标检测也只是视觉算法中比较简单有效的一种。所以, 本段代码只是抛砖引玉的一个样本,适用范围只针对于 2014 年 RoboMasters 夏令营的 场地和战车,希望可以看到大家更加简单有效的算法。

# Introduction for Visual Target Detection algorithm

## Function 
Based on cameras installed on the Vehicles, detect enemy vehicle in the visual range.

## Algorithm
### Target Detection and Distinction
1. Color Detection
Collect massive graphic data for enemy automaton, and make training. Obtain the color range of enemy automaton and make color detection on the whole graph by setting this as threshold. Find regions that probably contain enemy automaton and quantize into binary image.
2. Filter out Image Noise
Make “Opening” operation on obtained binary image, and filter out the image noise in the image detection solution.
3. Connected Component Detection
Make a connected component detection on the suspected regions that might have enemy. Find out each region’s out contour, and represent it by a similar rectangle. 
4. Connected Component combination
Based on the distance and color similarity between Connected Components, combine those Connected Components that is close to each other and have high color similarity.
5. Shape and Size Filter
Train on large number of enemy automaton graphs. Get enemy automaton’s shape information (such as length width ratio) and size information (area), and filter the unqualified region based on these data.
After five operations above, we can preliminary get enemy automation’s position, size and shape information.

### Target Motion Tracking 
Make motion tracking based on the detection solution form previous step.
Status Estimate
Based on automaton’s motion status in the last moment (includes position and speed), estimate automaton’s current motion status.
Tracks Correlations
Based on position and color information, correlate current automaton’s estimate status and the detection result.
Status update
If the correlation is success in the last step, then update the current motion status.
By making motion tracking on detection result, we can calculate enemy automation’s moving speed and moving direction.

### Pre-estimate advance quantity
1. Estimate Time Latency
Based on the actual debugging situation of our automaton, use the method that make large number of experience and collect the data to estimate the time latency between the time automaton receiving command to the time the shell (or bullet) is hitting the target (Include graphic operation time, Fall time and shell projectile time).
2. Calculate advance quantity
Based on time latency and enemy automaton’s moving speed, calculate the advance quantity of shell launch. Compensate into enemy automaton’s moving status.

## Summary
There are so many ways to realize enemy detection in automaton. Visual detection is one of them, and target detection based on color is one of simpler and effective way in visual algorithm. So, this paragraph of code is just a inspiring sample which is only used on battlefields and vehicles 2014 RoboMasters summer Camp. We hope to see more simple and effective algorithm from you.
