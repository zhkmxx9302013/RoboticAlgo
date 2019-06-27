## RRT
RRT算法没有损失函数约束，倾向于拓展到开放的未探索区域，只要时间足够，迭代次数足够多，没有不会被探索到的区域。
![](https://res.cloudinary.com/djhkiiiap/image/upload/v1561456160/qweaasddaq.png)
## RRT*
RRT*加入了损失函数的启发，对路径进行渐进优化。渐进最优的RRT*算法，该算法在原有的RRT算法上，改进了父节点选择的方式，采用代价函数来选取拓展节点领域内最小代价的节点为父节点，同时，每次迭代后都会重新连接现有树上的节点，从而保证计算的复杂度和渐进最优解。
![](https://res.cloudinary.com/djhkiiiap/image/upload/v1561456973/eeeqsxdsfgg1.png)
![](https://res.cloudinary.com/djhkiiiap/image/upload/v1561471748/2000.gif)