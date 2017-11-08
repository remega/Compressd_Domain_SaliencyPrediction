function [x_ave,y_ave] = mv_vote(eVx,eVy)

k = 16;
[x_ave,y_ave] = direction_cluster(eVx,eVy,k);%得到最终的摄像机运动方向



