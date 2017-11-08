function [flag] = cm_judge(eVx,eVy,eff_addr)
%flag=1，表示当前帧摄像机运动
%flag=0，表示当前帧摄像机静止
%V = sqrt(eVx.^2 + eVy.^2);
V =eVx.^2 + eVy.^2;
T = length(find(V <=2));
eff_num = length(eff_addr);
if T > 0.5*eff_num
    flag = 0;
else
    flag = 1;
end

