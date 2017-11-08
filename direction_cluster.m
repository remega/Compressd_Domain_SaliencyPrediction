function [x_ave,y_ave] = direction_cluster(mv_x,mv_y,k)

% tan_angle = atan2(mv_y, mv_x) * 180 / pi;
% angle_neg = find(tan_angle < 0);
% tan_angle(angle_neg) = tan_angle(angle_neg) + 360;
% tan_angle = floor(tan_angle/(360/k));

% 
tan_angle = atan2(mv_y, mv_x) / pi+1;
tan_angle = floor(tan_angle/(2/k));
tan_angle(tan_angle==16)=15;

num_cluster = zeros(1,k);

for ii = 1:k
    num_cluster(ii) = length(find(tan_angle==ii-1));
end
max_cluster = max(num_cluster);
addr_num = find(num_cluster==max_cluster);
addr = find(tan_angle==addr_num(1)-1);

x_value = mv_x(addr);
y_value = mv_y(addr);
Len=length(x_value);
Lencut=floor(Len/2);
x_sort=sort(x_value);
y_sort=sort(y_value);
% if abs(x_sort(1))>abs(x_sort(Len))
% 	x_ave = sum(x_sort(Lencut:Len)) / (Len-Lencut+1);
% else
% 	x_ave = sum(x_sort(1:2*Lencut)) /(2*Lencut) ;
% end
% 
% if abs(y_sort(1))>abs(y_sort(Len))
% 	y_ave = sum(y_sort(Lencut:Len)) / (Len-Lencut+1);
% else
% 	y_ave = sum(y_sort(1:2*Lencut)) /(2*Lencut)  ;
% end

if abs(x_sort(1))>abs(x_sort(Len))
	x_ave = sum(x_sort(Lencut:Len)) / (Len-Lencut+1);
else
	x_ave = sum(x_sort(1:Lencut)) /(Lencut) ;
end

if abs(y_sort(1))>abs(y_sort(Len))
	y_ave = sum(y_sort(Lencut:Len)) / (Len-Lencut+1);
else
	y_ave = sum(y_sort(1:Lencut)) /(Lencut)  ;
end
% [x_ave] = magnitude_vote(x_value);
% [y_ave] = magnitude_vote(y_value);

% x_ave = sum(x_value) / length(x_value);
% y_ave = sum(y_value) / length(y_value);

