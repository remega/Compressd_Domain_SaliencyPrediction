function [v_ave] = magnitude_vote(value)


max_class = 20;
step = (max(value) - min(value) + 1) / max_class;
range = min(value) : step : max(value);
len = length(range);
label = zeros(1, len);

for ii = 1 : len-1
    len1 = length(find(value >= range(ii)));
    len2 = length(find(value >= range(ii+1)));
    label(ii) = len1 - len2;
end
label(len) = length(find(value >= range(len)));
addr_max = find(label == max(label));
v_ave = 0;
for ii = 1 : length(addr_max)
    v_temp = value(value >= range(addr_max(ii)));
    if addr_max(ii)<len
        v_use = v_temp(v_temp < range(addr_max(ii)+1));
    else
        v_use = v_temp;
    end
    v_ave = sum(v_use)/length(v_use) + v_ave;
end
v_ave=v_ave/length(addr_max);
;