function [T_crop_new]=upsample(T_crop,multiply,orisize)		
	[m n]=size(T_crop);
	T_crop_new=zeros(orisize(2),orisize(1));
	for ri=1:m
		for ci=1:n
		tempmat=ones(min(ri*multiply,orisize(2))-(ri-1)*multiply,min(ci*multiply,orisize(1))-(ci-1)*multiply)*T_crop(ri,ci);
        T_crop_new((ri-1)*multiply+1:min(ri*multiply,orisize(2)),(ci-1)*multiply+1:min(ci*multiply,orisize(1)))=tempmat;	
        end
	end				
end