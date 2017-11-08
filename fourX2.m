function [T_crop_new]=fourX2(T_crop)	
	
	[m n]=size(T_crop);
	T_crop_new=[];
	
	for ri=1:m
	 
		temprow=[];
		for ci=1:n
		temp=zeros(4)+T_crop(ri,ci);
		temprow=[temprow temp];
		% T_crop_new((ri-1)*4+1:ri*4,(ci-1)*4+1:ci*4)=tempmat;
		end
		T_crop_new=[T_crop_new; temprow];
	end				

	end