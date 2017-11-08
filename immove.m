function out=immove(im,mov_x,mov_y)
%frame is immobile, image moves.

[m, n]=size(im);
padX=zeros(m,abs(mov_x));
if(mov_x>0)
	temp=[padX im];
else
	temp=[im padX];
end

padY=zeros(abs(mov_y),n+abs(mov_x));
if(mov_y>0)
	temp2=[padY; temp];
else
	temp2=[temp; padY];
end

[m2, n2]=size(temp);

if mov_x>0
	if mov_y>0
		out = temp2(1:m,1:n);
	else	
		out = temp2(m2-m+1:m2,1:n);
	end
else
	if mov_y>0
		out = temp2(1:m,n2-n+1:n2);
	else
		out = temp2(m2-m+1:m2,n2-n+1:n2);
	end
end

end