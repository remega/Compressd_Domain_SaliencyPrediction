function contrastmap=Sudoku_contrastcpp5(subsaliencymap,delta,cusize,patchsize)

[m,n]=size(subsaliencymap);
contrastmap=zeros(m,n);

WINDOW_SIZE=round(patchsize/cusize);
len=floor(WINDOW_SIZE/2);
[X Y] = meshgrid(1:WINDOW_SIZE,1:WINDOW_SIZE);
x0=len+1;
y0=len+1;
PATCH_RADIUS =  sqrt(max(X(:))^2+max(Y(:))^2);
R = ((X-x0).^2+(Y-y0).^2).^0.5;
% delta=1;
W=exp(-R.^2/(2*delta^2));
% W=W./sum(W(:));


subsaliencymap2=subsaliencymap+0.00000001;
% for ii=1:len
% subsaliencymap2=[subsaliencymap(1,:);subsaliencymap2;subsaliencymap(m,:)];
% end
% subsaliencymap3=subsaliencymap2;
% for ii=1:len
% subsaliencymap3=[subsaliencymap2(:,1) subsaliencymap3 subsaliencymap2(:,n)];
% end
for ii=1:len
	subsaliencymap2=[zeros(1,size(subsaliencymap2,2));subsaliencymap2;zeros(1,size(subsaliencymap2,2))];
end
subsaliencymap3=subsaliencymap2;
for ii=1:len
	subsaliencymap3=[zeros(size(subsaliencymap3,1),1) subsaliencymap3 zeros(size(subsaliencymap3,1),1)];
end
[m1,n1]=size(subsaliencymap3);
subsaliencymap3=pm_norm(subsaliencymap3);
%subsaliencymap2=round(subsaliencymap2*16);

 contrastmap=computecontrast5(subsaliencymap3,W,len,WINDOW_SIZE,m,n);

% for i=1+len:m+len
% 	for j=1+len:n+len
% 		temppatch=subsaliencymap3(i-len:i-len+WINDOW_SIZE-1,j-len:j-len+WINDOW_SIZE-1);
% 		patch_mean=sum(sum(temppatch.*W));//new: patch_mean=subsaliencymap3(i,j);
% 		my_contrast=W.*((temppatch-patch_mean).^2);
% 		contrast_value=sqrt(sum(my_contrast(:)));new：contrast_value=W.*abs(temppatch-patch_mean));
% 		contrastmap(i-len,j-len)=contrast_value;
% 	end
% end 


contrastmap=contrastmap./max(contrastmap(:));

end