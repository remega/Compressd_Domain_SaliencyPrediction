

function  outsaliency=cu_contrast5(salmap,cusize,delta,patchsize)

if cusize~=1
[orisizeM orisizeN]=size(salmap);

numrow=ceil(orisizeM/cusize);
numcol=ceil(orisizeN/cusize);

lastcol=numcol-(numcol-1)*cusize;
lastrow=numrow-(numrow-1)*cusize;

subsaliencymap=zeros(numrow,numcol);
for k=1:numrow
	for j=1:numcol
		startrow=(k-1)*cusize+1;
		startcol=(j-1)*cusize+1;
		if(k==numrow)
			endrow=orisizeM;
		else
			endrow=k*cusize;
		end
		if(j==numcol)
			endcol=orisizeN;
		else
			endcol=j*cusize;
		end
		temp=salmap(startrow:endrow,startcol:endcol);
		subsaliencymap(k,j)=mean(mean(temp));
	end
end

contrastmap=Sudoku_contrastcpp5(subsaliencymap,delta,cusize,patchsize);

resaliencymap=zeros(orisizeM,orisizeN);
for k=1:numrow
	for j=1:numcol
		startrow=(k-1)*cusize+1;
		startcol=(j-1)*cusize+1;
		if(k==numrow)
			endrow=orisizeM;
		else
			endrow=k*cusize;
		end
		if(j==numcol)
			endcol=orisizeN;
		else
			endcol=j*cusize;
		end
		
		resaliencymap(startrow:endrow,startcol:endcol)=contrastmap(k,j)*ones(endrow-startrow+1,endcol-startcol+1);		
	end
end
outsaliency=resaliencymap./max(resaliencymap(:));

else %cusize=1;

outsaliency=Sudoku_contrastcpp5(salmap,delta,cusize,patchsize);
outsaliency=outsaliency./max(outsaliency(:));
end

end