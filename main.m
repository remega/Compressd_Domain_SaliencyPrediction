
 close all; clear;
 tic
 load('video_database.mat')

 video_name='BasketballDrill';
 Bin_DIR='./HEVCfiles/';

 %Video information extraction 
 for k=1:length(video_database.video_names_list)
	if(strcmp(video_database.video_names_list{k},video_name))
	index_video=k;
	break;
	end
 end
 numFrames=video_database.videos_info.frames(index_video);
 FrameRate=video_database.videos_info.framerate_fps(index_video);
 video_size=video_database.videos_info.size(index_video,:);
 width = video_size(1);
 height = video_size(2);
 

 %HEVC features extraction 
 system(['HM_120.exe -b ' Bin_DIR video_name '/str.bin' ]);%Change for .hevc
 fid=fopen('decoder_fs.txt','w+');
 fprintf(fid,'%d %d %d',width,height,numFrames);
 fclose(fid);
 system('code_mat_h.exe');
 fout_mvx = fopen(['mv_x.txt'],'r'); 
 fout_mvy = fopen(['mv_y.txt'],'r'); 
 fout_depth = fopen(['depth.txt'],'r'); 
 fout_bitmap = fopen(['bitmap.txt'],'r'); 
 imag_num = numFrames;
 video_mv_x = cell(1, imag_num);
 video_mv_y = cell(1, imag_num);
 video_bitmap = cell(1, imag_num);
 video_depth = cell(1, imag_num);
 for kk=1:numFrames
     lineNum=fscanf(fout_mvx, '%d',[1 1]);
     temp=fscanf(fout_mvx, '%d ',[width/4 height/4]);
     video_mv_x{kk}=temp';
     lineNum=fscanf(fout_mvy, '%d',[1 1]);
     temp=fscanf(fout_mvy, '%d ',[width/4 height/4]);
	 video_mv_y{kk}=temp';
     lineNum=fscanf(fout_depth, '%d',[1 1]);
     temp=fscanf(fout_depth, '%d ',[width/4 height/4]);
	 video_depth{kk}=temp';
     lineNum=fscanf(fout_bitmap, '%d',[1 1]);
     temp=fscanf(fout_bitmap, '%d ',[ceil(width/64) ceil(height/64)]);
	 video_bitmap{kk}=temp';
 end
 save([Bin_DIR video_name '/mv_x.mat'],'video_mv_x')
 save([Bin_DIR video_name '/bitmap.mat'],'video_bitmap')
 save([Bin_DIR video_name '/depth.mat'],'video_depth')
 save([Bin_DIR video_name '/mv_y.mat'],'video_mv_y')
 fclose(fout_mvx);
 fclose(fout_mvy);
 fclose(fout_depth);
 fclose(fout_bitmap);
 fclose('all');   
 file=dir('*.txt');
for n=1:length(file)
    delete(file(n).name);
end


mv_x=cell(1,numFrames);
mv_y=cell(1,numFrames);
bit_norm=cell(1,numFrames);
depth_norm=cell(1,numFrames);
mv_norm=cell(1,numFrames);
x_cammov=zeros(1,numFrames);
y_cammov=zeros(1,numFrames);


 %Camera motion detection
for ii = 1:numFrames
    Vx = video_mv_x{ii}/4;
    Vy = video_mv_y{ii}/4;
    Bm =upsample(video_bitmap{ii},16,video_size/4);
    eff_addr = find(Bm < min(min(Bm))+20);
    Vxf = medfilt2(Vx,[2,2]);
    Vyf = medfilt2(Vy,[2,2]);
	eVx = Vxf(eff_addr);
    eVy = Vyf(eff_addr);
	[flag] = cm_judge(eVx,eVy,eff_addr);
    if flag == 1
        [x_ave,y_ave] = mv_vote(eVx,eVy);
        Vx = Vxf - x_ave;
        Vy = Vyf - y_ave;
        mv_new = sqrt(Vx.^2 + Vy.^2);
		
    else
        mv_new = sqrt(Vx.^2 + Vy.^2);
		x_ave=0;
		y_ave=0;
    end
	x_cammov(ii)=round(x_ave);
	y_cammov(ii)=round(y_ave);
	mv_x{ii}=Vx;
	mv_y{ii}=Vy;
    mv_norm{ii} = mv_new / (max(max(mv_new))+eps);
    bit_norm{ii}= Bm / (max(max(Bm))+eps);
    depth_norm{ii}= video_depth{ii} / (max(max(video_depth{ii}))+eps);
end
	clear video_mv_x;
	clear video_mv_y;
	clear video_bitmap;
	clear video_depth;
	
	
% Feature computation & Generate the final map by SVM
load('svmRBFmodel_all','model','meanVec','stdVec')
addpath('./libsvm-3.17')
FeatureWeight=[0.058,0.171,-0.095,0.068,-0.01,0.509,-0.051,0.154,0.196];
linesvm=1;%Use linear svm or C-SVC
%Hyper-parameters
dims = [200, 200]; % size of the downsized images we work with SVM
PreframSmooth = round(FrameRate*0.3);% frame number for smoothing 
delta_mv_temporal=26;% delta for each feature
delta_bit_temporal=46;
delta_depth_temporal=46;
delta_bit_spacial=3;
delta_mv_spacial=11;
delta_depth_spacial=13;
patchsizebit=64*5;%patch size for spacial feature
patchsizemv=4*48;
patchsizedepth=8*24;
PreframTemporal=round(PreframSmooth*7);% frame number temporal feature

distMatrix = getdistMatrix( height, width ); 
Sal_map=cell(1,numFrames);
mvx_crop_new=cell(1,numFrames);
mvy_crop_new=cell(1,numFrames);
bit_crop_new=cell(1,numFrames);
depth_crop_new=cell(1,numFrames);
for k = 1 : numFrames		
		mv_crop=0;
		mvx_crop=0;
		mvy_crop=0;
		bit_crop=0;
		depth_crop=0;
		count=0;		
		for ii= k-PreframSmooth+1:k
		if(ii>0)
			mv_crop=mv_crop+mv_norm{ii};
			mvx_crop=mvx_crop+mv_x{ii};
			mvy_crop=mvy_crop+mv_y{ii};
			bit_crop=bit_crop+bit_norm{ii};
			depth_crop=depth_crop+depth_norm{ii};
			count=count+1;
		end
		end
		if(k>=PreframSmooth)
		mv_norm{k-PreframSmooth+1}=[];
		mv_x{k-PreframSmooth+1}=[];
		mv_y{k-PreframSmooth+1}=[];
		bit_norm{k-PreframSmooth+1}=[];
		depth_norm{k-PreframSmooth+1}=[];
		end
		mvx_crop = mvx_crop/count;
		mvy_crop = mvy_crop/count;		
		mv_crop = mv_crop/count;
		bit_crop = bit_crop/count;
		depth_crop = depth_crop/count;
		bit_crop_new{k} = bit_crop;
		depth_crop_new{k} = depth_crop;
		mvx_crop_new{k} = mvx_crop;
		mvy_crop_new{k} =mvy_crop; 
	    mv_crop=mysterythrehold(mv_crop,height,width);
		bit_crop=mysterythrehold(bit_crop,height,width);
		depth_crop=mysterythrehold(depth_crop,height,width);

		cammovsumX=0;
		cammovsumY=0;
		mvXYvaluesum=zeros(height/4,width/4);
		bitvaluesum=zeros(height/4,width/4);
		depthvaluesum=zeros(height/4,width/4);
		refbit=bit_crop_new{k};
		refdepth=depth_crop_new{k};
		refmvx=mvx_crop_new{k};
		refmvy=mvy_crop_new{k};
		for j=1 : PreframTemporal-1
		if k-j>0
			tempbit=bit_crop_new{k-j};
			tempdepth=depth_crop_new{k-j};
			tempmvx=mvx_crop_new{k-j};
			tempmvy=mvy_crop_new{k-j};
			cammovsumX = x_cammov(k-j+1)+cammovsumX;
			cammovsumY = y_cammov(k-j+1)+cammovsumY;
			tempbit=immove(tempbit,round(cammovsumX/4),round(cammovsumY/4));
			tempdepth=immove(tempdepth,round(cammovsumX/4),round(cammovsumY/4));			
			tempmvx=immove(tempmvx,round(cammovsumX/4),round(cammovsumY/4));
			tempmvy=immove(tempmvy,round(cammovsumX/4),round(cammovsumY/4));
			mvXYvaluesum=mvXYvaluesum+exp(-j/delta_mv_temporal)*sqrt((refmvx-tempmvx).^2+(refmvy-tempmvy).^2);			
			bitvaluesum=bitvaluesum+exp(-j/delta_bit_temporal)*abs(refbit-tempbit);
			depthvaluesum=depthvaluesum+exp(-j/delta_depth_temporal)*abs(refdepth-tempdepth);			
		end
		end
		
		if k-PreframTemporal>0
			mvx_crop_new{k-PreframTemporal+1}=[];
			mvy_crop_new{k-PreframTemporal+1}=[];
			bit_crop_new{k-PreframTemporal+1}=[];
			depth_crop_new{k-PreframTemporal+1}=[];
		end
		
		bit_crop2=fourX2(bit_crop);	
		depth_crop2=fourX2(depth_crop);				
		mv_crop2=fourX2(mv_crop);	
		mv_ori = imfilter(mv_crop2, fspecial('gaussian', round((height/7.5)*1), round((height/30)*1)));%32 8 for 240p 64,16 for 480p 96 24 for 720p 
		bit_ori = imfilter(bit_crop2, fspecial('gaussian', round((height/7.5)*1), round((height/30)*1)));
		depth_ori = imfilter(depth_crop2, fspecial('gaussian', round((height/7.5)*1), round((height/30)*1)));			
		mv_ori=pm_norm(mv_ori);
		bit_ori=pm_norm(bit_ori);
		depth_ori=pm_norm(depth_ori);				
		mv_temporal=fourX2(mvXYvaluesum);
		bit_temporal=fourX2(bitvaluesum);
		depth_temporal=fourX2(depthvaluesum);
		mv_temporal = imfilter(mv_temporal, fspecial('gaussian', round((height/7.5)*1), round((height/30)*1)));%32 8 for 240p 64,16 for 480p 96 24 for 720p 
		bit_temporal = imfilter(bit_temporal, fspecial('gaussian', round((height/7.5)*1), round((height/30)*1)));
		depth_temporal = imfilter(depth_temporal, fspecial('gaussian', round((height/7.5)*1), round((height/30)*1)));				
		mv_temporal=pm_norm(mv_temporal);
		bit_temporal=pm_norm(bit_temporal);
		depth_temporal=pm_norm(depth_temporal);   
		bit_contrast=cu_contrast5(bit_crop,64/4,delta_bit_spacial,patchsizebit/4);
		bit_contrast=fourX2(bit_contrast);
		bit_spacial = imfilter(bit_contrast, fspecial('gaussian', round(height/7.5)*1, round(height/30)*1));
		bit_spacial=pm_norm(bit_spacial);
		depth_contrast=cu_contrast5(depth_crop,8/4,delta_depth_spacial,patchsizedepth/4);
		depth_contrast=fourX2(depth_contrast);
		depth_spacial = imfilter(depth_contrast, fspecial('gaussian', round(height/7.5)*1, round(height/30)*1));	
		depth_spacial=pm_norm(depth_spacial);
		mvx_contrast=cu_contrast5(mvx_crop,4/4,delta_mv_spacial,patchsizemv/4);
		mvy_contrast=cu_contrast5(mvy_crop,4/4,delta_mv_spacial,patchsizemv/4);
           
		mvx_contrast(isnan(mvx_contrast))=0;
		mvy_contrast(isnan(mvy_contrast))=0;
		mvx_contrast=fourX2(mvx_contrast);
		mvy_contrast=fourX2(mvy_contrast);
		mv_contrast=sqrt(mvx_contrast.^2+mvy_contrast.^2);
		mv_spacial = imfilter(mv_contrast, fspecial('gaussian', round(height/7.5)*1, round(height/30)*1));
		mv_spacial=pm_norm(mv_spacial);
				
		depth_temporal(isnan(depth_temporal))=0;
		depth_ori(isnan(depth_ori))=0;
		bit_temporal(isnan(bit_temporal))=0;
		bit_ori(isnan(bit_ori))=0;
		mv_ori(isnan(mv_ori))=0;
		mv_temporal(isnan(mv_temporal))=0;
		bit_spacial(isnan(bit_spacial))=0;
		depth_spacial(isnan(depth_spacial))=0;
		mv_spacial(isnan(mv_spacial))=0;
		
		depth_temporal=mysterythrehold(depth_temporal,height,width);
		bit_temporal=mysterythrehold(bit_temporal,height,width);
		mv_temporal=mysterythrehold(mv_temporal,height,width);	
		if linesvm
		featuresTest=[];
		featuresTest(:, 1)=mv_ori(:);
		featuresTest(:, 2)=mv_temporal(:);
		featuresTest(:, 3)=mv_spacial(:);
		featuresTest(:, 4)=bit_ori(:);
		featuresTest(:, 5)=bit_temporal(:);
		featuresTest(:, 6)=bit_spacial(:);
		featuresTest(:, 7)=depth_ori(:);
		featuresTest(:, 8)=depth_temporal(:);
		featuresTest(:, 9)=depth_spacial(:);
		featuresTest=featuresTest-repmat(meanVec,[size(featuresTest, 1), 1]);
		featuresTest=featuresTest./repmat(stdVec,[size(featuresTest, 1), 1]);
		comb_fea=featuresTest*FeatureWeight';
		comb_fea=reshape(comb_fea,[height width]);		
		else
		featuresTest=[];
		mv_ori=imresize(mv_ori,dims,'bilinear');
		mv_temporal=imresize(mv_temporal,dims,'bilinear');
		bit_ori=imresize(bit_ori,dims,'bilinear');
		bit_temporal=imresize(bit_temporal,dims,'bilinear');
		depth_ori=imresize(depth_ori,dims,'bilinear');
		depth_temporal=imresize(depth_temporal,dims,'bilinear');
		bit_spacial=imresize(bit_spacial,dims,'bilinear');
		mv_spacial=imresize(mv_spacial,dims,'bilinear');
		depth_spacial=imresize(depth_spacial,dims,'bilinear');
		featuresTest(:, 1)=mv_ori(:);
		featuresTest(:, 2)=mv_temporal(:);
		featuresTest(:, 3)=mv_spacial(:);
		featuresTest(:, 4)=bit_ori(:);
		featuresTest(:, 5)=bit_temporal(:);
		featuresTest(:, 6)=bit_spacial(:);
		featuresTest(:, 7)=depth_ori(:);
		featuresTest(:, 8)=depth_temporal(:);
		featuresTest(:, 9)=depth_spacial(:);
		featuresTest=featuresTest-repmat(meanVec,[size(featuresTest, 1), 1]);
		featuresTest=featuresTest./repmat(stdVec,[size(featuresTest, 1), 1]);
		pixelnum=size(featuresTest,1);
		
		[predict_label, accuracy, dec_values] = libsvmpredict(zeros(pixelnum,1), featuresTest, model);
		comb_fea=reshape(dec_values,dims);
		comb_fea=pm_norm(comb_fea);
		comb_fea=imresize(comb_fea,[height width],'bilinear');
		comb_fea=imfilter(comb_fea, fspecial('gaussian', round(height/7.5), round(height/30)));
		end
		comb_fea=comb_fea.*distMatrix;
		comb_fea=pm_norm(comb_fea);
		Sal_map{k}=comb_fea;
	k
end  	

save(['Saliencymap_' video_name '.mat'],'Sal_map')



%Visualization
myObj = VideoWriter('SaliencyMap.avi');
myObj.FrameRate = FrameRate;
open(myObj);
for k = 1 : numFrames	
	writeVideo(myObj,Sal_map{k});
end
close(myObj); 



%Fixations extraction
ThisVideo_fixadata=video_database.fixdata(find(video_database.fixdata(:,2)==index_video),:);
FixationPerFrame=cell(1,numFrames);
FixationMapPerFrame=cell(1,numFrames);
Frame_durationMs=(1000/FrameRate);
 for k=1:size(ThisVideo_fixadata,1)
	fixposition=[ThisVideo_fixadata(k,5);ThisVideo_fixadata(k,6)];
	
	if fixposition(1)>0&&fixposition(1)<=width&&fixposition(2)>0&&fixposition(2)<=height
	StartFrame=ceil(ThisVideo_fixadata(k,3)/Frame_durationMs);
	EndFrame=ceil((ThisVideo_fixadata(k,3)+ThisVideo_fixadata(k,4))/Frame_durationMs);
	if(StartFrame<=0)
		StartFrame=1;
	end
	if(EndFrame>numFrames)
		EndFrame=numFrames;
	end
	for i=StartFrame:EndFrame
		FixationPerFrame{i}=[FixationPerFrame{i} fixposition];
	end
	
	end
 end
 
 toc