function depth_crop_new=mysterythrehold(depth_crop,im_height,im_width)

		ave = sum(sum(depth_crop)) / (im_height*im_width);
		sum_big = depth_crop(depth_crop > (max(max(depth_crop))-ave));
		thresh = sum(sum_big) / length(sum_big);
		depth_crop(depth_crop > thresh) = thresh;
		depth_crop_new = depth_crop / (max(max(depth_crop)) + 0.000001);
end	