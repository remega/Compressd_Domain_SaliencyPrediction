clc
clear;
[heart_scale_label, heart_scale_inst] = libsvmread('heart_scale');
model = libsvmtrain(heart_scale_label, heart_scale_inst, '-t 0 -c 1 -g 0.07 -b 1');
% [predict_label, accuracy, dec_values] = libsvmpredict(heart_scale_label, heart_scale_inst, model,' -b 1');
[a,b,c] = libsvmpredict(heart_scale_label, heart_scale_inst, model, '-b 1');


model1 = svmtrain(heart_scale_inst, heart_scale_label, 'kernel_function', 'rbf');

%% SVMÍøÂçÔ¤²â
 [predict_label] = svmclassify(model,test_wine);