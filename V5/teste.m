
clear all
close all

base_data_dir='C:\Users\hppc\Desktop\5Ano\PIV\projeto\maizena2\';
d1=dir([base_data_dir 'depth1*']);
d2=dir([base_data_dir 'depth2*']);
r1=dir([base_data_dir 'rgb_image1_*']);
r2=dir([base_data_dir 'rgb_image2_*']);
for i=1:length(d1),
    imgseq1(i).rgb=[base_data_dir r1(i).name];
    imgseq2(i).rgb=[base_data_dir r2(i).name];
    imgseq1(i).depth=[base_data_dir d1(i).name];
    imgseq2(i).depth=[base_data_dir d2(i).name];
end
%load calibration data
load cameraparametersAsus;

[objects, cam1toW, cam2toW] = track3D_part2( imgseq1, imgseq2, cam_params);

%%

clear all
close all

base_data_dir='C:\Users\hppc\Desktop\5Ano\PIV\projeto\lab1\';
d1=dir([base_data_dir 'depth1*']);
d2=dir([base_data_dir 'depth2*']);
r1=dir([base_data_dir 'rgb_image1_*']);
r2=dir([base_data_dir 'rgb_image2_*']);
for i=1:length(d1),
    imgseq1(i).rgb=[base_data_dir r1(i).name];
    imgseq2(i).rgb=[base_data_dir r2(i).name];
    imgseq1(i).depth=[base_data_dir d1(i).name];
    imgseq2(i).depth=[base_data_dir d2(i).name];
end
%load calibration data
load cameraparametersAsus;
load lab1JPC.mat

cam1toW.R=R1;
cam1toW.T=T1;

cam2toW.R=R2;
cam2toW.T=T2;

objects=track3D_part1(imgseq1, imgseq2,  cam_params, cam1toW, cam2toW);
