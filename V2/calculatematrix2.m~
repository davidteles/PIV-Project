%%
clear;
close all;
load('cameraparametersAsus.mat');
% READ IMAGES and GENERATE POINT CLOUDS
imagefolder='/Volumes/Samsung_T5/PIV/Projecto/data_rgb/';
[imgbrgb,imgbdepth]=calculatebackground(imagefolder);
dep1=imgbdepth(:,:,1);
im1=imgbrgb(:,:,1);
dep2=imgbdepth(:,:,2);
im2=imgbrgb(:,:,2);
background1=imgbdepth(:,:,1);
background2=imgbdepth(:,:,2);
%dep2(find(dep2(:)>4000))=0;
xyz1=get_xyzasus(dep1(:),[480 640],(1:480*640)',cam_params.Kdepth,1,0);

xyz2=get_xyzasus(dep2(:),[480 640],(1:480*640)',cam_params.Kdepth,1,0);

%REGISTER RGB TO DEPTH

rgbd1 = get_gd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
rgbd2 = get_gd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);

figure(1);imagesc(rgbd1 );
colormap(gray)
figure(2);imagesc(rgbd2 );
colormap(gray);

% 
% pc1=pointCloud(xyz1,reshape(rgbd1,[480*640 3]));
% pc2=pointCloud(xyz2,reshape(rgbd2,[480*640 3]));
% 
% 
% 
% 
% 
% figure(3);clf; showPointCloud(pc1);
% figure(4);clf; showPointCloud(pc2);

%GET CORRESPONDING POINTS
np=4;
figure(1);x1=zeros(np,1);y1=x1;x2=y1;y2=x1;
for i=1:np,
    figure(1);
    [xa ya]=ginput(1);text(xa,ya,int2str(i));
    xa=fix(xa);ya=fix(ya);
    x1(i)=xa;y1(i)=ya;
    aux1=xyz1(sub2ind([480 640],ya,xa),:);
    figure(3);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
    hold off;
    figure(2);
    [xa ya]=ginput(1);text(xa,ya,int2str(i));
    xa=fix(xa);ya=fix(ya);
    x2(i)=xa;y2(i)=ya;
    aux1=xyz2(sub2ind([480 640],fix(ya),fix(xa)),:);
    figure(4);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
    hold off;drawnow;
end
%%
figure(1);hold on; plot(x1,y1,'*r');hold off;
figure(2);hold on;plot(x2,y2,'*r');hold off;
ind1=sub2ind(size(dep2),y1,x1);
ind2=sub2ind(size(dep2),y2,x2);
%%
P1=xyz1(ind1,:);
P2=xyz2(ind2,:);
inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(inds,:);P2=P2(inds,:);
[d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
%% 
%SHOW ALL CLOUDS FUSING
d=dir(strcat(imagefolder,'rgb_image1_*'));
for i=1:length(d),
    im1=imread([imagefolder 'rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread([imagefolder 'rgb_image2_' d(i).name(12:end-3) 'png']);
    
    load([imagefolder 'depth1_' d(i).name(12:end-3) 'mat'])
    dep1=depth_array;
    backremoved1=double(dep1)-background1;
    
    
    
    dep1=backremoved1;
    load([imagefolder 'depth2_' d(i).name(12:end-3) 'mat'])
    dep2=depth_array;
    backremoved2=double(dep2)-background2;
    
    
    
    dep2=backremoved2;
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
    
    
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    
    figure(1);hold off;
    showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;
end