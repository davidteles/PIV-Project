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

%Go throw all the images
for i=1:length(d),
    clear objects1x;
    clear objects1y;
    clear objects1z;
    clear objects2x;
    clear objects2y;
    clear objects2z;
    clear lb1;
    clear uv1;
    clear lb2;
    clear uv2;
    clear objectspointcloud;
    
    
    %Load RGB image
    im1=imread([imagefolder 'rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread([imagefolder 'rgb_image2_' d(i).name(12:end-3) 'png']);
    
    %Load depth images
    load([imagefolder 'depth1_' d(i).name(12:end-3) 'mat']);
    dep1=depth_array;
    
    load([imagefolder 'depth2_' d(i).name(12:end-3) 'mat']);
    dep2=depth_array;
    
    
    %Morphollogical filtering on binary mask
    backremoved1=abs(double(dep1)-background1)>300;
    backremoved1=imopen(backremoved1,strel('disk',5));
    backremoved1=imopen(backremoved1,strel('disk',10));
    backremoved1=imdilate(backremoved1,strel('disk',25));
    backremoved1=imerode(backremoved1,strel('disk',26));
    %backremoved1=imclose(backremoved1,strel('disk',25));
    backremoved1=imopen(backremoved1,strel('disk',10)); 

    backremoved2=abs(double(dep2)-background2)>300;
    backremoved2=imopen(backremoved2,strel('disk',5));
    backremoved2=imopen(backremoved2,strel('disk',10));
    backremoved2=imdilate(backremoved2,strel('disk',25));
    backremoved2=imerode(backremoved2,strel('disk',26));
    %backremoved2=imclose(backremoved2,strel('disk',25));
    backremoved2=imopen(backremoved2,strel('disk',10));
    
    %Apply labels to all the areas
    lb1=bwlabel(backremoved1);
    lb2=bwlabel(backremoved2);
    
    uv1 = unique(lb1);
    uv2 = unique(lb2);
    
    
    
    numberofobjects=0;
    
    %Creat Point clouds for objects detected by camera 1
    for lb=1:size(uv1,1)
        
        
        %Calculate mask per object
        mask=abs(backremoved1)==lb;
        
        %Apply Mask
        dep1=times(double(dep1),double(mask));
        
        %Get transformation from Depth to RGB
        xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
        
        %Cut and reshape RGB image using depth
        rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
    
        %
        figure(1);hold off;
        
        %Calculate Point Cloud
        pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
        
        if exist('objectspointcloud')==0
            objectspointcloud=pc1;
        else
            
            objectspointcloud=pcmerge(objectspointcloud,pc1,0.001);
        end
        
        
        %Load x,y,z limits
        objects1x(:,lb)=pc1.XLimits;
        objects1y(:,lb)=pc1.YLimits;
        objects1z(:,lb)=pc1.ZLimits;
        %showPointCloud(pc1)
        
        %Display Cut RGB images   
        imagesc(rgbd1);
        
        %pause;
        drawnow;
        
    end
    
    
     %Creat Point clouds for objects detected by camera 2
    for lb=1:size(uv2,1)
   
        
        %Increment the number o objects detected
        numberofobjects=numberofobjects+1;
        
        %Calculate mask per object
        mask=abs(backremoved2)==lb;
        
        %Apply Mask
        dep2=times(double(dep2),double(mask));
        
        %Get transformation from Depth to RGB
        xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
        
        %Cut and reshape RGB image using depth
        rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
        
        %
        figure(2);hold off;
        
        %Calculate Point Cloud
        pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
        
        if exist('objectspointcloud')==0
            objectspointcloud=pc2;
            
        else
            objectspointcloud=pcmerge(objectspointcloud,pc2,0.001);
        end
        
        %Load x,y,z limits
        objects2x(:,lb)=pc2.XLimits;
        objects2y(:,lb)=pc2.YLimits;
        objects2z(:,lb)=pc2.ZLimits;
        
        %showPointCloud(pc2)
        
        %Display Cut RGB images
        imagesc(rgbd2);
        
        %pause;
        drawnow;
    end
    
    
    
    
    
    
            
            
            
            figure(3);hold off;
            showPointCloud(objectspointcloud);
            %pcshow(pcmerge(pc1,pc2,0.001));
            drawnow;
       
    
    
    
    
    
    
    %Display Cut RGB images
    %figure(1);hold off;
    %imagesc(rgbd1);
    %figure(2);hold off;
    %imagesc(rgbd2);
    
    
    
    
    

end