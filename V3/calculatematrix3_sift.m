%%
clear;
close all;
load('cameraparametersAsus.mat');

shouldPlot = true;

% READ IMAGES and GENERATE POINT CLOUDS
imagefolder='/Volumes/Samsung_T5/PIV/Projecto/data_rgb 3/';
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

if shouldPlot
    figure(1);imagesc(rgbd1 );
    colormap(gray)
    figure(2);imagesc(rgbd2 );
    colormap(gray);
end


%% SIFT point matching
Ia = single(rgbd1);
Ib = single(rgbd2);

[xa,ya,xb,yb]=findFeatures(Ia,Ib,shouldPlot);
%% Procrustes for SIFT matches
ind1=sub2ind(size(dep1),round(ya),round(xa));
ind2=sub2ind(size(dep2),round(yb),round(xb));

P1=xyz1(ind1,:);

P2=xyz2(ind2,:);

inds=find((P1(:,3).*P2(:,3))>0);

P1=P1(inds,:);P2=P2(inds,:);

[d,xx,tr]=ransac1(P1,P2)
%[d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);


xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);


%% SHOW ALL CLOUDS FUSING
d=dir([imagefolder 'rgb_image1_*']);


for i=1:length(d)
    %Go throw all the images
    clear( 'objects1x', 'objects1y', 'objects1z', 'objects2x', 'objects2y', 'objects2z');
    clear( 'lb1', 'uv1', 'lb2', 'uv2', 'objectspointcloud');
    
    %Load RGB image
    im1=imread([imagefolder 'rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread([imagefolder 'rgb_image2_' d(i).name(12:end-3) 'png']);
    
    %Load depth images
    load([imagefolder 'depth1_' d(i).name(12:end-3) 'mat']);
    dep1=depth_array;
    
    load([imagefolder 'depth2_' d(i).name(12:end-3) 'mat']);
    dep2=depth_array;
    
    
    %Morphological filtering on binary mask
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
    
    %Create Point clouds for objects detected by camera 1
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
        %imagesc(rgbd1);
        
        %pause;
        %drawnow;
        
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
        %imagesc(rgbd2);
        
        %pause;
        %drawnow;
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