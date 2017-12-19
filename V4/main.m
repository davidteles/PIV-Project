%%
clear;
close all;
load('cameraparametersAsus.mat');

mode=1;
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


if(mode==1)

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

    [d,xx,tr]=ransac1(P1,P2);
    %[d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
elseif(mode==2)
    
    [xa,ya,xb,yb]=chosePoints(xyz1,xyz2);
    ind1=sub2ind(size(dep2),ya,xa);
    ind2=sub2ind(size(dep2),yb,xb);
    %%
    P1=xyz1(ind1,:);
    P2=xyz2(ind2,:);
    inds=find((P1(:,3).*P2(:,3))>0);
    P1=P1(inds,:);P2=P2(inds,:);
    
    [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
else
    
    tr=transformation;
    
end


xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);


%% SHOW ALL CLOUDS FUSING
d=dir([imagefolder 'rgb_image1_*']);


for i=1:length(d)
    %Go throw all the images
    clear( 'objects1x', 'objects1y', 'objects1z','objects1h', 'objects2x', 'objects2y', 'objects2z','objects2h');
    clear( 'lb1', 'uv1', 'lb2', 'uv2', 'objectspointcloud');
    
    %Load RGB image
    im1=imread([imagefolder 'rgb_image1_' d(i).name(12:end-3) 'png']);
    im2=imread([imagefolder 'rgb_image2_' d(i).name(12:end-3) 'png']);
    
    %Load depth images
    load([imagefolder 'depth1_' d(i).name(12:end-3) 'mat']);
    dep1=depth_array;
    
    load([imagefolder 'depth2_' d(i).name(12:end-3) 'mat']);
    dep2=depth_array;
    
    
    %Creat binary mask
    backremoved1=abs(background1-double(dep1))>250;
    backremoved2=abs(background2-double(dep2))>250;
    
    %Add countour around depth changes
    [fx1, fy1]=gradient(mat2gray(dep1));
    G1=(fx1.^2 + fy1.^2)>((250)^2);
    backremoved1=backremoved1+G1;
    
    [fx2, fy2]=gradient(mat2gray(dep2));
    G2=(fx2.^2 + fy2.^2)>((250)^2);
    backremoved2=backremoved2+G2;
    
    %Morphological filtering on binary mask
    backremoved1=imopen(backremoved1,strel('disk',10));
    backremoved2=imopen(backremoved2,strel('disk',10));
    
    
    %Apply labels to all the areas
    lb1=bwlabel(backremoved1);
    lb2=bwlabel(backremoved2);
    
    uv1 = unique(lb1);
    uv2 = unique(lb2);
    
    %Get transformation from Depth to RGB
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);    
    
    %Cut and reshape RGB image using depth
    rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
    %Calculate Point Cloud
    pctotal1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pctotal2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    
    numberofobjects=0;
    
   
    %Create Point clouds for objects detected by camera 1
    for lb=1:size(uv1,1)-1
            
        %Calculate mask per object
        mask=(abs(lb1)==lb);
        
        %Apply Mask
        deptemp=times(double(dep1),double(mask));
        
        %Get transformation from Depth to RGB
        xyz1=get_xyzasus(deptemp(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
        
        %Cut and reshape RGB image using depth
        rgbd1 = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
    
        
        %Get hsv of rgb
        temprgb(:,:,1)=rgbd1(:,:,1);
        temprgb(:,:,2)=rgbd1(:,:,2);
        temprgb(:,:,3)=rgbd1(:,:,3);
        %figure(1);hold off;
        %imagesc(temprgb);
        temphsv=rgb2hsv(temprgb);
        %figure(2);hold off;
        %imagesc(temphsv);
        hue=temphsv(:,:,1);
        hue=hue(hue~=0);
        objects1h(lb)=median(hue(:))
        
        %pause;
        
        
        
        %
        %figure(1);hold off;
        
        %Calculate Point Cloud
        pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
        
        if exist('objectspointcloud')==0
            objectspointcloud=pc1;
        else    
            objectspointcloud=pcmerge(objectspointcloud,pc1,0.001);
        end
        
        
        %Load x,y,z limits
        aux=pc1.Location(:,1);
        objects1x(1,lb)=min(aux);
        objects1x(2,lb)=max(aux);
        aux=pc1.Location(:,2);
        objects1y(1,lb)=min(aux);
        objects1y(2,lb)=max(aux);
        aux=pc1.Location(:,3);
        objects1z(1,lb)=min(aux);
        objects1z(2,lb)=max(aux);
        
        %Display Cut RGB images   
        %imagesc(rgbd1);
        
        %pause;
        %drawnow;
        
    end
    
    
    %Creat Point clouds for objects detected by camera 2
    for lb=1:size(uv2,1)-1
        %Increment the number o objects detected
        numberofobjects=numberofobjects+1;
        
        %Calculate mask per object
        mask=(abs(lb2)==lb);
        
        %Apply Mask
        deptemp=times(double(dep2),double(mask));
        
        %Get transformation from Depth to RGB
        xyz2=get_xyzasus(deptemp(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
        
        %Cut and reshape RGB image using depth
        rgbd2 = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
        
        %Get hsv of rgb
        temprgb(:,:,1)=rgbd2(:,:,1);
        temprgb(:,:,2)=rgbd2(:,:,2);
        temprgb(:,:,3)=rgbd2(:,:,3);
        %figure(1);hold off;
        %imagesc(temprgb);
        temphsv=rgb2hsv(temprgb);
        %figure(2);hold off;
        %imagesc(temphsv);
        hue=temphsv(:,:,1);
        hue=hue(hue~=0);
        objects2h(lb)=median(hue(:))
        
        
        
        
        %
        %figure(2);hold off;
        
        %Calculate Point Cloud
        pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));

        if exist('objectspointcloud')==0
            objectspointcloud=pc2;
            
        else
            objectspointcloud=pcmerge(objectspointcloud,pc2,0.001);
        end
        
        %Load x,y,z limits
        aux=pc2.Location(:,1);
        objects2x(1,lb)=min(aux);
        objects2x(2,lb)=max(aux);
        aux=pc2.Location(:,2);
        objects2y(1,lb)=min(aux);
        objects2y(2,lb)=max(aux);
        aux=pc2.Location(:,3);
        objects2z(1,lb)=min(aux);
        objects2z(2,lb)=max(aux);
        
        %showPointCloud(pc2)
        
        %Display Cut RGB images
        %imagesc(rgbd2);
        
        %pause;
        %drawnow;
        
    end
    
    if exist('objectspointcloud')~=0
            figure(3);hold off;
            showPointCloud(objectspointcloud);
            
    end
    
    figure(4);hold off;
    pcshow(pcmerge(pctotal1,pctotal2,0.001));
    drawnow;
    
    
    for j=1:size(uv1,1)-1
        
        
        X=[objects1x(1,j),objects1x(2,j),objects1x(2,j),objects1x(1,j),objects1x(1,j)];
        Y=[objects1y(1,j),objects1y(2,j),objects1y(2,j),objects1y(1,j),objects1y(1,j)];
        Z=[objects1z(1,j),objects1z(2,j),objects1z(2,j),objects1z(1,j),objects1z(1,j)];
        
        %Just to stop the script
        %erro
    end
    
        for j=1:size(uv2,1)-1
        
        
        X=[objects2x(1,j),objects2x(2,j),objects2x(2,j),objects2x(1,j),objects2x(1,j)];
        Y=[objects2y(1,j),objects2y(2,j),objects2y(2,j),objects2y(1,j),objects2y(1,j)];
        Z=[objects2z(1,j),objects2z(2,j),objects2z(2,j),objects2z(1,j),objects2z(1,j)];
        
        %Just to stop the script
        %erro
    end
    
    
    
    %Display Cut RGB images
    %figure(1);hold off;
    %imagesc(rgbd1);
    %figure(2);hold off;
    %imagesc(rgbd2);
    
    
    
    
    

end