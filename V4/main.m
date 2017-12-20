%%
clear;
close all;
load('cameraparametersAsus.mat');

mode=1;
shouldPlot = true;

huemargin=0.1;
distancemargin=0.1;

detected=struct('xlimits',{},'x_st',{},'ylimits',{},'y_st',{},'zlimits',{},'z_st',{},'huecam1',{},'huecam2',{},'frames1',{},'frames2',{},'total_frames',{});

objects=struct('X',{},'Y',{},'Z',{},'frames_tracked',{});


% READ IMAGES and GENERATE POINT CLOUDS
imagefolder='/Volumes/Samsung_T5/PIV/Projecto/data_rgb 4/';
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
numberofobjects=0;


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
    backremoved1=imopen(backremoved1,strel('disk',15));
    backremoved2=imopen(backremoved2,strel('disk',15));
    backremoved1=imopen(backremoved1,strel('square',20));
    backremoved2=imopen(backremoved2,strel('square',20));
    backremoved1=imclose(backremoved1,strel('square',25));
    backremoved2=imclose(backremoved2,strel('square',25));
    backremoved1=imdilate(backremoved1,strel('square',5));
    backremoved2=imdilate(backremoved2,strel('square',5));
    backremoved1=imopen(backremoved1,strel('square',40));
    backremoved2=imopen(backremoved2,strel('square',40));
    
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
        figure(1);hold off;
        imagesc(temprgb);
        temphsv=rgb2hsv(temprgb);
        %figure(2);hold off;
        %imagesc(temphsv);
        hue=temphsv(:,:,1);
        hue=hue(hue~=0);
        objects1h(lb)=mean(hue(:));
        
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
        figure(2);hold off;
        imagesc(temprgb);
        temphsv=rgb2hsv(temprgb);
        %figure(2);hold off;
        %imagesc(temphsv);
        hue=temphsv(:,:,1);
        hue=hue(hue~=0);
        objects2h(lb)=mean(hue(:));
        
        
        
        
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
        flag=0;
        %check if it corresponds to any of the old objects
        for counter=1:size(detected,2)

            %check if the regions intecept
            if range_intersection(detected(counter).xlimits,objects1x(:,j),distancemargin)==1 && range_intersection(detected(counter).ylimits,objects1y(:,j),distancemargin)==1 && range_intersection(detected(counter).zlimits,objects1z(:,j),distancemargin)==1
                
                %disp('range');
                if ((detected(counter).huecam1-huemargin<objects1h(:,j) && detected(counter).huecam1+huemargin>objects1h(:,j)) || detected(counter).huecam1==0)
                    %disp('color');
                    flag=1;
                    break;
                end

            else
                flag=0;
                
            end
        end
        %No match found so add new object
        if flag==1
            detected(counter).xlimits=objects1x(:,j);
            detected(counter).ylimits=objects1y(:,j);
            detected(counter).zlimits=objects1z(:,j);
            detected(counter).huecam1=objects1h(:,j);
            
            detected(counter).x_st(length(detected(counter).total_frames)+1,1)=detected(counter).xlimits(1);
            detected(counter).x_st(length(detected(counter).total_frames)+1,2)=detected(counter).xlimits(2);
            detected(counter).y_st(length(detected(counter).total_frames)+1,1)=detected(counter).ylimits(1);
            detected(counter).y_st(length(detected(counter).total_frames)+1,2)=detected(counter).ylimits(2);
            detected(counter).z_st(length(detected(counter).total_frames)+1,1)=detected(counter).zlimits(1);
            detected(counter).z_st(length(detected(counter).total_frames)+1,2)=detected(counter).zlimits(2);
            
            if isempty(detected(counter).frames1)==1
                detected(counter).frames1=i;
            end
            if detected(counter).frames1(:,length(detected(counter).frames1))~=i;
                detected(counter).frames1(:,length(detected(counter).frames1)+1)=i;
            end
            %criar o n? total de frames
            if detected(counter).total_frames(:,length(detected(counter).total_frames))~=i;
                detected(counter).total_frames(:,length(detected(counter).total_frames)+1)=i;
            end
            
            
        elseif flag==0
            numberofobjects=numberofobjects+1;
            detected(numberofobjects).xlimits=objects1x(:,j);
            detected(numberofobjects).ylimits=objects1y(:,j);
            detected(numberofobjects).zlimits=objects1z(:,j);
            detected(numberofobjects).huecam1=objects1h(:,j);
            detected(numberofobjects).huecam2=0;
            
            detected(numberofobjects).x_st(1,:)=detected(numberofobjects).xlimits;
            detected(numberofobjects).y_st(1,:)=detected(numberofobjects).ylimits;
            detected(numberofobjects).z_st(1,:)=detected(numberofobjects).zlimits;
            
            if isempty(detected(numberofobjects).frames1)==1
                detected(numberofobjects).frames1=i;
                
            end
            %criar o n? total de frames
            if isempty(detected(numberofobjects).total_frames)==1
                detected(numberofobjects).total_frames=i;
            end
            
        end
        %Just to stop the script
        %erro
    end
    
    for j=1:size(uv2,1)-1
        flag=0;
        %check if it corresponds to any of the old objects
        for counter=1:size(detected,2)
            %check if the regions intecept
            if range_intersection(detected(counter).xlimits,objects2x(:,j),distancemargin)==1 && range_intersection(detected(counter).ylimits,objects2y(:,j),distancemargin)==1 && range_intersection(detected(counter).zlimits,objects2z(:,j),distancemargin)==1
                %disp('range');
                if ((detected(counter).huecam2-huemargin<objects2h(:,j) && detected(counter).huecam2+huemargin>objects2h(:,j)) || detected(counter).huecam2==0)
                    %disp('color');
                    flag=1;
                    break;
                end

            else
                flag=0;
                
            end
        end
        %No match found so add new object
        if flag==1
            detected(counter).xlimits(1)=min(objects2x(1,j),detected(counter).xlimits(1));
            detected(counter).xlimits(2)=max(objects2x(2,j),detected(counter).xlimits(2));
            detected(counter).ylimits(1)=min(objects2y(1,j),detected(counter).ylimits(1));
            detected(counter).ylimits(2)=max(objects2y(2,j),detected(counter).ylimits(2));
            detected(counter).zlimits(1)=min(objects2z(1,j),detected(counter).zlimits(1));
            detected(counter).zlimits(2)=max(objects2z(2,j),detected(counter).zlimits(2));
            detected(counter).huecam2=objects2h(:,j);
            
            detected(counter).x_st(length(detected(counter).total_frames)+1,1)=detected(counter).xlimits(1);
            detected(counter).x_st(length(detected(counter).total_frames)+1,2)=detected(counter).xlimits(2);
            detected(counter).y_st(length(detected(counter).total_frames)+1,1)=detected(counter).ylimits(1);
            detected(counter).y_st(length(detected(counter).total_frames)+1,2)=detected(counter).ylimits(2);
            detected(counter).z_st(length(detected(counter).total_frames)+1,1)=detected(counter).zlimits(1);
            detected(counter).z_st(length(detected(counter).total_frames)+1,2)=detected(counter).zlimits(2);
            
            if isempty(detected(counter).frames2)==1
                detected(counter).frames2=i;
            end
            if detected(counter).frames2(:,length(detected(counter).frames2))~=i
                detected(counter).frames2(:,length(detected(counter).frames2)+1)=i;
                
            end
             %criar o n? total de frames
            if detected(counter).total_frames(:,length(detected(counter).total_frames))~=i;
                detected(counter).total_frames(:,length(detected(counter).total_frames)+1)=i;
            end
            
        elseif flag==0
            numberofobjects=numberofobjects+1;
            detected(numberofobjects).xlimits=objects2x(:,j);
            detected(numberofobjects).ylimits=objects2y(:,j);
            detected(numberofobjects).zlimits=objects2z(:,j);
            detected(numberofobjects).huecam1=0;
            detected(numberofobjects).huecam2=objects2h(:,j);
            
            detected(numberofobjects).x_st(1,:)=detected(numberofobjects).xlimits;
            detected(numberofobjects).y_st(1,:)=detected(numberofobjects).ylimits;
            detected(numberofobjects).z_st(1,:)=detected(numberofobjects).zlimits;
            
            if isempty(detected(numberofobjects).frames2)==1
                detected(numberofobjects).frames2=i;
            end
            %criar o n? de frames total
            if isempty(detected(numberofobjects).total_frames)==1
                detected(numberofobjects).total_frames=i;
                
            end
        end
        
        
    end
    
    %for counter=1:size(detected,2)
        %Insert code to draw the boxes one per object
        
    %end
    
    %Display Cut RGB images
    %figure(1);hold off;
    %imagesc(rgbd1);
    %figure(2);hold off;
    %imagesc(rgbd2);
    
    
    
    
    

end
%%
%construir a estrutura de output
%to confirm the structure, check the model from prof JPClab1.mat
for i=1:numberofobjects
   objects(i).X=[detected(i).x_st(:,2) detected(i).x_st(:,2) detected(i).x_st(:,2) detected(i).x_st(:,2) detected(i).x_st(:,1) detected(i).x_st(:,1) detected(i).x_st(:,1) detected(i).x_st(:,1)];
   objects(i).Y=[detected(i).y_st(:,2) detected(i).y_st(:,1) detected(i).y_st(:,2) detected(i).y_st(:,1) detected(i).y_st(:,2) detected(i).y_st(:,1) detected(i).y_st(:,2) detected(i).y_st(:,1)]; 
   objects(i).Z=[detected(i).z_st(:,1) detected(i).z_st(:,1) detected(i).z_st(:,2) detected(i).z_st(:,2) detected(i).z_st(:,1) detected(i).z_st(:,1) detected(i).z_st(:,2) detected(i).z_st(:,2)]; 
   objects(i).frames_tracked = detected(i).total_frames;
end
