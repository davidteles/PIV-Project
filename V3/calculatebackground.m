function [imgbrgb,imgbdepth]=calculatebackground(folder)

oldfolder = cd (folder);
drgb=dir('*.png');
ddepth=dir('*.mat');

imgbrgb=zeros(480,640,2);
imgbdepth=zeros(480,640,2);

for j =1:2
    counterrgb = 0;
    counterdepth = 0;
    for i=1:length(drgb)
        if drgb(i).name(1)~='.' && drgb(i).name(10)== int2str(j)
            counterrgb = counterrgb+1;
            positionoffilergb(counterrgb)=i;  
        end
    end
    for i=1:length(ddepth)
        if ddepth(i).name(1)~='.' && ddepth(i).name(6)== int2str(j)
            counterdepth = counterdepth+1;
            positionoffiledepth(counterdepth)=i;  
        end
    end

    
    imgsrgb=zeros(480,640,counterrgb);
    imgsdepth=zeros(480,640,counterdepth);
    
    if counterrgb==counterdepth
        for i=1:counterrgb
           
           aux=double(rgb2gray(imread(drgb(positionoffilergb(i)).name)));
           
           file = ddepth(positionoffiledepth(i)).name;
           load(file);
        
           imgsrgb(:,:,i)=aux;
           imgsdepth(:,:,i)=double(depth_array);

        end
        
        imgbrgb(:,:,j)=(median(imgsrgb,3));
        imgbdepth(:,:,j)=(median(imgsdepth,3));
        %figure(j+2);
        %imagesc(imgbdepth(:,:,j));
        %colormap(gray);
        
        %figure(j);
        %imagesc(imgbrgb(:,:,j));
        %colormap(gray);
    end


end
cd (oldfolder);
end
