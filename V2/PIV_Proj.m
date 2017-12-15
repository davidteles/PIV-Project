function [imgbrgb,imgbdepth]=calculatebackground()

cd /Volumes/Samsung_T5/PIV/Projecto/data_rgb
drgb=dir('*.png');
ddepth=dir('*.mat')
x=207;
y=173;


for j =1:2
    counterrgb = 0;
    counterdepth = 0;
    for i=1:length(drgb)

        if drgb(i).name(1)~='.' && drgb(i).name(10)~=j
            counterrgb = counterrgb+1;
            positionoffilergb(counterrgb)=i;  
        end
    end
    for i=1:length(ddepth)
        if ddepth(i).name(1)~='.' && ddepth(i).name(6)~=j
            counterdepth = counterdepth+1;
            positionoffiledepth(counterdepth)=i;  
        end
    end


    imgsrgb=zeros(480,640,counterrgb);
    if counterrgb==counterdepth

        for i=1:counterrgb
           
           aux=double(rgb2gray(imread(drgb(positionoffilergb(i)).name)));
           
           file = ddepth(positionoffiledepth(i)).name;
           load(file);
        
           imgsrgb(:,:,i)=aux;
           imgsdepth(:,:,i)=double(depth_array);

        end
        figure(j+2)
        imgbdepth(:,:,j)=median(imgsdepth,3);
        imagesc(imgbdepth(:,:,j))
        colormap(gray)
        imgbrgb(:,:,j)=median(imgsrgb,3);
        figure(j)
        imagesc(imgbrgb(:,:,j))
        colormap(gray)
    end


    
end