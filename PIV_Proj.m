cd /Volumes/Samsung_T5/PIV/Projecto/data_rgb
drgb=dir('*.png');
ddepth=dir('*.mat')
x=207;
y=173;

counterrgb = 0;
counterdepth = 0;

for i=1:length(drgb)
    
    if drgb(i).name(1)~='.' & drgb(i).name(10)~='2'
        counterrgb = counterrgb+1;
        positionoffilergb(counterrgb)=i;  
    end
end
for i=1:length(ddepth)
    if ddepth(i).name(1)~='.' & ddepth(i).name(6)~='2'
        counterdepth = counterdepth+1;
        positionoffiledepth(counterdepth)=i;  
    end
end


imgsrgb=zeros(480,640,counterrgb);
if counterrgb==counterdepth
    
    for i=1:counterrgb
       figure(1)
       aux=double(rgb2gray(imread(drgb(positionoffilergb(i)).name)));
       imshow(uint8(aux));
       file = ddepth(positionoffiledepth(i)).name;
       load(file);
       figure(2)
       imagesc(depth_array);
       drawnow;
       imgsrgb(:,:,i)=aux;
       imgsdepth(:,:,i)=double(depth_array);

    end
    figure(3)
    imgbdepth=median(imgsdepth,3);
    imagesc(imgbdepth)
    colormap(gray)

%     figure(3)
%     plot(squeeze(imgsrgb(x,y,:)))
% 
% 
%     figure(5)
%     hist(squeeze(imgsrgb(x,y,:)),[0:255])

    imgbrgb=median(imgsrgb,3);
    figure(4)
    imagesc(imgbrgb)
    colormap(gray)
    
    
    
    for i=1:counterrgb
       figure(1)
       imshow(uint8(imgsrgb(:,:,i)-imgbrgb()));
       
       ib=abs(imgsdepth(:,:,i)-imgbdepth)>300;
    
        lb=bwlabel(ib);

        uv = unique(lb);
        n  = histcounts(lb,uv);

        for j=1:max(lb(:))

            if n(j) < 125
                lb(lb==uv(j))=0;
            end

        end

        figure(2)
        imagesc(lb)
        drawnow
    end
    
    
end