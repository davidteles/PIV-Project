cd /Volumes/Samsung_T5/PIV/Projecto/data_rgb
ddepth=dir('*.mat')

counterdepth = 0;

for i=1:length(ddepth)
    if ddepth(i).name(1)~='.' & ddepth(i).name(6)~='2'
        counterdepth = counterdepth+1;
        positionoffiledepth(counterdepth)=i;  
    end
end

imgsdepth=zeros(480,640,counterdepth);

for i=1:counterdepth
    
    file = ddepth(positionoffiledepth(i)).name;
    load(file);
    imagesc(depth_array);
    drawnow;
    imgsdepth(:,:,i)=double(depth_array);
    
   
end
figure(2)
imgb=median(imgsdepth,3);
imagesc(imgb)
colormap(gray)




for i=1:counterdepth
   figure(3)
   imagesc(abs(imgsdepth(:,:,i)-imgb)>300);
   drawnow;
   %pause
end

for i=1:counterdepth
    ib=abs(imgsdepth(:,:,i)-imgb)>300;
    
    lb=bwlabel(ib);
    
    uv = unique(lb);
    n  = histcounts(lb,uv);
    
    for j=1:max(lb(:))
      
        if n(j) < 125
            lb(lb==uv(j))=0;
        end
         
    end
    
    
    
    
    figure(4)
    imagesc(lb)
    drawnow
    %pause

%     figure(5)
%     hist(lb(:),[0:980])
%     drawnow
end
