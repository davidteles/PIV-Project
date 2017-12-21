function [imgbrgb,imgbdepth]=calculatebackground(imgseq1,imgseq2)


drgb=length(imgseq1);
ddepth=length(imgseq1);

imgbrgb=zeros(480,640,2);
imgbdepth=zeros(480,640,2);


    counterrgb = drgb;
    counterdepth = ddepth;
    
%     for i=1:length(drgb)
%         if drgb(i).name(1)~='.' && drgb(i).name(10)== int2str(j)
%             counterrgb = counterrgb+1;
%             positionoffilergb(counterrgb)=i;  
%         end
%     end
%     for i=1:length(ddepth)
%         if ddepth(i).name(1)~='.' && ddepth(i).name(6)== int2str(j)
%             counterdepth = counterdepth+1;
%             positionoffiledepth(counterdepth)=i;  
%         end
%     end

    
    imgsrgb=zeros(480,640,counterrgb);
    imgsdepth=zeros(480,640,counterdepth);
    
    if counterrgb==counterdepth
        for i=1:counterrgb
           
           %aux=double(rgb2gray(imread(drgb(positionoffilergb(i)).name)));
           aux=double(rgb2gray(imread(imgseq1(i).rgb)));
           
           %file = ddepth(positionoffiledepth(i)).name;
           
           load(imgseq1(i).depth);
        
           imgsrgb(:,:,i)=aux;
           imgsdepth(:,:,i)=double(depth_array);

        end
        
        imgbrgb(:,:,1)=(median(imgsrgb,3));
        imgbdepth(:,:,1)=(median(imgsdepth,3));
        
        for i=1:counterrgb
           
           %aux=double(rgb2gray(imread(drgb(positionoffilergb(i)).name)));
           aux=double(rgb2gray(imread(imgseq2(i).rgb)));
           
           %file = ddepth(positionoffiledepth(i)).name;
           
           load(imgseq2(i).depth);
        
           imgsrgb(:,:,i)=aux;
           imgsdepth(:,:,i)=double(depth_array);

        end
        
        imgbrgb(:,:,2)=(median(imgsrgb,3));
        imgbdepth(:,:,2)=(median(imgsdepth,3));
        
        %figure(j+2);
        %imagesc(imgbdepth(:,:,j));
        %colormap(gray);
        
        %figure(j);
        %imagesc(imgbrgb(:,:,j));
        %colormap(gray);
    end




end
