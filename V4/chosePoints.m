function [x1,y1,x2,y2]=chosePoints(xyz1,xyz2)

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


end
