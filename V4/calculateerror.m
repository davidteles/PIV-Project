function d = calculateerror(tr,x,y)

for i=1:size(x,2)
    
    d=(x-y*tr.T+ones(length(y),1)*tr.c(1,:)).^2;
    d=sqrt(d(:,1)+d(:,2)+d(:,3));
end



end
