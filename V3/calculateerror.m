function d = calculateerror(tr,x,y)

for i=1:size(x,2)
    
    d=(x-y*tr.T+ones(length(y),1)*tr.c(1,:));
    
end



end
