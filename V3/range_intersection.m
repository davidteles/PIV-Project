function out=range_intersection(first,second)

    if first(1)<second(1) && first(2)>second(1) || first(1)<second(2) && first(2)>second(2)
        out=1;

    else
        
        out=0;
    end
    
end
