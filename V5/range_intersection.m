function out=range_intersection(first,second,margin)

    if ((first(1)-margin<second(1) && first(2)+margin>second(1)) || (first(1)-margin<second(2) && first(2)+margin>second(2)))
        out=1;

    else
        
        out=0;
    end
    
end
