function [xa,ya,xb,yb]=findFeatures(Ia,Ib,shouldPlot)

[fa, da] = vl_sift(Ia) ;
[fb, db] = vl_sift(Ib) ;

thresh = 2;
[matches, score] = vl_ubcmatch (da, db, thresh);

while length(matches)<4
    thresh = thresh*1.25;
    [matches, score] = vl_ubcmatch (da, db, thresh);
end

xa = fa(1,matches(1,:)) 
xb = fb(1,matches(2,:))
ya = fa(2,matches(1,:)) 
yb = fb(2,matches(2,:)) 


if shouldPlot
    figure(2) ; clf ;
    imagesc(cat(2, Ia, Ib)) ;

    xbp = xb + size(Ia,2) ;

    hold on ;
    h = line([xa ; xbp], [ya ; yb]) ;
    set(h,'linewidth', 1, 'color', 'b') ;

    vl_plotframe(fa(:,matches(1,:))) ;
    fb(1,:) = fb(1,:) + size(Ia,2) ;
    vl_plotframe(fb(:,matches(2,:))) ;
    axis image off ;
end



end