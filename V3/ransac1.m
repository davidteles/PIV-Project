function [best xxx transform] = ransac1(x,y)
%[f inlierIdx] = ransac1( x,y,ransacCoef,funcFindF,funcDist )
%	Use RANdom SAmple Consensus to find a fit from X to Y.
%	X is M*n matrix including n points with dim M, Y is N*n;
%	The fit, f, and the indices of inliers, are returned.
%
%	RANSACCOEF is a struct with following fields:
%	minPtNum,iterNum,thDist,thInlrRatio
%	MINPTNUM is the minimum number of points with whom can we 
%	find a fit. For line fitting, it's 2. For homography, it's 4.
%	ITERNUM is the number of iteration, THDIST is the inlier 
%	distance threshold and ROUND(THINLRRATIO*n) is the inlier number threshold.
%
%	FUNCFINDF is a func handle, f1 = funcFindF(x1,y1)
%	x1 is M*n1 and y1 is N*n1, n1 >= ransacCoef.minPtNum
%	f1 can be of any type.
%	FUNCDIST is a func handle, d = funcDist(f,x1,y1)
%	It uses f returned by FUNCFINDF, and return the distance
%	between f and the points, d is 1*n1.
%	For line fitting, it should calculate the dist between the line and the
%	points [x1;y1]; for homography, it should project x1 to y2 then
%	calculate the dist between y1 and y2.
%	Yan Ke @ THUEE, 20110123, xjed09@gmail.com


minPtNum = 4;
iterNum = 300;
thInlrRatio = 0.025;
thDist = 0.02;
ptNum = size(x,1);
thInlr = round(0.5*ptNum);

inlrNum = zeros(1,iterNum);

best=1000;

for p = 1:iterNum
    
	% 1. fit using  random points
	sampleIdx = randIndex(ptNum,minPtNum);
    x(sampleIdx,:);
    y(sampleIdx,:);
	[d xx tr] = procrustes(x(sampleIdx,:),y(sampleIdx,:),'scaling',false,'reflection',false);
	
	% 2. count the inliers, if more than thInlr, refit; else iterate
	dist = calculateerror(tr,x,y);
    sse=norm(dist,2);
	inlier1 = find(dist < thDist);
	inlrNum(p) = length(inlier1);
    
    if sse<best
        
        best=sse;
        xxx=xx;
        transform=tr;
        
    end
 
end

end
