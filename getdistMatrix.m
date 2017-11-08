function distMatrix = getdistMatrix( hei, wid )

    imgr = hei;
    imgc = wid;    
    midpointx = floor( imgr/2 );
    midpointy = floor( imgc/2 );
    distMatrix = zeros( imgr, imgc );
    for x = 1 : imgr
        for y = 1 : imgc
            distMatrix( x, y ) = floor( sqrt( ( x-midpointx )^2 + ( y-midpointy )^2 ) );
        end
    end
    distMatrix = distMatrix / max( distMatrix( : ) );
    distMatrix = 1 - distMatrix;  
    distMatrix = double(distMatrix);
   
return;