function voxelsOut = voxelCarving(I,t,R,K,voxelsIn,n,p) 
%
%  Input(s):
%           I - silhouette image
%           t - location of the camera
%           R - rotation matrix of the camera
%           K - intrinsic parameters of the camera
%           voxelsIn - starting voxels, before carving
%           n - maximum fuzzy value of the voxels
%           p - number for which if a voxel has fuzzy value < (n - p) it is
%               discarded
%  Output(s):
%           voxelsOut - voxels after carving
%

    % We consider only the 3D coordinates of the voxels
    voxelsProj = voxelsIn(:,1:3);
    
    % Then we project them into camera coordinates and image coordinates 
    voxelsCamera = t + voxelsProj*R; 
    L = voxelsCamera*K; 
    L = L'; 
    Lnorm=round(L(1:2,:)./L(3,:));  
    
    % If a voxel has its image outside the silhouette or outside the
    % boundaries of the frame, its fuzzy value is decremented by one. If
    % such value is less than a fixed threshold, we discard the voxel,
    % otherise it is kept (allowing soft constraints).
    % Otherwise if a voxel has its image inside the silhouette, it is
    % immedialy kept, without any change in the fuzzy value.
    j=1;
    voxelsOut = voxelsIn;
    for i = 1:size(Lnorm,2)
        if ~((Lnorm(2,i)>0 && Lnorm(2,i)<(size(I,1)+1) && Lnorm(1,i)>0 &&...            
                Lnorm(1,i)<(size(I,2)+1))&& I(Lnorm(2,i),Lnorm(1,i))>=1 )
            
            voxelsIn(i,4)=voxelsIn(i,4)-1;
            if  voxelsIn(i,4)>= (n-p)
                voxelsOut(j,1:4)=voxelsIn(i,1:4);
                j=j+1;         
            end
        else
            voxelsOut(j,1:4)=voxelsIn(i,1:4);
            j=j+1;     
        end
    end
    
    voxelsOut=voxelsOut(1:j-1,1:4);
end