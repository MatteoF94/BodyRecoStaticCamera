function myPatch = create3DReconstruction(voxels,delta)
%
%  Input(s):
%           voxels - set of voxels describing the volume occupied by the
%                    target 
%           delta - side length of a voxel
%  Output(s):
%           myPatch - 3D patch object representing the volume
%

    % Discretize again the volume around the target, without repetitions
    ux = unique(voxels(:,1));
    uy = unique(voxels(:,2));
    uz = unique(voxels(:,3));
    
    ux = [ux(1)-delta; ux; ux(end)+delta];
    uy = [uy(1)-delta; uy; uy(end)+delta];
    uz = [uz(1)-delta; uz; uz(end)+delta];
    
    % Create a grid with the discretized values
    [X,Y,Z] = meshgrid( ux, uy, uz );
    V = zeros( size( X ) );
    N = numel( voxels(:,1) );
    maxValue = max(voxels(:,4));
    
    for ii=1:N
        ix = (ux == voxels(ii,1));
        iy = (uy == voxels(ii,2));
        iz = (uz == voxels(ii,3));
        V(iy,ix,iz) = voxels(ii,4)/maxValue;
    end
    
    myPatch = patch(isosurface(X,Y,Z,V,0.5));
    isonormals(X,Y,Z,V,myPatch)
    set(myPatch,'FaceColor','g','EdgeColor','none');
    set(gca,'DataAspectRatio',[1 1 1]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(0,-87)
    lighting('gouraud')
    camlight(0,0)
    axis('tight')
end