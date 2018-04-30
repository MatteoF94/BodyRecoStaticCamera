function [voxel, voxel3Dx, voxel3Dy, voxel3Dz, voxels_number] = initializeVoxels(xlim, ylim, zlim, voxel_size)
%
%  Input(s):
%           xlim - boundaries of the volume on the X axes
%           ylim - boundaries of the volume on the Y axes
%           zlim - boundaries of the volume on the Z axes
%           voxel_size - lengths of a voxel sides
%  Output(s):
%           voxel - discretization of the volume bounded by the input
%                   limits
%           voxel3Dx - X coordinates of the rectangular grid containing the
%                      voxels
%           voxel3Dy - Y coordinates of the rectangular grid containing the
%                      voxel
%           voxel3Dz - Z coordinates of the rectangular grid containing the
%                      voxel
%           voxel_number - number of generated voxels
%           
%

    % Find the total number of voxels given the boundaries and size
    voxels_number(1) = abs(xlim(2)-xlim(1))./voxel_size(1);
    voxels_number(2) = abs(ylim(2)-ylim(1))./voxel_size(2);
    voxels_number(3) = abs(zlim(2)-zlim(1))./voxel_size(3);
    voxels_number_act = voxels_number +1;
    total_number = prod(voxels_number_act);

    voxel = (ones(uint64(total_number), 4));

    sx = xlim(1);
    ex = xlim(2);
    sy = ylim(1);
    ey = ylim(2);
    sz = zlim(1);
    ez = zlim(2);

    if(ex>sx)
        x_step = voxel_size(1);
    else
        x_step = -voxel_size(1);
    end

    if(ey>sy)
        y_step = voxel_size(2);
    else
        y_step = -voxel_size(2);
    end

    if(sz>ez)
        z_step = voxel_size(3);
    else
        z_step = -voxel_size(3);
    end

    % Create a 3D distribution of voxels
    [voxel3Dx, voxel3Dy, voxel3Dz] = meshgrid(sx:x_step:ex, ...
                                              sy:y_step:ey, ...
                                              ez:z_step:sz);

    l = 1;
    
    % Fill up voxels table with coordinates
    for z=ez:z_step:sz
        for x=sx:x_step:ex
            for y=sy:y_step:ey
                voxel(l,1:3) = [x y z]; 
                l=l+1;
            end
        end
    end
end