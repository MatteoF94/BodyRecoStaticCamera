function voxels_out = voxelCarving(I, m, R, M_calibr_interna, voxels_in,n,p) 
    voxels_da_proiettare = voxels_in(:,1:3); % Tengo solo le prime tre componenti 
    % Proiezione 
    voxel_camera = m + voxels_da_proiettare*R; 
    L = voxel_camera*M_calibr_interna; 
    L = L'; 
    Lnorm=round(L(1:2,:)./L(3,:));  
    % PROCEDURA DI VOTAZIONE: 
    % Se un voxel viene proiettato fuori dai limiti dell'immagine o 
    % in un punto esterno alla silhouette, il suo punteggio viene decrementato 
    % Vengono restituiti dalla funzione solo quei voxel che sono proiettati 
    % all'interno della silhouette o che pur avendo perso un punto hanno 
    % ancora punteggio superiore alla soglia 
    j=1;
    voxels_out=voxels_in;
    for i = 1:size(Lnorm,2)
        if ~((Lnorm(2,i)>0 && Lnorm(2,i)<(size(I,1)+1) && Lnorm(1,i)>0 &&...            
                Lnorm(1,i)<(size(I,2)+1))&& I(Lnorm(2,i),Lnorm(1,i))>=1 )
            voxels_in(i,4)=voxels_in(i,4)-1;
            if  voxels_in(i,4)>= (n-p)
                voxels_out(j,1:4)=voxels_in(i,1:4);
                j=j+1;         
            end
        else
            voxels_out(j,1:4)=voxels_in(i,1:4);
            j=j+1;     
        end
    end
    voxels_out=voxels_out(1:j-1,1:4);
end