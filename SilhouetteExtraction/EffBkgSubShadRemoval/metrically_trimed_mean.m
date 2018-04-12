function [y,w]=metrically_trimed_mean(x,alpha)
%
%  Computes the pixel-wise symetrically_trimed mean y for an array of
%  frames N x M x t, as well as the median w (for comparison purposes)
%
    if nargin==1
        alpha=.3; % percentual of the trim
    end
    x=double(x);
    w=median(x,3);
    s=size(x);
    T=round((1-alpha)*s(3)); 
    q=abs(x-repmat(w,[1 1 s(3)]));
    
    y = zeros(s(1),s(2));
    for i=1:s(1)
        for j=1:s(2)
            p=reshape(x(i,j,:),[1 s(3)]);
            [~,ind]=sort(reshape(q(i,j,:),[1 s(3)]));
            y(i,j)=mean(p(ind(1:T)));
        end
    end
end

