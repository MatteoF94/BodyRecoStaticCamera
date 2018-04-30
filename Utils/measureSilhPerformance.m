function performance = measureSilhPerformance(groundTruth,silhouettes)
%
%  Input(s):
%           groundTruth - set of silhouettes considered ground truth 
%           silhouettes - set of other silhouettes to be compared
%  Output(s):
%           performance - similarity between the two sets
%

    numSilhs = length(groundTruth);
    area = double(size(groundTruth{1},1)*size(groundTruth{1},2));
    
    errors = zeros(1,numSilhs);
    for i = 1:numSilhs
        
        % Calculate the error of a single image as the percentage of
        % misclassified pixels
        errors(i) = sum(sum(xor(groundTruth{i},silhouettes{i})))/area;
    end
    
    % The performance is the average amount of correctly classified pixels
    performance = (1-mean2(errors))*100;
end