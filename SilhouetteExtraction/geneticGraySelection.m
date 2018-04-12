function [rT,gT,bT] = geneticGraySelection(bkgs,img,refSilh,numChrom,iters)
%
%  Algorithm steps: 1) Generate a random population of numChrom chromosomes
%                   2) Evaluate the fitness of each chromosome
%                   3) Test an ending condition (in this implementation the
%                      end is given by the last iteration)
%                   4) Create a new population exploiting selection,
%                      crossover and mutation of the chromosomes
%                   5) Replace the population to repeat the algorithm from
%                      step 2
%
%  Input(s): 
%           bkgs - images of the background used to compute the background model
%           img - image to compare with the background to detect the target
%           refSilh - manually extracted silhouette, used as reference
%           numChrom - number of chromosomes in the population (optional, 20 default)
%           iters - iteration of the genetic algorithm (optional, 100 default)
%  Output(s): 
%           rT - threshold of the red channel
%           gT - threshold of the green channel
%           bT - threshold of the blue channel
%

    % Define the initial population of chromosomes, having just 20~30 of
    % them
    chromosomes = cell(numChrom,1);
    fitness = zeros(numChrom,1);
    for i = 1:numChrom
        chromosomes{i} = [randi([1,20]),randi([1,20]),randi([1,20])];
    end
    
    % Loop of the genetic algorithm
    for i = 1:iters
        
        % Evaluation of the fitness function, calculated as the absolute
        % difference between the reference silhouette and the extracted
        % one, counting the misclassified pixels
        for j = 1:numChrom
            currChrom = chromosomes{j};
            currSilh = grayscaleSubtraction(bkgs,{img},...
                         struct('ch','r','value',currChrom(1)),...
                         struct('ch','g','value',currChrom(2)),...
                         struct('ch','b','value',currChrom(3)));
            diff = imabsdiff(refSilh,boolean(currSilh{1}));
            fitness(j) = sum(diff(:));
        end
        
        totFitness = sum(fitness);
        
        % Assign a weight to each chromosome, depending on its fitness. 
        % Since we are dealing with a low number of alleles for each
        % chromosome, roulette wheel selection is used.
        fitness(1) = fitness(1)/totFitness;
        for j = 2:numChrom
            fitness(j) = fitness(j)/totFitness + fitness(j-1);
        end
        
        oldChromosomes = chromosomes;
        for j = 1:numChrom/2
            p1 = oldChromosomes{find(fitness>=rand(), 1 )};
            p2 = oldChromosomes{find(fitness>=rand(), 1 )};
            
            % Crossover: randomly select a gene and swap it with the 
            % correspondant of the other chromosome
            if rand()>= 0.8
                idx = randi([1 3]);
                [p1(idx), p2(idx)] = deal(p2(idx),p1(idx));
            end
            
            % Mutation: randomly select a gene and change its value
            % incrementing or decrementing of 0,1 or 2.
            for k = 1:3
                if rand()>=0.99
                    p1(k) = p1(k) + randi([-2 2]);
                    if p1(k) > 20
                        p1(k) = 20;
                    end
                    if p1(k) < 1
                        p1(k) = 1;
                    end
                end
                
                if rand()>=0.99
                    p2(k) = p2(k) + randi([-2 2]);
                    if p2(k) > 20
                        p2(k) = 20;
                    end
                    if p2(k) < 1
                        p2(k) = 1;
                    end
                end
            end
            
            % Put the children in the new chromosome pool
            chromosomes{2*j-1} = p1;
            chromosomes{2*j} = p2;
        end
    end
    
    % Once the last pool is populated, evaluate the last time the fitness
    % of the chromosome and select the best one as solution of the
    % assignment problem
    for j = 1:numChrom
           currChrom = chromosomes{j};
           currSilh = grayscaleSubtraction(bkgs,{img},...
                        struct('ch','r','value',currChrom(1)),...
                        struct('ch','g','value',currChrom(2)),...
                        struct('ch','b','value',currChrom(3)));
           diff = imabsdiff(refSilh,boolean(currSilh{1}));
           fitness(j) = sum(diff(:));
    end
    
    idx = find(fitness>=max(fitness),1);
    t = num2cell(chromosomes{idx});
    [rT,gT,bT] = deal(t{:});
end