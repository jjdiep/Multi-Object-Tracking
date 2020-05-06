function Pruned_x_k=PruningAndMerging(x_k,T,U,J_max)
% Function used to Prune and Merge the updated state estimates x_k 
            %
            %   Inputs:
            %       x_k     - Current updated state esimate at timestep k
            %       T       - Truncation threshold
            %       U       - Merging threshold for Mahalanobis distance
            %       J_max   - Maximum number of particles allowed in the
            %                 pruned state estimate
            %
            %   Outputs:
            %       Pruned-x_k       - Pruned state estimate
            %   Author: Siddharth Ratapani Navin
            %   Date:   10/28/2018
%% TABLE 2 - Pruning for the Guassian Mixture PHD Filter
    %% Find indices of all weights above the threshold weight T
    
    index=1;
    for i=1:length(x_k)
        if(x_k(i).weight>T)
            I(index)=i;
            index=index+1;
        end    
    end
    l=0;
    
    %% Merging Loop for Merging state estimates with a Mahalanobis Distance < U
    
    while(~isempty(I))
        l=l+1;
        max_weight=0;
        j=0;
        % Find index of max weight
        for i=1:length(I)
            if(x_k(I(i)).weight>max_weight)
                max_weight=x_k(I(i)).weight;
                j=I(i);
            end
        end
      
        % Find set of objects that you can merge
        merge_set_size=1;
        L=[];
        for i=1:length(I)
            % Calculate Mahalanobis Distance between each object and the
            % object with max weight to merge it
            Mah_d=(x_k(I(i)).mu-x_k(j).mu)'*(x_k(I(i)).P\(x_k(I(i)).mu-x_k(j).mu));
            % Each set L consists of all the indices of objects with
            % similiar estimates
            if(Mah_d<=U)
                L(merge_set_size)=I(i);
                merge_set_size=merge_set_size+1;
            end
        end
        % Calculate weight for new merged member
        Pruned_x_k(l).weight=0;
        for i=1:length(L)
            Pruned_x_k(l).weight=Pruned_x_k(l).weight+x_k(L(i)).weight;
        end
        %Calculate the state vector for the merged member by taking a
        %weighted mean
        Pruned_x_k(l).mu=zeros(3,1);
        for i=1:length(L)
            Pruned_x_k(l).mu=Pruned_x_k(l).mu+x_k(L(i)).weight*x_k(L(i)).mu;
        end
        Pruned_x_k(l).mu=Pruned_x_k(l).mu/Pruned_x_k(l).weight;
        %Calculate the Covariance of the merged member
        Pruned_x_k(l).P=zeros(3,3);
        for i=1:length(L)
            Pruned_x_k(l).P=Pruned_x_k(l).P + x_k(L(i)).weight*(x_k(L(i)).P + (Pruned_x_k(l).mu-x_k(L(i)).mu)*(Pruned_x_k(l).mu-x_k(L(i)).mu)');
        end
        Pruned_x_k(l).P=Pruned_x_k(l).P/Pruned_x_k(l).weight;
        % Remove the merged members from the original array of Indices I
        I=setdiff(I,L);
        
    end
    %% Discarding state estimates if the total number of estimates > 100 
    % Check if number of objects >100 and select only the elements with the
    % top 100 weights
    if(length(Pruned_x_k)>J_max)
        Pruned_x_k=nestedSortStruct(Pruned_x_k, 'weight');
        Pruned_x_k=Pruned_x_k(end-99:end);
    end
end