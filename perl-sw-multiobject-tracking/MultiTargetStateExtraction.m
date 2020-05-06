function Extracted_x= MultiTargetStateExtraction(x_k)
% Function used to Extract the final state of the objects at timestep 'k'
            %
            %   Inputs:
            %       x_k     - current state estimate at time-step k
            %
            %   Outputs:
            %       Extracted_x - state estimate after weight thresholding
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
%% Table 3 - Multi-Target State Extraction
l=1;
for i=1:length(x_k)
    % Check if weight is greater than preset threshold
    if(x_k(i).weight>0.0001)
        Extracted_x(l)=x_k(i);
        l=l+1;
    end
end
end