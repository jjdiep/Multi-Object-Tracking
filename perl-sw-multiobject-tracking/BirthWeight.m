function birth_weight= BirthWeight(x)
% Function returns the value of the weights of the new born entities at
% [250,250] & [-250,-250]
%
            %   Inputs:
            %       x            - current state estimate
            %
            %   Outputs:
            %       birth_weight - weight assigned to newborn entities
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
    m1_gamma=[-250,-250,0,0]';
    m2_gamma=[250,250,0,0]';
    P_gamma=diag([100,100,25,25]');
    birth_weight=0.1*mvnpdf(x,m1_gamma,P_gamma)+0.1*mvnpdf(x,m2_gamma,P_gamma);

end
