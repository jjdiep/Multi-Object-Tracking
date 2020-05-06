function [PHDupdate,P]= UpdatePHDComponents(object)
% Function that calculates the values of the PHD update paramters for each of the
% predicted estimates
            %
            %   Inputs:
            %       object    - Predicted object state estimate
            %
            %   Outputs:
            %       PHDupdate - Pre-calculated Update Components
            %       P         - Updated Covariance estimate  
            %
            %   Author: Siddharth Ratapani Navin
            %   Date:   10/28/2018
%% Calculate PHD Update Components           
    H=eye(3);
    R=100*eye(3); 
    PHDupdate.eta=H*object.mu;
    PHDupdate.S=R+H*object.P*H';
    PHDupdate.S=(PHDupdate.S+PHDupdate.S')/2; % hack because MATLAB thinks it is not symmtric because of digit precision
    PHDupdate.K = object.P * H' * (PHDupdate.S \ eye(size(PHDupdate.S))); 
%% Update Covariance
    I = eye(size(object.P));
    P = (I - PHDupdate.K * H) * object.P * (I - PHDupdate.K * H)' ...
        + PHDupdate.K * R * PHDupdate.K'; % Joseph update form
end