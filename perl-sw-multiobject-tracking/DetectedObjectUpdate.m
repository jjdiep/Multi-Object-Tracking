function detected_object=DetectedObjectUpdate(object,z,PHDupdate)
% Function used to Update the state estimate with sensor readings  
            %
            %   Inputs:
            %       object      - Object for one single particle
            %       z           - Sensor reading
            %       PHDUpdate   - Object conssting of precalculated
            %                     parameters
            %       
            %   Outputs:
            %       detected_object  - Object for one single particle with
            %                          updated state estimates
            %   Author: Siddharth Ratapani Navin
            %   Date:   10/28/2018
    detected_object.weight=0.98*object.weight*mvnpdf(z',PHDupdate.eta,PHDupdate.S);
    detected_object.mu=object.mu+PHDupdate.K*(z'-PHDupdate.eta);
    detected_object.P=object.P;
end
