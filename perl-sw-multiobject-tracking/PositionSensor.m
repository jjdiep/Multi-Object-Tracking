function z=PositionSensor(t)
% Function used to simulate sensor readings at timestep 'k'
% Particles are spawned at [250,250] & [-250,-250] and die at [500,-1000] &
% [1000,-500] after 100s. They meet at 50 s.
%  A third particle is spawned after 66s at [575,-415] and dies at 100s.
            %
            %   Inputs:
            %       t     - current timestep 's'
            %
            %   Outputs:
            %       z     - Sensor reading at time 't' s
            %
            %   Author: Sidddharth Ratapani Navin
            %   Date:   10/28/2018
    z(1,:)=mvnrnd([-250+12.7475*t*0.9806;-250+12.7475*t*-0.1961],10*eye(2)); % Sample sensor readings from the normal distrubtion with sigma=10 m/s^2
    z(2,:)=mvnrnd([250+12.7475*t*0.1961;250+12.7475*t*-0.9806],10*eye(2));
    if(t>66) % Spsawned Particle Sensing
        z(3,:)=mvnrnd([417.4+12.7475*(t-67)*-0.92;-587+12.7475*(t-67)*-0.39],10*eye(2));
    end
end