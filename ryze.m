classdef (Sealed, CaseInsensitiveProperties=true, TruncatedProperties=true) ryze < matlab.mixin.CustomDisplay & dynamicprops & handle
%   Connect to a Ryze® Tello drone in MATLAB.
%
%   Syntax:
%       r = ryze
%       r = ryze(Name/ID)
%
%   Description:
%       r = ryze                Creates connection to a Ryze® Tello drone over Wi-Fi.
%       r = ryze(Name/ID)       Creates connection to the specified Ryze® Tello drone over Wi-Fi.
%
%   Example:
%   Connect to a Ryze® Tello drone:
%       r = ryze('Tello');
%
%   Connect to a specific Ryze® Tello drone:
%       r = ryze('TELLO-D31670');
%
%   Input Arguments:
%   Name/ID - Name or unique ID of the Ryze drone
%
%   Output Arguments:
%   r - Ryze drone hardware connection
%
%   See also takeoff, land

%   Copyright 2019-2020 The MathWorks, Inc.

    properties(SetAccess = private)

        % Name of the Ryze drone
        Name

        % Unique ID of the Ryze drone
        ID

        % Current flying state of the drone. Ryze Tello drone firmware doesn't send
        % this information
        State

        % Percentage of battery charge remaining
        BatteryLevel

        % Available cameras on the drone
        AvailableCameras
    end

    properties(Access = private, Hidden)

        % IP Address of the Ryze drone
        IPAddress

        % UDP port of the drone
        DronePort

        % Host port to which sensor data packets are sent by the drone
        HostPort

        % Secondary port to which drone sends on demand drone data
        DroneDataHostPort

        % FPV camera address
        FPVCameraAddress

        % Host port to which video packets are sent by the drone
        VideoStreamPort

        % Flag indicating low battery
        LowBattery

        % Current speed of the drone(in m/s)
        Speed

        % Current altitude of the drone above takeoff surface(in m)
        Altitude

        % Euler angles representing the rotation from the NED frame (determined at drone startup) to the estimated drone body frame
        Attitude

        % Distance limits for move commands of ryze tello
        DistanceLimits

        % Speed limits for move commands of ryze tello
        SpeedLimits

        % Flag to indicate if the first packet of sensor data has arrived
        FirstPacketReceived

        % Status to indicate if an OK was sent by the drone in response to
        % a control command
        CommandAck

        % Status to indicate if an error was sent by the drone in response
        % to a control command
        CommandError

        % Time out for drone commands
        CommandTimeout

        % Parser output of constructor inputs
        ParserOutput

        % Counter to determine drone disconnection
        NoDataReceivedCount

        % Flag that indicates if drone's UDP connection is still active
        ConnectionActive

        % Flag to indicate if MATLAB is still receiving data from drone
        IsTelloConnected

        % Timer that keeps reading sensor data packet from the drone
        SensorTimerObj

        % Timer that keeps reading drone status data packet from the drone
        DroneDataTimerObj
    end

    properties(SetAccess = private, Hidden)

        % UDP socket handle to send commands to the drone and to receive
        % the stream of sensor data
        DroneConnection

        % UDP connection handle to receive on-demand drone status data like
        % the SSID
        SecondaryDataConnection
    end

    properties(Access = private, Constant, Hidden)

        % Packet to be sent to drone in non-SDK mode to retrieve SSID
        SSIDPacket = uint8([204 88 0 124 72 17 0 1 0 27 136]);
    end
    methods
        %% get and set methods
        function battery =  get.BatteryLevel(obj)
        % Get function for BatteryLevel to prevent internal process stalling while reading this property in a loop

            battery = obj.BatteryLevel;
            drawnow;
        end

        function state =  get.State(obj)
        % Get function for State property to convert enum to char

            state = lower(string(obj.State));
            drawnow;
        end
    end

    methods(Access = public)
        %% Constructor
        function obj = ryze(varargin)

            try
                narginchk(0,1);

                % Register message catalog
                ryzeio.internal.Utility.loadResources();

                % Validate the MATLAB preference for H.264 decoding
                % This function will create a new MATLAB_H264 preference if one doesn't exist
                initH264pref(obj);

                % Property initializations
                initProperties(obj, varargin{:});

                % Perform handshake and establish UDP connection with drone
                connectDrone(obj);

                % Setup drone
                setupDrone(obj);

            catch e
                delete(obj);
                throwAsCaller(e);
            end
        end
    end

    methods(Access = protected)
        %% Destructor
        function delete(obj)

        % Shut down the drone motors first to ensure that the drone
        % lands when the user clears the drone connection during flight
            if(obj.ConnectionActive && obj.IsTelloConnected)
                try
                    land(obj);
                catch
                end
            end

            % Stop timer and clear timer object
            if(~isempty(obj.SensorTimerObj) && isvalid(obj.SensorTimerObj))
                obj.SensorTimerObj.stop();
                obj.SensorTimerObj = [];
            end

            % Close UDP sensor data connection
            if(~isempty(obj.DroneConnection))
                % Stop video stream and close connection
                write(obj.DroneConnection, uint8('streamoff'));
                unregister(obj.DroneConnection);
                close(obj.DroneConnection);
            end

            % Update drone connection status to false
            if(obj.ConnectionActive)
                droneConnection(obj,'write', false);
            end
        end
    end

    methods(Access = public)
        %% Drone control functions

        function takeoff(obj)
        %   Drone takeoff function
        %
        %   Syntax:
        %       takeoff(r)
        %
        %   Description:
        %       Takes off the Ryze drone from the ground.
        %
        %   Example:
        %       r = ryze('tello');
        %       takeoff(r);
        %
        %   Input Arguments:
        %   r    - ryze tello object
        %
        %   See also land, abort

            try
                narginchk(1,1);

                % Display takeoff command failed to execute due to low battery(g1904001)
                if(obj.LowBattery)
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:insufficientCharge', 'takeoff');
                end

                % Send takeoff command and wait for ACK from the drone
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('takeoff'));
                try
                    waitForCommandAck(obj);
                catch e
                    if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneFlying', 'takeoff');
                    else
                        throwAsCaller(e);
                    end
                end
                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
            catch e
                throwAsCaller(e);
            end
        end

        function land(obj, varargin)
        %   Drone land function
        %
        %   Syntax:
        %       land(r)
        %
        %   Description:
        %       Initiates smooth land of the drone.
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       land(r);
        %
        %   Input Arguments:
        %   r    - ryze tello object
        %
        %   See also takeoff, abort

            try
                narginchk(1,1);

                % Send land command and wait for ACK from the drone
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('land'));
                try
                    waitForCommandAck(obj);
                catch e
                    if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneLanded', 'land');
                    else
                        throwAsCaller(e);
                    end
                end
                obj.State = ryzeio.internal.DroneStateEnum.Landed;
            catch e
                throwAsCaller(e);
            end
        end

        function abort(obj)
        %   Function to abort drone flight
        %
        %   Syntax:
        %       abort(r)
        %
        %   Description:
        %       Shuts down drone motors instantaneously. Execute takeoff command
        %       to start flying the drone again.
        %
        %       This command works only with a Tello EDU and errors out if executed on a standard
        %       Tello.
        %
        %   Example:
        %       r = ryze("TelloEDU");
        %       takeoff(r);
        %       abort(r);
        %
        %   Input Arguments:
        %   r    - ryze tello object
        %
        %   See also takeoff, land

            try
                % Error out if abort function is tried on a standard Tello
% comentado-->  %if(~strcmpi(obj.Name, "TelloEDU"))
% comentado-->  %    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:unsupportedFunction', 'abort');
% comentado-->  %end
                narginchk(1,1);

                % Send abort command and wait for ACK from the drone
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('emergency'));
                waitForCommandAck(obj);
                obj.State = ryzeio.internal.DroneStateEnum.Landed;
            catch e
                throwAsCaller(e);
            end
        end

        %% Navigation data read methods

        function [speed, time] = readSpeed(obj)
        %   Function to read current speed(in m/s) of the drone
        %
        %   Syntax:
        %       [speed, time] = readSpeed(r);
        %
        %   Description:
        %       Function to read current speed of the drone in m/s,
        %       with respect to the North-East-Down frame, estimated at
        %       drone startup.
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       [speed, time] = readSpeed(r);
        %
        %   Input Arguments:
        %   r       - ryze tello object
        %
        %   Output Arguments:
        %   speed   - 1*3 double vector of drone speed(in m/s) along X, Y, and Z axes
        %   time    - timestamp in date-time format
        %
        %   See also readHeight, readOrientation, land

            try
                narginchk(1,1);
                % Tello drone returns the speed in dm/s
                speed   = obj.Speed.Value/10;
                time = obj.Speed.Time;
                drawnow;
            catch e
                throwAsCaller(e);
            end
        end

        function [height, time] = readHeight(obj)
        %   Function returns the height(in m) of the drone relative to the
        %   takeoff surface
        %
        %   Syntax:
        %       [height, time] = readHeight(r);
        %
        %   Description:
        %       Function to read current height of the drone in m
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       [height, time] = readHeight(r);
        %
        %   Input Arguments:
        %   r       - ryze tello object
        %
        %   Output Arguments:
        %   height  - height of the drone from takeoff surface (in m)
        %   time    - timestamp in date-time format
        %
        %   See also readSpeed, readOrientation, land

            try
                narginchk(1,1);
                % Drone returns the height in cm
                height  = obj.Altitude.Value/100;
                time = obj.Altitude.Time;
                drawnow
            catch e
                throwAsCaller(e);
            end
        end

%--->	% FUNÇÃO CRIADA PARA UTILIZAÇÃO DO SENSOR INFRA VERMELHO (IR)
	function [height_ired, time] = readHeightIR(obj)
	    try
                narginchk(1,1);
                % Drone returns the height in cm
                height_ired  = obj.Altitude.ValueIR/100;
                time = obj.Altitude.Time;
                drawnow
            catch e
                throwAsCaller(e);
            end
        end

        function [angles, time] = readOrientation(obj)
        %   Function to read current orientation of the drone with respect to the
        %   NED frame calculated at drone startup.
        %
        %   Syntax:
        %       [angles, time] = readOrientation(r);
        %
        %   Description:
        %       Function to read current orientation of the drone with respect to the
        %       NED frame calculated at drone startup. 0 radians along Z-axis being
        %       the direction the drone faces during startup.
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       [angles, time] = readOrientation(r);
        %
        %   Input Arguments:
        %   r       - ryze object
        %
        %   Output Arguments:
        %   angles  - 1*3 vector of Euler rotation angles along ZYX axes (in radians)
        %   time    - timestamp in date-time format
        %
        %   See also readSpeed, readHeight, land

            try
                narginchk(1,1);
                angles  = deg2rad(obj.Attitude.Value);
                time = obj.Attitude.Time;
                drawnow
            catch e
                throwAsCaller(e);
            end
        end

        %% Navigation functions

        function moveforward(obj, varargin)
        %   Function to move the drone forward by the specified
        %   duration (in s) or distance(in m)
        %
        %   Syntax:
        %       moveforward(r)
        %       moveforward(r, duration)
        %       moveforward(r, duration, Name, Value)
        %       moveforward(r, Name, Value)
        %
        %   Description:
        %       Function to move the drone forward by the specified
        %       duration (in s) or distance(in m).
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       moveforward(r,'Distance',3,'Speed',0.5);
        %
        %   Input Arguments:
        %   r           - ryze object
        %   duration    - Duration(in s) for which the drone has to move
        %
        %   Name-Value pairs:
        %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
        %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
        %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
        %
        %   See also moveback, land, abort, turn

            try
                narginchk(1, 7);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'forward', varargin);
            catch e
                throwAsCaller(e);
            end
        end

        function moveback(obj, varargin)
        %   Function to move the drone back by the specified
        %   duration (in s) or distance(in m)
        %
        %   Syntax:
        %       moveback(r)
        %       moveback(r, duration)
        %       moveback(r, duration, Name, Value)
        %       moveback(r, Name, Value)
        %
        %   Description:
        %       Function to move the drone back by the specified
        %       duration (in s) or distance(in m).
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       moveback(r,'Distance',3,'Speed',0.5);
        %
        %   Input Arguments:
        %   r           - ryze object
        %   duration    - Duration(in s) for which the drone has to move
        %
        %   Name-Value pairs:
        %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
        %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
        %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
        %
        %   See also moveright, land, abort, turn

            try
                narginchk(1, 7);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'back', varargin);
            catch e
                throwAsCaller(e);
            end
        end

        function moveright(obj, varargin)
        %   Function to move the drone right by the specified
        %   duration (in s) or distance(in m)
        %
        %   Syntax:
        %       moveright(r)
        %       moveright(r, duration)
        %       moveright(r, duration, Name, Value)
        %       moveright(r, Name, Value)
        %
        %   Description:
        %       Function to move the drone right by the specified
        %       duration (in s) or distance(in m).
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       moveright(r,'Distance',3,'Speed',0.5);
        %
        %   Input Arguments:
        %   r           - ryze object
        %   duration    - Duration(in s) for which the drone has to move
        %
        %   Name-Value pairs:
        %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
        %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
        %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
        %
        %   See also moveleft, land, abort, turn

            try
                narginchk(1, 7);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'right', varargin);
            catch e
                throwAsCaller(e);
            end
        end

        function moveleft(obj, varargin)
        %   Function to move the drone left by the specified
        %   duration (in s) or distance(in m)
        %
        %   Syntax:
        %       moveleft(r)
        %       moveleft(r, duration)
        %       moveleft(r, duration, Name, Value)
        %       moveleft(r, Name, Value)
        %
        %   Description:
        %       Function to move the drone forward by the specified
        %       duration (in s) or distance(in m).
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       moveleft(r,'Distance',3,'Speed',0.5);
        %
        %   Input Arguments:
        %   r           - ryze object
        %   duration    - Duration(in s) for which the drone has to move
        %
        %   Name-Value pairs:
        %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
        %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
        %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
        %
        %   See also moveup, land, abort, turn

            try
                narginchk(1, 7);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'left', varargin);
            catch e
                throwAsCaller(e);
            end
        end

        function moveup(obj, varargin)
        %   Function to move the drone up by the specified
        %   duration (in s) or distance(in m)
        %
        %   Syntax:
        %       moveup(r)
        %       moveup(r, duration)
        %       moveup(r, duration, Name, Value)
        %       moveup(r, Name, Value)
        %
        %   Description:
        %       Function to move the drone up by the specified
        %       duration (in s) or distance(in m).
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       moveup(r,'Distance',3,'Speed',0.5);
        %
        %   Input Arguments:
        %   r           - ryze object
        %   duration    - Duration(in s) for which the drone has to move
        %
        %   Name-Value pairs:
        %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
        %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
        %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
        %
        %   See also movedown, land, abort, turn

            try
                narginchk(1, 7);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'up', varargin);
            catch e
                throwAsCaller(e);
            end
        end

        function movedown(obj, varargin)
        %   Function to move the drone down by the specified
        %   duration (in s) or distance(in m)
        %
        %   Syntax:
        %       movedown(r)
        %       movedown(r, duration)
        %       movedown(r, duration, Name, Value)
        %       movedown(r, Name, Value)
        %
        %   Description:
        %       Function to move the drone down by the specified
        %       duration (in s) or distance(in m).
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       movedown(r,'Distance',3,'Speed',0.5);
        %
        %   Input Arguments:
        %   r           - ryze object
        %   duration    - Duration(in s) for which the drone has to move
        %
        %   Name-Value pairs:
        %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
        %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
        %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
        %
        %   See also moveup, land, abort, turn

            try
                narginchk(1, 7);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'down', varargin);
            catch e
                throwAsCaller(e);
            end
        end

        function move(obj, varargin)
        %   Function to move the drone along 3 axis at the same time
        %
        %   Syntax:
        %       move(r, relativeDistance)
        %       move(r, relativeDistance, Name, Value)
        %
        %   Description:
        %       Function to move the drone by the relative coordinate
        %       position
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       move(r, [1 1 -1], 'Speed', 0.5);
        %
        %   Input Arguments:
        %   r                 - ryze object
        %   relativeDistance  - Relative coordinate value in [X,Y,Z]
        %
        %   Name-Value pairs:
        %   Speed             - Name value pair to control speed of move (default = 0.4m/s). Range: 0.1m/s - 1m/s
        %   WaitUntilDone     - Name Value pair to control blocking or non-blocking mode of move (default = true)
        %
        %   See also takeoff, land, abort, turn

            try
                narginchk(2,6);

                % Validate if the input parameters are within range
                [coordinate, speed, blocking] = ryzeio.internal.Utility.validateMoveParams(varargin, obj.DistanceLimits, obj.SpeedLimits);

                % Convert units to cm
                coordinate = coordinate.*100;
                speed = speed * 100;

                % Set speed and then send move command
                obj.CommandAck = false;
                command = char(sprintf("speed %d", round(speed)));
                write(obj.DroneConnection, uint8(command));
                waitForCommandAck(obj);

                obj.CommandAck = false;
                obj.CommandError = false;
                if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Hovering)))
                    obj.State = ryzeio.internal.DroneStateEnum.Flying;
                end

                % Convert coordinates to NED convention and send move SDK
                % command
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8(char(sprintf("go %d %d %d %d",round(coordinate(1)), round(-1*coordinate(2)), round(-1*coordinate(3)), round(speed)))));

                % Block MATLAB till ACK is received from the drone
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                            if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                            end
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneLanded', 'move');
                        else
                            throwAsCaller(e);
                        end
                    end
                else
                    % Wait to see if there are any errors sent by the drone
                    t = tic;
                    while(toc(t)<0.25)
                        if(obj.CommandError)
                            if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                            end
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneLanded', 'move');
                        end
                        pause(0.05);
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        function turn(obj, angle)
        %   Function to turn the drone by a relative angle in radians
        %
        %   Syntax:
        %       turn(r, angle)
        %
        %   Description:
        %       Turns the drone by the specified angle in radians
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       turn(r, deg2rad(-45));
        %
        %   Input Arguments:
        %   r       - ryze object
        %   angle   - angle in radians
        %
        %   See also takeoff, land, abort

            try
                narginchk(2, 2);

                % Validate the angle and convert it to degrees
                angle = rad2deg(angle);
                ryzeio.internal.Utility.validateAngle(angle, [-360 360]);

                obj.CommandAck = false;
                if(angle >= 0)
                    write(obj.DroneConnection, uint8(sprintf('cw %d', round(angle))));
                else
                    write(obj.DroneConnection, uint8(sprintf('ccw %d', round(abs(angle)))));
                end
                try
                    waitForCommandAck(obj);
                catch e
                    if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneLanded', 'turn');
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        function flip(obj, direction)
        %   Drone flip function
        %
        %   Syntax:
        %       flip(r, direction)
        %
        %   Description:
        %       Flips the drone in the desired direction
        %
        %   Example:
        %       r = ryze();
        %       takeoff(r);
        %       flip(r, 'forward');
        %
        %   Input Arguments:
        %       r - ryze tello object
        %       direction - Direction to flip. Possible values are:
        %       ["forward", "back", "left", "right"]
        %
        %   See also takeoff, land, abort

            try
                narginchk(2,2);

                % Validate the flip direction
                direction = ryzeio.internal.Utility.validateFlipDirection(direction);

                obj.CommandAck = false;
                if(strcmpi(direction, 'forward'))
                    write(obj.DroneConnection, uint8('flip f'));
                elseif(strcmpi(direction, 'back'))
                    write(obj.DroneConnection, uint8('flip b'));
                elseif(strcmpi(direction, 'left'))
                    write(obj.DroneConnection, uint8('flip l'));
                elseif(strcmpi(direction, 'right'))
                    write(obj.DroneConnection, uint8('flip r'));
                end
                try
                    waitForCommandAck(obj);
                catch e
                    if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:flipError');
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        function cameraObj = camera(obj, varargin)
        %   This function creates a camera object to access the drone's FPV camera.
        %
        %   camObj = camera(ryzeObj, name) returns a camera object, camObj, that can be used to
        %   access the specified drone camera.
        %
        %   Usage:
        %       Construct a camera object to connect to a Ryze drone camera
        %
        %       camObj = camera(ryzeObj, 'FPV');
        %       OR
        %       camObj = camera(ryzeObj);
        %
        %       % Preview a stream of image frames.
        %       preview(camObj);
        %
        %       % Acquire and display a single image frame.
        %       img = snapshot(camObj);
        %       imshow(img);

            try
                narginchk(1,2);

                % Select the drone's FPV camera by default
                if(1 == nargin)
                    droneCamera = "FPV";
                else
                    % Second input is the drone camera
                    droneCamera = varargin{1};
                end

                % String to char conversion
                droneCamera = char(droneCamera);

                % Validate if the droneCamera inputted is one of the available drone
                % cameras
                try
                    droneCamera = validatestring(droneCamera, obj.AvailableCameras);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:invalidCamera');
                end

                % Throw error if H264 decoding using openh264 library is enabled
                if(~getpref('MATLAB_H264', 'Status'))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:h264Disabled');
                end

                % Start video stream in Tello drone
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('command'));
                try
                    waitForCommandAck(obj);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('streamon'));
                try
                    waitForCommandAck(obj);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                % Create camera addon connection
                cameraObj = ryzeioaddons.Camera(obj.DroneConnection, string(droneCamera), obj.FPVCameraAddress, obj.VideoStreamPort);
            catch e
                % Stop video stream in case of an error
                write(obj.DroneConnection, uint8('streamoff'));
                throwAsCaller(e);
            end
        end
    end

    %% Internal functions
    methods(Access = private)

        function output = parseinputs(~, inputs)
        % Parse the inputs entered by the user

            nInputs = numel(inputs);
            output.Name = [];

            % If the user enters no input argument
            if nInputs == 0
                return;
            end

            % Validate the drone name or drone ID entered by the user
            if nInputs > 0
                name = inputs{1};
                output.Name = ryzeio.internal.Utility.validateDroneName(name);
            end
        end

        function initProperties(obj, varargin)
        % Function to initialize class properties

        % Check if connection already exists in workspace
            status = droneConnection(obj, 'read');
            if(status)
                ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionExists');
            end

            % Parse the input arguments
            obj.ParserOutput = parseinputs(obj, varargin);

            % Check the drone name inputted by the user
            if(isempty(obj.ParserOutput.Name))
                obj.Name = "Tello";
            else
                obj.Name   = string(obj.ParserOutput.Name);
            end

            % Initialize empty drone ID
            obj.ID = '';

            % Initialize State to "Unknown" since this data is not sent by
            % Tello firmware
            obj.State = ryzeio.internal.DroneStateEnum.Landed;

            % Initialize the IPAddress and UDP port to send command packets
            % to the ryze drone
            obj.IPAddress   = '192.168.10.1';
            obj.DronePort   = 8889;
            obj.HostPort    = 8890;
            obj.DroneDataHostPort = 8899;
            obj.FPVCameraAddress = "";
            obj.VideoStreamPort = 11111;

            % Assign initial values for drone settings/properties
            obj.CommandTimeout = 10;
            obj.BatteryLevel = 100;
            obj.FirstPacketReceived = false;
            obj.LowBattery = false;
            obj.ConnectionActive  = false;
            obj.IsTelloConnected = false;

            % Assign initial values for navigation data
            obj.Speed = [];
            obj.Altitude = [];
            obj.Attitude = [];
            obj.DistanceLimits = [0.2 5];
            obj.SpeedLimits = [0.1 1];

            % The ryze Tello drone has only a forward facing camera that can
            % be accessed from MATLAB
            obj.AvailableCameras = ["FPV"];
        end

        function initH264pref(~)
        % Function to check if H.264 support is enabled/disabled
        % Create a new preference, if one does not exist

            if(~ispref('MATLAB_H264', 'Status'))
                addpref('MATLAB_H264', 'Status', true);
            end
        end

        function connectDrone(obj)
        % Function to perform handshake and establish UDP connection with the drone

        % Establish UDP connection to get SSID data(DroneData) and
        % UDP connection to send command and receive sensor data
            try
                obj.DroneConnection = ryzeio.internal.WifiConnection(obj.IPAddress, obj.DronePort, obj.HostPort);
                % Initialize and start timer to read data from the AsyncIO buffer
                obj.SensorTimerObj = internal.IntervalTimer(0.1);
                addlistener(obj.SensorTimerObj, 'Executing', @(~, ~)obj.readSensorData);
                obj.SensorTimerObj.start();

                obj.SecondaryDataConnection = ryzeio.internal.WifiConnection(obj.IPAddress, obj.DronePort, obj.DroneDataHostPort);
                % Initialize and start timer to read data from the AsyncIO
                % buffer for drone's status data like SSID
                obj.DroneDataTimerObj = internal.IntervalTimer(0.5);
                addlistener(obj.DroneDataTimerObj, 'Executing', @(~, ~)obj.readDroneData);
                obj.DroneDataTimerObj.start();
            catch e
                throwAsCaller(e);
            end
        end

        function setupDrone(obj)
        % Function to setup the following after connection:
        %   1. Set the Tello drone in command mode
        %   2. Start a thread to read the battery state every 1s

            try
                % Set the ryze Tello drone in command mode
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('command'));

                % Wait for the ACK from the previous command
                try
                    waitForCommandAck(obj);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                % Wait till the first packet is received to ensure the
                % object display has the latest value for BatteryLevel
                t = tic;
                while(~obj.FirstPacketReceived && toc(t) < obj.CommandTimeout)
                    drawnow
                end

                % Send GET command to read drone's SSID and read the
                % response packet.
                write(obj.SecondaryDataConnection, obj.SSIDPacket);
                t = tic;
                while(toc(t) < obj.CommandTimeout && isempty(obj.ID))
                    drawnow
                end

                % Validate drone ID and assign drone name
                [obj.Name, obj.ID] = ryzeio.internal.Utility.validateDroneID(obj.Name, obj.ID);

                % To make the drone switch from raw packet to SDK mode
                write(obj.DroneConnection, uint8('command'));

                if(isTelloEDU(obj))
                    obj.Name = "TelloEDU";
                else
                    % Error out if user entered TelloEDU and the drone that is connected is Tello
                    if(strcmpi(obj.Name, "TelloEDU"))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:incorrectDroneName', string(obj.Name));
                    end
                    obj.Name = "Tello";
                end

                % We don't need this UDP connection anymore (This was used to only get the drone ID)
                close(obj.SecondaryDataConnection);
                % Stop timer and clear timer object
                if(~isempty(obj.DroneDataTimerObj) && isvalid(obj.DroneDataTimerObj))
                    obj.DroneDataTimerObj.stop();
                    obj.DroneDataTimerObj = [];
                end

                % Update drone connection if connection is successful
                droneConnection(obj, 'write', true);
                obj.ConnectionActive = true;
                obj.IsTelloConnected = true;
            catch e
                throwAsCaller(e);
            end
        end

        %% Functions to read and decode sensor data sent by drone to host port: 8890
        function readSensorData(obj)
        % Function to read the drone sensor data from the Asyncio buffer

        % Issue an error message and stop internal timer if the UDP
        % connection is disconnected or if UDP read fails (g1873914)

            try
                if(~isOpen(obj.DroneConnection))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end
                if(obj.DroneConnection.DataAvailable)
                    try
                        % Reset NoDataReceived counter
                        obj.NoDataReceivedCount = 0;
                        sensorData = read(obj.DroneConnection);
                    catch
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                    end
                    processSensorData(obj, sensorData);
                else
                    % Increment the NoDataReceived counter and error after
                    % 30 retries (30*0.5s = 15s)
                    obj.NoDataReceivedCount = obj.NoDataReceivedCount + 1;
                    if(obj.NoDataReceivedCount >= 30)
                        obj.IsTelloConnected = false;
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionLost');
                    end
                end
            catch e
                obj.SensorTimerObj.stop();
                obj.SensorTimerObj = [];
                throwAsCaller(e);
            end
        end

        function processSensorData(obj, sensorData)
        % Process multiple chunks of data received in single read operation

            if(~isempty(sensorData))
                for i = 1:length(sensorData)
                    decodeSensorData(obj, sensorData(i).Data);
                end
            end
        end

        function decodeSensorData(obj, sensorData)
        % Function to decode the sensor data sent by drone

            try
                dataReceived = char(sensorData);

                if(strcmpi(dataReceived, 'ok'))
                    obj.CommandAck = true;
                    if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                        obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                    end
                elseif(contains(dataReceived, {'error','unknown command'}))
                    % Stay silent about errors thrown by drone when
                    % previous non-blocking command was not run to
                    % completion
                    if(~contains(dataReceived,{'error Auto land', 'error Not joystick'}))
                        obj.CommandError = true;
                    end
                else
                    extractSensorData(obj, dataReceived);
                end
            catch e
                throwAsCaller(e);
            end
        end

        function extractSensorData(obj, sensorData)
        % Function to extract individual sensor data from the packet
        % containing all drone sensor data

        % Update battery level
            obj.BatteryLevel = str2double(extractBetween(sensorData,'bat:',';'));
            if(obj.BatteryLevel < 10)
                status = ryzeio.internal.Utility.validateAlertState(obj.LowBattery, obj.BatteryLevel);
                % Set the low battery flag
                if(status)
                    obj.LowBattery = true;
                end
            end

            % Update Attitude/Orientation Values
            Pitch = str2double(extractBetween(sensorData,'pitch:',';'));
            Roll = str2double(extractBetween(sensorData,'roll:',';'));
            Yaw = str2double(extractBetween(sensorData,'yaw:',';'));
            obj.Attitude.Value = [Yaw Pitch Roll];
            obj.Attitude.Time = datetime('now', 'TImeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

            % Update the Altitude/height of the drone
            obj.Altitude.Value = str2double(extractBetween(sensorData,';h:',';'));
            obj.Altitude.Time = datetime('now', 'TImeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

%--->       % VARIAVEL CRIADA PARA ARMAZENAR LEITURA DO IR
            obj.Altitude.ValueIR = str2double(extractBetween(sensorData,';tof:',';'));

            % Update the speed of the drone
            vgx = str2double(extractBetween(sensorData,'vgx:',';'));
            vgy = str2double(extractBetween(sensorData,'vgy:',';'));
            vgz = str2double(extractBetween(sensorData,'vgz:',';'));
            obj.Speed.Value = [vgx vgy vgz];
            obj.Speed.Time = datetime('now', 'TImeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

            if(~obj.FirstPacketReceived)
                % Make the 'FirstPacketReceived' true to indicate
                % that the first packet of sensor data has arrived
                obj.FirstPacketReceived = true;
            end
        end

        function basicMoveImpl(obj, direction, varargin)
        % Function to implement all basic move commands

            try
                % Validate input parameters of basic move APIs
                [distance, speed, blocking] = ryzeio.internal.Utility.validateBasicMoveParams(varargin{:}, obj.DistanceLimits, obj.SpeedLimits);

                % Validate the direction of movement
                direction = ryzeio.internal.Utility.validateMoveDirection(direction);

                % Convert distance to cm
                distance  = distance * 100;
                speed = speed * 100;

                % Set speed and then send move command
                obj.CommandAck = false;
                command = char(sprintf("speed %d", round(speed)));
                write(obj.DroneConnection, uint8(command));
                waitForCommandAck(obj);

                obj.CommandAck = false;
                obj.CommandError = false;
                if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Hovering)))
                    obj.State = ryzeio.internal.DroneStateEnum.Flying;
                end

                command = char(sprintf("%s %d", direction, round(distance)));
                write(obj.DroneConnection, uint8(command));

                % Block MATLAB till ACK is received from the drone
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                            if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                            end
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneLanded', sprintf('move%s',direction));
                        else
                            throwAsCaller(e);
                        end
                    end
                else
                    % Wait to see if there are any errors sent by the drone
                    pause(0.1);
                    if(obj.CommandError)
                        if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                            obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                        end
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:droneLanded', sprintf('move%s',direction));
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        %% Functions to read and decode drone status data sent by drone to Host port 8899
        function readDroneData(obj)
        % Function to read the drone status data from the Asyncio buffer

            try
                if(~isOpen(obj.SecondaryDataConnection))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                try
                    droneData = read(obj.SecondaryDataConnection);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end
                processDroneData(obj, droneData);
            catch e
                obj.DroneDataTimerObj.stop();
                obj.DroneDataTimerObj = [];
                throwAsCaller(e);
            end
        end

        function processDroneData(obj, droneData)
        % Process multiple chunks of drone status data received in single read operation

            if(~isempty(droneData))
                for i = 1:length(droneData)
                    decodeDroneData(obj, droneData(i).Data);
                end
            end
        end

        function decodeDroneData(obj, droneData)
        % Function to decode the drone SSID data sent by drone

            try
                if(17 == typecast([droneData(6) droneData(7)], 'uint16'))
                    obj.ID = string(char(droneData(12:end-2)));
                end
            catch e
                throwAsCaller(e);
            end
        end

        function status = droneConnection(~, operation, varargin)
        % Function to update the status of drone connection
        % true indicates an active connection

            persistent isDroneConnected
            if(strcmpi(operation, 'write'))
                isDroneConnected = varargin{1};
            elseif(strcmpi(operation, 'read'))
                status = isDroneConnected;
            end
        end

        function waitForCommandAck(obj)
        % Function to wait till an ACK is sent by the Tello drone in
        % response to a control command

            obj.CommandError = false;
            t = tic;
            while(toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                if(obj.CommandError)
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                end
                drawnow;
            end

            % Send error status if ACK is not received even after timeout
            if(~obj.CommandAck)
                ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandFailed');
            end
        end

        function status = isTelloEDU(obj)
        % Function to check if the Tello drone connected is the EDU version
        % This can be verified by sending the command "mon" to enable
        % mission pad control. This command will return an ACK in Tello EDU
        % and "unknown command" in a standard Tello

            status = false;
            obj.CommandAck = false;
            obj.CommandError = false;
            write(obj.DroneConnection, uint8('mon'));
            t = tic;
            while(toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                if(obj.CommandError)
                    status = false;
                    return;
                end
                drawnow;
            end
            % If a command ACK was received, it implies we have a Tello EDU
            % Turn off mission pad control
            if(obj.CommandAck)
                status = true;
                obj.CommandAck = false;
                write(obj.DroneConnection, uint8('moff'));
                t = tic;
                while(toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                    if(obj.CommandError)
                        return;
                    end
                    drawnow;
                end
            end
        end
    end

    methods (Access = public, Hidden)
        % Disable and hide the methods below:

        % Hidden methods from the hgsetget super class.
        function res = eq(obj, varargin)
            res = eq@handle(obj, varargin{:});
        end

        function result = fieldnames(obj)
            result = obj.fieldnames@handle();
        end

        function result = fields(obj)
            result = obj.fields@handle();
        end

        function res = findobj(obj, varargin)
            res = findobj@handle(obj, varargin{:});
        end

        function res = findprop(obj, varargin)
            res = findprop@handle(obj, varargin{:});
        end

        function res = addlistener(obj, varargin)
            res = addlistener@handle(obj, varargin{:});
        end

        function res = notify(obj, varargin)
            res = notify@handle(obj, varargin{:});
        end

        % Hidden methods from the dynamic properties superclass
        function res = addprop(obj, varargin)
            res = addprop@dynamicprops(obj, varargin{:});
        end

        function result = permute(obj,varargin)
            [result] = obj.permute@handle(varargin{:});
        end

        function result = reshape(obj,varargin)
            [result] = obj.reshape@handle(varargin{:});
        end

        function result = transpose(obj)
            [result] = obj.transpose@handle();
        end

        % Unsupported functions
        function c = horzcat(varargin)
            try
                if (nargin == 1)
                    c = varargin{1};
                else
                    ryzeio.internal.Utility.throwUnsupportedError;
                end
            catch e
                throwAsCaller(e);
            end
        end

        function c = vertcat(varargin)
            try
                if (nargin == 1)
                    c = varargin{1};
                else
                    ryzeio.internal.Utility.throwUnsupportedError;
                end
            catch e
                throwAsCaller(e);
            end
        end

        function c = cat(varargin)
            try
                if (nargin > 2)
                    ryzeio.internal.Utility.throwUnsupportedError;
                else
                    c = varargin{2};
                end
            catch
                throwAsCaller(e);
            end
        end

        function ge(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function gt(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function le(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function lt(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function ne(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function listener(~)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end
    end

    methods(Access = protected)
        function displayScalarObject(obj)
        % Function for  custom object display

            header = getHeader(obj);
            disp(header);

            % Display all class properties
            showAllProperties(obj);

            % Allow for the possibility of a footer.
            footer = getFooter(obj);
            if ~isempty(footer)
                disp(footer);
            end
        end
    end

    methods(Hidden, Access = public)
        function showAllProperties(obj)
        % Function to display all properties of the ryze class

            fprintf('                   Name: "%s"\n', obj.Name);
            fprintf('                     ID: "%s"\n', obj.ID);
            fprintf('                  State: "%s"\n', char(obj.State));
            fprintf('           BatteryLevel: %d %%\n', obj.BatteryLevel);
            fprintf('       AvailableCameras: ["%s"]\n', obj.AvailableCameras(1));
            fprintf('\n');
        end

        function showFunctions(~)
        % Function to display all the functions available in the ryze
        % class

            fprintf('\n');

            fprintf('   takeoff\n');
            fprintf('   land\n');
            fprintf('   abort\n');
            fprintf('   moveforward\n');
            fprintf('   moveback\n');
            fprintf('   moveright\n');
            fprintf('   moveleft\n');
            fprintf('   moveup\n');
            fprintf('   movedown\n');
            fprintf('   move\n');
            fprintf('   turn\n');
            fprintf('   readSpeed\n');
            fprintf('   readHeight\n');
            fprintf('   readHeightIR\n'); % ACRESCENTEI
            fprintf('   readOrientation\n');
            fprintf('   flip\n');
            fprintf('   camera\n');

            fprintf('\n');
        end
    end
end
