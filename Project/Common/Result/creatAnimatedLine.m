classdef creatAnimatedLine < matlab.System
    % Create AnimatedLine to demonstrate the trend of the data
    %
    % display the data by the iterator
    %
    % Copyright 2018 The MathWorks, Inc.
%% Properties
    properties(Nontunable)
        Anline = {};
        num = 0;
        color = [1, 0, 0; 0, 1, 0; 0, 0, 1; 1, 1, 0; 1, 0, 1]; % 'red', 'green', 'blue', 'yellow', 'magenta'
        linestyle = {'--','-',':','-.'};
    end
%% Methods    
    methods
        % initialize the object with the number of the lines
        % limit the axis with the input array
        function obj = creatAnimatedLine(N,varargin)
            obj.num = N;
            for i = 1:obj.num
                obj.Anline{i} = animatedline('Color', obj.color(i, :), 'Linestyle', obj.linestyle{i});
            end
            axis([min(varargin{1}(1,:)), max(varargin{1}(1,:)) + 0.2, min(varargin{1}(2,:)), max(varargin{1}(2,:)) + 0.2]);
        end
    end
    
    methods(Access = protected)
        % update the object with the array you want draw
        function stepImpl(obj, varargin)
            for i = 1:obj.num
                addpoints(obj.Anline{i}, varargin{i}(1), varargin{i}(2))
            end
            drawnow
        end
    end
end

