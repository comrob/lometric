classdef ErrorStats
    %ERRORSTATS Statistic indicator holder
    %   Storage of the statistic indicators
    
    properties
        minErr;
        maxErr;
        avgErr;
        rmsErr;
    end
    
    methods
        function obj = ErrorStats( minErr, maxErr, avgErr, rmsErr )
            %ERRORSTATS Construct an instance of this class
            %   Basic constructor to init indicators
            obj.minErr = minErr;
            obj.maxErr = maxErr;
            obj.avgErr = avgErr;
            obj.rmsErr = rmsErr;
        end
        
       % function obj = ErrorStats( values )
            %ERRORSTATS Construct an instance of this class
            %   Basic constructor to init indicators
       %     obj.minErr = minErr;
       %     obj.maxErr = maxErr;
       %     obj.avgErr = avgErr;
       %     obj.rmsErr = rmsErr;
       % end
        
        function outputArg = compute(obj,vals)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

