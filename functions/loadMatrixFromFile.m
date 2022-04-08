function [Result_f] = loadMatrixFromFile(filename_f, varargin)
% LOAD_MATRIX_FILE : this function load matrix from file
% params are: [filename], [head lines] (opt), [delimiter] (opt)

argCount = size(varargin,2)+1;
switch argCount
    case 2
        head_lines_f = cell2mat(varargin(1));  		
        delimiterIn = ' ';
    case 3
        head_lines_f = cell2mat(varargin(1));
        delimiterIn = cell2mat(varargin(2));  	
    otherwise
        head_lines_f = 0;
        delimiterIn = ' ';
end


tmp = importdata(filename_f,delimiterIn,head_lines_f);

    if(head_lines_f==0)
        Result_f = tmp;
    else
        Result_f = tmp.data;
    end
end